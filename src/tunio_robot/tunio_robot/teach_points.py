"""
교시점 YAML 로드·검증과 페그 회전량 변환.

로봇 하드웨어에 의존하지 않는다. 두산 API를 import 하지 않으므로
로봇 없이 단위 테스트로 검증할 수 있다. 실제 이동은 motion_primitives 가 한다.

여기서 하는 일:
  - teach_points.yaml 을 읽고 구조·범위를 검증한다
  - 줄 번호로 페그·발현 자세를 꺼낸다
  - AdjustPeg 의 direction(+1 조임 / -1 풀림)을 실제 관절 회전 부호로 바꾼다
    (명세: "줄별 실제 모터 회전 방향 변환은 로봇측 YAML이 담당한다")

교시 전에는 좌표가 전부 0이다. 그 상태로 움직이면 기타·로봇이 부딪히므로
taught: false 인 항목은 조회 시 거부한다. 교시를 마치면 true 로 바꾼다.
"""
import math
import os

import yaml

# 대상 줄. CLAUDE.md 4절 — 6·5·2·1번만 다룬다
STRINGS = (6, 5, 2, 1)

# AdjustPeg.Goal.direction 규약
TIGHTEN = 1     # 음 높이기
LOOSEN = -1     # 음 낮추기


# 헤드스톡 좌우. 같은 쪽끼리는 가깝고, 반대쪽은 손목이 뒤집혀야 한다
SIDES = {6: "left", 5: "left", 2: "right", 1: "right"}


def side_of(string_number):
    """그 줄의 페그가 헤드스톡 어느 쪽에 있는지."""
    side = SIDES.get(string_number)
    _require(side is not None, f"{string_number}번은 헤드스톡 배치에 없다")
    return side


class Route:
    """
    경유지. 관절 일부만 지정한다.

    None 인 자리는 그때의 현재 관절값을 그대로 쓴다. 부딪히지 않는 데
    필요한 관절(보통 J1 과 J3)만 적으면 된다. 전부 적으면 자세가 완전히
    고정되고, 일부만 적으면 오던 자세를 최대한 유지한 채 그 관절만 옮긴다.
    """

    def __init__(self, name, joints, taught):
        self.name = name
        self.joints = list(joints)
        self.taught = bool(taught)

    def resolve(self, current):
        """현재 관절값을 받아 실제로 명령할 6개 값을 만든다."""
        _require(len(current) == 6, f"현재 관절값이 6개가 아니다: {current}")
        return [current[i] if v is None else float(v)
                for i, v in enumerate(self.joints)]

    def __repr__(self):
        return f"<Route {self.name} {self.joints}>"


class TeachPointsError(Exception):
    """교시점 파일이 잘못됐거나 아직 교시되지 않은 항목을 요청했을 때."""


def _require(cond, msg):
    if not cond:
        raise TeachPointsError(msg)


class Pose:
    """
    교시 자세 하나. 관절각(j) 또는 작업 좌표(x) 중 하나로 적는다.

    j — 관절 1~6 각도(도). 대기 자세처럼 팔 모양 자체가 중요한 곳
    x — BASE 기준 [X, Y, Z, A, B, C]. 위치 mm, 자세 intrinsic ZYZ 도.
        페그처럼 "기타의 이 지점"이 중요한 곳. 좌표계 계산으로 다른 페그를
        내려면 작업 좌표여야 한다

    작업 좌표는 **교시할 때 켜져 있던 TCP 기준**이다. 다른 TCP 로 실행하면
    같은 숫자가 다른 위치를 가리킨다. robot.tcp 를 반드시 맞춰야 한다.
    """

    def __init__(self, name, taught, joints=None, task=None, tcp=None):
        self.name = name
        self.joints = list(joints) if joints is not None else None
        self.task = list(task) if task is not None else None
        self.taught = bool(taught)
        # 이 좌표를 읽을 때 켜져 있던 TCP 이름. 작업 좌표(x)에만 뜻이 있다.
        # None 이면 "안 적어 둬서 모른다" 이고, 그 상태로는 조회가 거부된다.
        self.tcp = tcp
        # 거치대 파지 자세의 팔 모양(IK 해)을 다음에 갈 페그 쪽별로 (#56).
        # pick.holder 에만 뜻이 있다. 값이 null 이면 지금 팔 모양을 그대로 쓴다.
        self.sol_by_side = {"left": None, "right": None}

    @property
    def kind(self):
        return "joint" if self.joints is not None else "task"

    @property
    def values(self):
        return self.joints if self.joints is not None else self.task

    def __repr__(self):
        mark = "" if self.taught else " (미교시)"
        return f"<Pose {self.name} {self.kind} {self.values}{mark}>"


class TeachPoints:
    """teach_points.yaml 한 벌."""

    def __init__(self, data, path=None):
        self.path = path
        self._raw = data
        self.robot = data.get("robot", {})
        self.motion = data.get("motion", {})
        self.gripper = data.get("gripper", {})
        self._frames = {}
        self._pegs = {}
        self._plucks = {}
        self._pick = None
        self._routes = {}
        self._parse()

    # -------------------------------------------------- 파싱·검증
    def _parse(self):
        d = self._raw
        _require(isinstance(d, dict), "최상위가 매핑이 아니다")

        for key in ("robot", "motion", "gripper", "frames", "pegs", "pluck"):
            _require(key in d, f"'{key}' 항목이 없다")

        # 로봇 식별 — DR_init 에 넣을 값. 예제 기준 dsr01 / m0609
        _require(self.robot.get("id"), "robot.id 가 비었다")
        _require(self.robot.get("model"), "robot.model 이 비었다")

        # 페그를 돌리는 관절 번호. 실물 확인 전이라 YAML 로 뺀다
        joint = self.motion.get("peg_joint")
        _require(isinstance(joint, int) and 1 <= joint <= 6,
                 f"motion.peg_joint 는 1~6 이어야 한다 (지금 {joint!r})")

        _require(self.gripper.get("presets"), "gripper.presets 가 비었다")

        limit = self.motion.get("max_angle_deg")
        _require(isinstance(limit, (int, float)) and limit > 0,
                 f"motion.max_angle_deg 는 양수여야 한다 (지금 {limit!r})")

        for name, item in d["frames"].items():
            self._frames[name] = self._pose(f"frames.{name}", item)

        for key, item in d["pegs"].items():
            no = self._string_no("pegs", key)
            self._pegs[no] = self._peg(no, item)

        for key, item in d["pluck"].items():
            no = self._string_no("pluck", key)
            pose = self._pose(f"pluck.{no}", item)
            # 이 줄을 훑는 방향. pluck_stroke_tool 에 곱한다. 저음 쪽과 고음 쪽이
            # 빈 공간이 반대라 부호가 다르다 (2026-09-20 실기 확인)
            sign = item.get("stroke_sign", 1)
            _require(sign in (1, -1),
                     f"pluck.{no}.stroke_sign 은 1 또는 -1 이어야 한다 (지금 {sign!r})")
            pose.stroke_sign = sign
            self._plucks[no] = pose

        missing = [s for s in STRINGS if s not in self._pegs]
        _require(not missing, f"pegs 에 빠진 줄: {missing}")
        missing = [s for s in STRINGS if s not in self._plucks]
        _require(not missing, f"pluck 에 빠진 줄: {missing}")

        # 좌우 전환 경유지. 없으면 경유 없이 바로 간다
        for name, item in (d.get("routes") or {}).items():
            joints = item.get("j")
            _require(isinstance(joints, list) and len(joints) == 6,
                     f"routes.{name}.j 는 값 6개 목록이어야 한다 (지금 {joints!r})")
            for v in joints:
                _require(v is None or isinstance(v, (int, float)),
                         f"routes.{name}.j 는 숫자 아니면 null 이어야 한다")
            self._routes[name] = Route(f"routes.{name}", joints,
                                       item.get("taught", False))

        # 페그 손잡이 실측. 없으면 Phase 0 를 못 돈다 (조회할 때 걸린다)
        self._handle = d.get("peg_handle")

        # 피크 거치대. 없는 YAML 도 로드는 되고, 조회할 때 걸린다
        holder = d.get("pick", {}).get("holder")
        if holder is not None:
            self._pick = self._grip_pose("pick.holder", holder)
            # 거치대 파지 자세의 팔 모양. 페그의 sol 과 같은 뜻이다 (#56).
            #   지금 거치대 접근은 J1 이 오른쪽인 해 하나뿐이라, 피크를 집고
            #   발현한 뒤 6·5번(왼쪽) 페그로 가려면 경유지를 타야 한다.
            #   왼쪽 해를 같이 정해 두면 그 경유를 건너뛸 수 있다.
            # null 이면 지금 팔 모양을 그대로 쓴다 (예전 동작).
            by_side = holder.get("sol_by_side") or {}
            _require(isinstance(by_side, dict),
                     f"pick.holder.sol_by_side 는 매핑이어야 한다 (지금 {by_side!r})")
            for k in by_side:
                _require(k in ("left", "right"),
                         f"pick.holder.sol_by_side 의 키는 left/right 뿐이다 (지금 {k!r})")
            for k, v in by_side.items():
                _require(v is None or (isinstance(v, int) and 0 <= v <= 7),
                         f"pick.holder.sol_by_side.{k} 는 0~7 정수 아니면 null 이어야 한다 "
                         f"(지금 {v!r})")
            self._pick.sol_by_side = {k: by_side.get(k) for k in ("left", "right")}

    @staticmethod
    def _string_no(section, key):
        try:
            no = int(key)
        except (TypeError, ValueError):
            raise TeachPointsError(f"{section} 의 키 {key!r} 가 줄 번호가 아니다")
        _require(no in STRINGS, f"{section} 에 대상 아닌 줄 {no} 이 있다 (대상 {list(STRINGS)})")
        return no

    @staticmethod
    def _pose(name, item):
        _require(isinstance(item, dict), f"{name} 이 매핑이 아니다")
        joints, task = item.get("j"), item.get("x")
        _require((joints is None) != (task is None),
                 f"{name} 은 j(관절각) 또는 x(작업좌표) 중 하나만 있어야 한다")
        values = joints if joints is not None else task
        label = "j" if joints is not None else "x"
        _require(isinstance(values, list) and len(values) == 6,
                 f"{name}.{label} 는 값 6개 목록이어야 한다 (지금 {values!r})")
        for v in values:
            _require(isinstance(v, (int, float)), f"{name}.{label} 에 숫자가 아닌 값이 있다")
        tcp = item.get("tcp")
        _require(tcp is None or isinstance(tcp, str),
                 f"{name}.tcp 는 문자열 아니면 null 이어야 한다 (지금 {tcp!r})")
        return Pose(name, item.get("taught", False), joints=joints, task=task, tcp=tcp)

    def _grip_pose(self, name, item):
        """자세 + 파지 설정(폭·힘)을 묶어 읽는다."""
        pose = self._pose(name, item)
        pose.grip_width_mm = item.get("grip_width_mm")
        pose.grip_force_n = item.get("grip_force_n")
        return pose

    def _peg(self, no, item):
        pose = self._grip_pose(f"pegs.{no}", item)
        # 팔 자세(역기구학 해). 경유지에서 접근점으로 갈 때 movejx 가 쓴다.
        # None 이면 그때의 현재 자세를 유지한다
        sol = item.get("sol")
        _require(sol is None or isinstance(sol, int),
                 f"pegs.{no}.sol 은 정수 아니면 null 이어야 한다 (지금 {sol!r})")
        pose.sol = sol
        sign = item.get("turn_sign")
        _require(sign in (1, -1), f"pegs.{no}.turn_sign 은 1 또는 -1 이어야 한다 (지금 {sign!r})")
        pose.turn_sign = sign
        return pose

    # -------------------------------------------------- 조회
    def frame(self, name):
        pose = self._frames.get(name)
        _require(pose is not None, f"frames 에 '{name}' 이 없다")
        return self._taught(pose)

    def peg(self, string_number):
        _require(string_number in self._pegs, f"pegs 에 {string_number}번 줄이 없다")
        return self._taught(self._pegs[string_number])

    def route(self, name="transit"):
        """
        경유지. 없거나 미교시면 None 이고, 그러면 경유 없이 간다.

        경유 없이 가면 경로가 기타를 쓸고 갈 수 있다. 교시가 끝나면
        taught 를 올려야 한다.
        """
        r = self._routes.get(name)
        return r if (r is not None and r.taught) else None

    def peg_handle(self):
        """
        페그 손잡이 실측값. Phase 0 가 폭으로 물림·사점·빈손을 가를 때 쓴다 (#31).

        실측 전이면 거부한다. 추정값으로 판정하면 빈손을 물림으로 읽어 조용히 틀린다.
        tools/peg_grip_probe.py 로 잰다.
        """
        h = self._handle
        _require(h is not None, "교시점 파일에 peg_handle 이 없다. "
                                "tools/peg_grip_probe.py 로 먼저 재라 (#31)")
        _require(h.get("measured"), "peg_handle.measured 가 false 다. 실측값이 아니다")
        t = h.get("thresholds_mm") or {}
        for key in ("empty_below", "dead_above"):
            _require(isinstance(t.get(key), (int, float)),
                     f"peg_handle.thresholds_mm.{key} 가 없다")
        _require(t["empty_below"] < t["dead_above"],
                 "peg_handle 문턱이 뒤집혀 있다 (empty_below 가 dead_above 보다 크다)")
        return h

    def pick_holder(self):
        """피크 거치대 자세. 피크를 집고 반환하는 자리다."""
        _require(self._pick is not None,
                 "교시점 파일에 pick.holder 가 없다. 피크를 집었다 놓는 방식이라 필요하다")
        return self._taught(self._pick)

    def pluck(self, string_number):
        _require(string_number in self._plucks, f"pluck 에 {string_number}번 줄이 없다")
        return self._taught(self._plucks[string_number])

    def _taught(self, pose):
        """교시가 끝났고 좌표 기준이 맞는 자세만 내준다. 아니면 거부한다."""
        _require(pose.taught,
                 f"{pose.name} 이 아직 교시되지 않았다. 교시 후 YAML 의 taught 를 true 로 바꿔라")
        self._check_tcp(pose)
        return pose

    def _check_tcp(self, pose):
        """
        작업 좌표가 **어느 TCP 로 읽힌 값인지** 확인한다 (이슈 #21).

        posx 는 그때 켜져 있던 TCP 기준의 숫자다. 다른 TCP 로 실행하면 같은 숫자가
        다른 자리를 가리킨다. 2026-09-20 에 실제로 겪었다 — 페그는 GripperDA_v1
        (툴 Z 208mm) 을 켜고 교시했고 발현·거치대는 끄고(플랜지) 교시해서, 한
        파일에 208mm 어긋난 두 기준이 섞여 있었다.

        connect() 가 robot.tcp 하나만 켜므로, 그와 다른 기준으로 적힌 좌표는
        **쓰면 안 된다.** 조용히 움직이는 대신 여기서 멈춘다.

        관절각(j)은 TCP 와 무관하므로 검사하지 않는다.
        기준을 옮기려면 tunio_robot.kinematics.shift_tcp 를 쓴다 — 오프셋이 순수
        평행이동이라 재교시 없이 계산으로 옮겨진다.
        """
        if pose.task is None:
            return
        want = self.robot.get("tcp")
        if not want:
            return
        _require(pose.tcp is not None,
                 f"{pose.name} 에 tcp 가 안 적혀 있다. 이 좌표를 읽을 때 켜져 있던 TCP "
                 f"이름을 적어라 (지금 쓰는 것은 '{want}'). 기준이 다르면 208mm "
                 "어긋난 곳으로 들어간다 (이슈 #21)")
        _require(pose.tcp == want,
                 f"{pose.name} 은 '{pose.tcp}' 기준으로 교시됐는데 지금 켜는 것은 "
                 f"'{want}' 다. 같은 숫자가 다른 자리를 가리킨다. 좌표를 옮기려면 "
                 "kinematics.shift_tcp 를 쓰라 (이슈 #21)")

    @property
    def peg_joint(self):
        """페그를 돌리는 관절 번호 (1~6)."""
        return self.motion["peg_joint"]

    @property
    def max_angle_deg(self):
        """1회 goal 의 회전 상한. 이보다 큰 요청은 거부한다."""
        return float(self.motion["max_angle_deg"])

    def joint_delta(self, string_number, direction, angle_deg):
        """
        페그 조절 goal(AdjustPeg)을 관절 상대 회전량으로 바꾼다.

        direction: +1 조임(음 높이기) / -1 풀림
        angle_deg: 항상 양수 크기
        반환: (관절 번호 1~6, 부호 붙은 회전량 도)

        줄마다 헤드에서의 방향이 달라 같은 "조임"도 관절 부호가 반대일 수 있다.
        그 변환이 pegs.<줄>.turn_sign 이다.
        """
        validate_goal(string_number, direction, angle_deg, self.max_angle_deg)
        peg = self.peg(string_number)
        return self.peg_joint, direction * peg.turn_sign * float(angle_deg)


def approach_axis(task_pose):
    """
    그리퍼가 다가가는 방향(툴 Z)을 BASE 기준 단위벡터로 돌려준다.

    자세는 intrinsic ZYZ. R = Rz(A)Ry(B)Rz(C) 의 3열이 툴 Z 다.
    Rz(C) 는 툴 Z 를 그대로 두므로 C 는 영향이 없다.
    """
    a, b = math.radians(task_pose[3]), math.radians(task_pose[4])
    return (math.sin(b) * math.cos(a), math.sin(b) * math.sin(a), math.cos(b))


def backed_off(task_pose, distance_mm):
    """
    같은 자세로 접근축을 따라 뒤로 물러난 자세를 만든다.

    페그로 바로 가면 경로가 기타를 쓸고 갈 수 있다. 접근축 위에서 물러난
    지점까지 먼저 가고, 거기서 직선으로 들어가면 경로가 예측 가능해진다.
    나올 때도 같은 직선으로 나온다.
    """
    axis = approach_axis(task_pose)
    out = list(task_pose)
    for i in range(3):
        out[i] = task_pose[i] - axis[i] * distance_mm
    return out


def validate_goal(string_number, direction, angle_deg, max_angle_deg):
    """페그 조절 goal 값 검증. 통과하면 None, 아니면 TeachPointsError."""
    _require(string_number in STRINGS,
             f"대상 줄이 아니다: {string_number} (대상 {list(STRINGS)})")
    _require(direction in (TIGHTEN, LOOSEN),
             f"direction 은 +1 또는 -1 이어야 한다 (지금 {direction!r})")
    _require(isinstance(angle_deg, (int, float)), f"angle_deg 가 숫자가 아니다: {angle_deg!r}")
    _require(angle_deg > 0, f"angle_deg 는 양수 크기여야 한다 (지금 {angle_deg})")
    _require(angle_deg <= max_angle_deg,
             f"angle_deg {angle_deg} 가 상한 {max_angle_deg} 를 넘는다")


def load(path):
    """YAML 파일을 읽어 TeachPoints 를 만든다."""
    _require(os.path.exists(path), f"교시점 파일이 없다: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return TeachPoints(data, path=path)
