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
import os

import yaml

# 대상 줄. CLAUDE.md 4절 — 6·5·2·1번만 다룬다
STRINGS = (6, 5, 2, 1)

# AdjustPeg.Goal.direction 규약
TIGHTEN = 1     # 음 높이기
LOOSEN = -1     # 음 낮추기


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

    def __init__(self, name, taught, joints=None, task=None):
        self.name = name
        self.joints = list(joints) if joints is not None else None
        self.task = list(task) if task is not None else None
        self.taught = bool(taught)

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
            self._plucks[no] = self._pose(f"pluck.{no}", item)

        missing = [s for s in STRINGS if s not in self._pegs]
        _require(not missing, f"pegs 에 빠진 줄: {missing}")
        missing = [s for s in STRINGS if s not in self._plucks]
        _require(not missing, f"pluck 에 빠진 줄: {missing}")

        # 피크 거치대. 없는 YAML 도 로드는 되고, 조회할 때 걸린다
        holder = d.get("pick", {}).get("holder")
        if holder is not None:
            self._pick = self._grip_pose("pick.holder", holder)

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
        return Pose(name, item.get("taught", False), joints=joints, task=task)

    def _grip_pose(self, name, item):
        """자세 + 파지 설정(폭·힘)을 묶어 읽는다."""
        pose = self._pose(name, item)
        pose.grip_width_mm = item.get("grip_width_mm")
        pose.grip_force_n = item.get("grip_force_n")
        return pose

    def _peg(self, no, item):
        pose = self._grip_pose(f"pegs.{no}", item)
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

    def pick_holder(self):
        """피크 거치대 자세. 피크를 집고 반환하는 자리다."""
        _require(self._pick is not None,
                 "교시점 파일에 pick.holder 가 없다. 피크를 집었다 놓는 방식이라 필요하다")
        return self._taught(self._pick)

    def pluck(self, string_number):
        _require(string_number in self._plucks, f"pluck 에 {string_number}번 줄이 없다")
        return self._taught(self._plucks[string_number])

    @staticmethod
    def _taught(pose):
        _require(pose.taught,
                 f"{pose.name} 이 아직 교시되지 않았다. 교시 후 YAML 의 taught 를 true 로 바꿔라")
        return pose

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
