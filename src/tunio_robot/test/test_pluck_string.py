"""
pluck_string 단위 테스트. 두산 API 를 가짜 객체로 바꿔 꽂고 호출 순서만 검증한다.

로봇·ROS 없이 돈다:  cd src/tunio_robot && python3 -m pytest test/test_pluck_string.py -q
"""
import sys
import types

import pytest

from tunio_robot import motion_primitives as MP
from tunio_robot.teach_points import Pose, Route as TPRoute


class FakeApi:
    """
    movej/movel 호출을 순서대로 기록한다.

    반환은 실물 규약을 따른다 — 성공 0, 거부 -1 (DSR_ROBOT2.py 2941행).
    None 을 돌려주면 반환값 검사가 무조건 통과해서 테스트가 보증이 못 된다
    (CLAUDE.md 3.5.1 "대역 API 는 실제 반환 규약을 흉내 내야 한다").
    reject 에 호출 순번(0부터)을 넣으면 그 호출만 -1 로 거부한다.
    """

    DR_BASE = "BASE"
    DR_TOOL = "TOOL"
    DR_MV_MOD_ABS = "ABS"
    DR_MV_MOD_REL = "REL"

    def __init__(self):
        self.calls = []
        self.reject = set()

    def _record(self, kind, pos, kw):
        self.calls.append((kind, list(pos), kw))
        return -1 if len(self.calls) - 1 in self.reject else 0

    def movej(self, pos, **kw):
        return self._record("movej", pos, kw)

    def movel(self, pos, **kw):
        return self._record("movel", pos, kw)

    def movejx(self, pos, **kw):
        return self._record("movejx", pos, kw)


@pytest.fixture
def fake_api(monkeypatch):
    # DR_common2 는 두산 설치본에만 있다. posx/posj 를 리스트 그대로 돌려주는 가짜로 대체
    fake_common = types.ModuleType("DR_common2")
    fake_common.posx = lambda v: list(v)
    fake_common.posj = lambda v: list(v)
    monkeypatch.setitem(sys.modules, "DR_common2", fake_common)

    api = FakeApi()
    monkeypatch.setattr(MP, "_api", api)
    return api


def _pose_x(x):
    return Pose("pluck.6", True, task=list(x))


CONTACT = [642.93, -29.60, 185.00, 57.46, -175.68, 150.31]


def test_sequence_without_standby(fake_api):
    MP.pluck_string(_pose_x(CONTACT), {})
    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel", "movel", "movel", "movel", "movel"]

    above, contact, stroke, retract, back = fake_api.calls
    assert above[1][2] == pytest.approx(CONTACT[2] + 20.0)      # 접촉점 위 2cm (Base +Z)
    assert above[2]["ref"] == "BASE"
    assert contact[1] == CONTACT                                # 접촉점
    assert stroke[1][:3] == [-8.0, 0.0, 0.0]                    # 옆으로 (Tool 상대)
    assert stroke[2]["ref"] == "TOOL" and stroke[2]["mod"] == "REL"
    assert stroke[2]["radius"] == 3                             # 후퇴와 블렌딩
    assert retract[1][:3] == [0.0, 0.0, -8.0]                   # 즉시 후퇴
    assert retract[2]["ref"] == "TOOL" and retract[2]["mod"] == "REL"
    assert back[1] == above[1]                                  # 다시 2cm 위


def test_sequence_with_standby(fake_api):
    standby = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pluck_string(_pose_x(CONTACT), {"vel": 30, "acc": 30}, standby=standby)
    kinds = [c[0] for c in fake_api.calls]
    assert kinds[0] == "movej" and kinds[-1] == "movej"         # 앞뒤로 대기 자세
    assert fake_api.calls[0][1] == [0, 0, 75, 0, 100, 0]
    assert kinds[1:-1] == ["movel"] * 5


def test_sequence_with_task_standby(fake_api):
    """기타 위 대기 자세가 작업 좌표(x)면 movej 가 아니라 Base 직선(movel)으로 간다."""
    standby = Pose("frames.pluck_standby", True,
                   task=[641.83, -13.17, 192.0, 180.0, -180.0, -90.0])
    MP.pluck_string(_pose_x(CONTACT), {}, standby=standby)
    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel"] * 7                               # 대기 자세까지 전부 movel
    assert fake_api.calls[0][1] == standby.task
    assert fake_api.calls[0][2]["ref"] == "BASE"
    assert fake_api.calls[-1][1] == standby.task


def test_motion_overrides(fake_api):
    motion = {"pluck_up_mm": 25.0, "pluck_stroke_tool": [0, 6, 0, 0, 0, 0],
              "pluck_retract_tool": [0, 0, -10, 0, 0, 0], "pluck_vel_fine": 15}
    MP.pluck_string(_pose_x(CONTACT), motion)
    above, contact, stroke, retract, _ = fake_api.calls
    assert above[1][2] == pytest.approx(CONTACT[2] + 25.0)
    assert contact[2]["vel"] == 15
    assert stroke[1][:3] == [0, 6, 0]
    assert retract[1][:3] == [0, 0, -10]


def test_stroke_sign_flips_direction(fake_api):
    """2·1번은 고음 쪽 바깥이라 6·5번과 반대로 훑는다."""
    p = _pose_x(CONTACT)
    p.stroke_sign = -1
    MP.pluck_string(p, {"pluck_stroke_tool": [8, 0, 0, 0, 0, 0]})
    stroke = fake_api.calls[2]
    assert stroke[1][:3] == [-8.0, 0.0, 0.0]                  # 부호만 뒤집힌다
    assert fake_api.calls[3][1][:3] == [0.0, 0.0, -8.0]       # 후퇴는 그대로


def test_rejects_bad_stroke_sign(fake_api):
    p = _pose_x(CONTACT)
    p.stroke_sign = 2
    with pytest.raises(MP.MotionError, match="stroke_sign"):
        MP.pluck_string(p, {})
    assert fake_api.calls == []


def test_rejects_joint_only_pose(fake_api):
    with pytest.raises(MP.MotionError, match="x"):
        MP.pluck_string(Pose("pluck.6", True, joints=[0] * 6), {})
    assert fake_api.calls == []                                 # 움직이지 않았다


def test_approach_rejection_stops_before_contact(fake_api):
    """접촉점 위로 가는 이동이 거부되면 거기서 멈춘다 — 내려가지 않는다 (#50)."""
    fake_api.reject = {0}                                       # 첫 이동(위 2cm)을 거부
    with pytest.raises(MP.MotionError, match="movel_base"):
        MP.pluck_string(_pose_x(CONTACT), {})
    assert len(fake_api.calls) == 1                             # 접촉점으로 안 내려갔다


def test_stroke_rejection_still_lifts_off_the_string(fake_api):
    """
    스트로크가 거부돼도 줄에서 떼고 나서 예외를 올린다 (#50).

    이 구간은 피크가 줄에 닿아 있다. 그냥 예외로 빠지면 진동하는 줄을 누른 채
    멈춰서 버징이 나고 그 줄 측정이 망가진다 (CLAUDE.md 3.4).
    """
    fake_api.reject = {2}                                       # 스트로크를 거부
    with pytest.raises(MP.MotionError, match="movel_tool_rel"):
        MP.pluck_string(_pose_x(CONTACT), {})

    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel"] * 4                               # 위 → 접촉 → 스트로크 → 들어올림
    above = fake_api.calls[0]
    assert fake_api.calls[-1][1] == above[1]                    # 마지막은 접촉점 위로 복귀
    assert fake_api.calls[-1][2]["ref"] == "BASE"


def test_retract_rejection_also_lifts_off(fake_api):
    """후퇴가 거부된 경우도 같다. 피크는 스트로크 끝점에서 줄에 닿아 있다."""
    fake_api.reject = {3}                                       # 후퇴를 거부
    with pytest.raises(MP.MotionError, match="movel_tool_rel"):
        MP.pluck_string(_pose_x(CONTACT), {})
    assert len(fake_api.calls) == 5                             # 들어올림 한 번이 더 붙는다
    assert fake_api.calls[-1][1] == fake_api.calls[0][1]


def test_lift_off_failure_keeps_the_original_error(fake_api):
    """들어올림마저 거부되면 원래 실패를 올린다 — 덮어쓰지 않는다."""
    fake_api.reject = {2, 3}                                    # 스트로크와, 그다음 들어올림
    with pytest.raises(MP.MotionError, match="movel_tool_rel"):
        MP.pluck_string(_pose_x(CONTACT), {})
    assert len(fake_api.calls) == 4                             # 들어올림은 시도했다
    assert fake_api.calls[3][2]["ref"] == "BASE"                # 그게 들어올림이었다


def test_rejects_short_retract(fake_api):
    with pytest.raises(MP.MotionError, match="5mm"):
        MP.pluck_string(_pose_x(CONTACT), {"pluck_retract_tool": [0, 0, -3, 0, 0, 0]})
    assert fake_api.calls == []


def test_requires_connect(monkeypatch):
    monkeypatch.setattr(MP, "_api", None)
    with pytest.raises(MP.MotionError, match="connect"):
        MP.pluck_string(_pose_x(CONTACT), {})


# ---------------------------------------------------------- 피크 집기·반환
class FakeGripper:
    def __init__(self):
        self.calls = []

    def select(self, name):
        self.calls.append(name)


@pytest.fixture
def fake_gripper(monkeypatch):
    g = FakeGripper()
    monkeypatch.setattr(MP, "_gripper", g)
    return g


HOLDER = [684.64, -199.20, 175.72, 133.40, -174.14, -134.71]


def test_pick_up_sequence(fake_api, fake_gripper):
    holder = Pose("pick.holder", True, task=HOLDER)
    holder.grip_preset = "peg"
    standby = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_up(holder, {"pick_up_mm": 50.0}, standby=standby)

    assert fake_gripper.calls == ["open", "peg"]            # 열고 → 잡는다
    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel", "movel", "movel", "movej"]    # 위 → 거치대 → 위 → 대기
    above, contact, up, _ = fake_api.calls
    assert above[1][2] == pytest.approx(HOLDER[2] + 50.0)
    assert contact[1] == HOLDER
    assert up[1] == above[1]


def test_pick_return_sequence(fake_api, fake_gripper):
    holder = Pose("pick.holder", True, task=HOLDER)
    home = Pose("frames.home", True, joints=[0, 0, 90, 0, 90, 0])
    MP.pick_return(holder, {}, home=home)

    assert fake_gripper.calls == ["open"]                   # 놓기만 한다
    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel", "movel", "movel", "movej"]    # 위 → 거치대 → 위 → 홈
    assert fake_api.calls[-1][1] == [0, 0, 90, 0, 90, 0]


def test_pick_preset_defaults_to_close(fake_api, fake_gripper):
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {})
    assert fake_gripper.calls == ["open", "close"]


def test_pick_return_with_via(fake_api, fake_gripper):
    """발현 직후(기타 위 대기)에서 거치대로 바로 movel 하지 않고 경유 자세를 먼저 거친다."""
    holder = Pose("pick.holder", True, task=HOLDER)
    via = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_return(holder, {}, via=via)

    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movej", "movel", "movel", "movel"]        # 경유 → 위 → 거치대 → 위
    assert fake_api.calls[0][1] == [0, 0, 75, 0, 100, 0]


def test_pick_up_with_via(fake_api, fake_gripper):
    """
    피크를 가지러 갈 때도 경유 자세를 먼저 거친다 (#34 후속).

    반대 방향(pick_return)은 PR #38 에서 막혔는데 이쪽은 비어 있었다.
    팔은 직전 동작이 남긴 자리에 있고, 페그 조절 직후면 헤드스톡 쪽이다.
    거기서 거치대 위로 바로 movel 하면 기타 넥을 가로지른다.
    """
    holder = Pose("pick.holder", True, task=HOLDER)
    via = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_up(holder, {"pick_up_mm": 50.0}, via=via)

    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movej", "movel", "movel", "movel"]        # 경유 → 위 → 거치대 → 위
    assert fake_api.calls[0][1] == [0, 0, 75, 0, 100, 0]


def test_pick_up_via_runs_before_the_straight_descent(fake_api, fake_gripper):
    """경유는 직선 하강보다 먼저다. 뒤로 가면 막으려던 그 직선을 그대로 탄다."""
    via = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, via=via)
    assert fake_api.calls[0][0] == "movej"                      # 관절 보간이 먼저
    assert fake_gripper.calls == ["open", "close"]              # 그다음 열고 잡는다


def test_pick_up_without_via_keeps_old_path(fake_api, fake_gripper):
    """경유를 안 주면 예전 그대로 돈다. 도구들이 이 경로를 쓴다."""
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {})
    kinds = [c[0] for c in fake_api.calls]
    assert kinds == ["movel", "movel", "movel"]                 # 경유 없음


def test_pick_rejects_joint_only_holder(fake_api, fake_gripper):
    with pytest.raises(MP.MotionError, match="x"):
        MP.pick_up(Pose("pick.holder", True, joints=[0] * 6), {})
    assert fake_api.calls == [] and fake_gripper.calls == []


def test_pick_up_descent_rejected_does_not_close_the_gripper(fake_api, fake_gripper):
    """
    거치대로 내려가는 이동이 거부되면 그리퍼를 닫지 않는다 (#50).

    이게 #50 이 짚은 제일 위험한 자리다 — 반환을 안 보면 안 내려갔는데도 다음
    줄에서 손가락이 닫혀 **빈손을 물고 "집었다"** 가 된다. 놓친 피크는 비전이
    없어 되찾을 수 없다 (CLAUDE.md 2절).
    """
    fake_api.reject = {1}                                      # 내려가는 이동을 거부
    with pytest.raises(MP.MotionError, match="movel_base"):
        MP.pick_up(Pose("pick.holder", True, task=HOLDER), {})

    assert fake_gripper.calls == ["open"]                       # 열기만 했고 닫지 않았다
    assert len(fake_api.calls) == 2                            # 들어 올리지도 않았다


def test_pick_return_descent_rejected_does_not_open_the_gripper(fake_api, fake_gripper):
    """반환도 같다 — 안 내려갔는데 손을 펴면 피크를 거치대 밖에 떨군다 (#50)."""
    fake_api.reject = {1}
    with pytest.raises(MP.MotionError, match="movel_base"):
        MP.pick_return(Pose("pick.holder", True, task=HOLDER), {})
    assert fake_gripper.calls == []                             # 펴지 않았다


# ---------------------------------------------------------- 이동 속도 배율 (#56)
def test_speed_scale_multiplies_travel_but_not_the_stroke(fake_api):
    """
    배율은 **이동·접근에만** 걸린다.

    스트로크는 피크가 줄을 긁는 구간이라 소리 세기가 여기서 정해진다.
    2026-09-20 에 4줄 소리가 확인된 값이 100/200 이고, PR #54 가 코드 기본값
    1000/2000 과 갈려 있던 걸 막 고쳤다. 배율이 여기까지 먹으면 그 수정이
    통째로 무효가 된다.
    """
    MP.pluck_string(_pose_x(CONTACT), {"speed_scale": 10.0,
                                       "pluck_vel_stroke": 100, "pluck_acc_stroke": 200})
    above, contact, stroke, _retract, _back = fake_api.calls
    assert above[2]["vel"] == 250.0                     # 100 x 10, 상한 250 에서 잘림
    assert contact[2]["vel"] == 20                      # fine 은 안 걸린다
    assert stroke[2]["vel"] == 100                      # 검증값 그대로
    assert stroke[2]["acc"] == 200


def test_speed_scale_is_capped(fake_api):
    """상한을 넘기지 않는다. 배율을 크게 줘도 직선은 SPEED_CAP_LINEAR 에서 잘린다."""
    MP.pluck_string(_pose_x(CONTACT), {"speed_scale": 1000.0})
    assert fake_api.calls[0][2]["vel"] == MP.SPEED_CAP_LINEAR
    assert fake_api.calls[0][2]["acc"] == MP.SPEED_CAP_LINEAR * 2


def test_no_speed_scale_keeps_old_numbers(fake_api):
    """안 주면 예전 그대로다. 도구들이 자기 dict 로 이 경로를 쓴다."""
    MP.pluck_string(_pose_x(CONTACT), {})
    assert fake_api.calls[0][2]["vel"] == 100


def test_turn_motion_refuses_the_scale():
    """페그 회전은 배율을 안 탄다. 줄에 장력을 거는 동작이라 느린 게 설계다."""
    out = MP.turn_motion({"speed_scale": 10.0, "turn_vel": 2.0, "turn_acc": 4.0})
    assert out["speed_scale"] == 1.0
    assert out["vel"] == 2.0 and out["acc"] == 4.0


# ---------------------------------------------------------- 대기 자세 건너뛰기 (#56)
class PosxApi(FakeApi):
    """get_current_posx 를 흉내 낸다. 두산 래퍼는 (posx, sol) 로 준다."""

    def __init__(self, xyz):
        super().__init__()
        self.xyz = list(xyz)

    def get_current_posx(self):
        return (self.xyz + [0.0, 0.0, 0.0])[:6], 2


PLUCK_STANDBY_J = [-1.69, 44.18, 31.30, 0.13, 104.52, 90.34]


def _standby_j():
    return Pose("frames.pluck_standby", True, joints=PLUCK_STANDBY_J)


def test_standby_is_skipped_when_the_arm_is_already_there(monkeypatch, fake_api):
    """
    발현 접근점은 대기 자세에서 12~22mm 다. 거기서 대기 자세로 가 봐야 제자리다.

    그런데 관절 이동 한 번 값을 그대로 낸다 — 발현마다 두 번(앞·뒤) 붙는다.
    2026-09-21 통합 실기에서 사용자가 지적한 그 왕복이다.
    """
    from tunio_robot import kinematics as K
    here = K.fk(PLUCK_STANDBY_J)["tcp"]
    api = PosxApi(here)
    monkeypatch.setattr(MP, "_api", api)

    MP.pluck_string(_pose_x(CONTACT), {}, standby=_standby_j())
    assert [c[0] for c in api.calls] == ["movel"] * 5        # movej 가 하나도 없다


def test_standby_still_runs_when_the_arm_is_far(monkeypatch, fake_api):
    """멀면 그대로 간다. 거치대에서 오는 경로가 이쪽이다."""
    api = PosxApi([591.63, -143.75, 372.0])                 # 거치대 위
    monkeypatch.setattr(MP, "_api", api)

    MP.pluck_string(_pose_x(CONTACT), {}, standby=_standby_j())
    kinds = [c[0] for c in api.calls]
    assert kinds[0] == "movej" and kinds[-1] == "movej"


def test_standby_runs_when_the_position_cannot_be_read(fake_api):
    """못 읽으면 건너뛰지 않는다. 모를 때는 가는 쪽이 안전하다."""
    # FakeApi 에는 get_current_posx 가 없다
    MP.pluck_string(_pose_x(CONTACT), {}, standby=_standby_j())
    assert [c[0] for c in fake_api.calls][0] == "movej"


def test_standby_skip_can_be_turned_off(monkeypatch, fake_api):
    """0 이면 예전 동작이다."""
    from tunio_robot import kinematics as K
    api = PosxApi(K.fk(PLUCK_STANDBY_J)["tcp"])
    monkeypatch.setattr(MP, "_api", api)

    MP.pluck_string(_pose_x(CONTACT), {"standby_skip_mm": 0.0}, standby=_standby_j())
    assert [c[0] for c in api.calls][0] == "movej"


# ---------------------------------------------------------- 거치대 팔 모양 (#56)
ROUTE = TPRoute("routes.transit", [90.0, 0.0, 0.0, 0.0, 0.0, None], True)


class SolApi(FakeApi):
    """
    지금 관절각과 ikin 을 흉내 낸다.

    ikin 은 sol 마다 다른 J1 을 돌려준다 — 팔 모양 전환 판단이 J1 각으로 이뤄지므로
    그 값이 가짜에도 있어야 뜻이 있다 (CLAUDE.md 3.5.1, 가짜는 실제 규약을 흉내 낸다).
    """

    J1_OF_SOL = {0: -10.0, 2: -10.0, 3: 5.0, 6: 170.0, 7: 175.0}

    def __init__(self, now_j1=-10.0):
        super().__init__()
        self.now_j1 = now_j1

    def get_current_posj(self):
        return [self.now_j1] + [0.0] * 5

    def ikin(self, pos, sol):
        return [self.J1_OF_SOL.get(sol, 0.0), -30.0, -90.0, 0.0, 60.0, 0.0]


def test_pick_up_with_sol_uses_joint_interpolation(monkeypatch, fake_api, fake_gripper):
    """
    Sol 을 주면 거치대 위 대기점까지 movejx 로 간다.

    movel 은 지금 팔 모양을 유지하므로 IK 해의 갈래를 못 바꾼다. 거치대 접근이
    J1 오른쪽 해 하나뿐이라 6·5번(왼쪽) 페그로 갈 때마다 경유지를 타던 것이
    이 갈래 전환으로 없어진다 (2026-09-21 실기 지적).
    """
    api = SolApi(now_j1=-10.0)
    monkeypatch.setattr(MP, "_api", api)
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, sol=3, route=ROUTE)
    kinds = [c[0] for c in api.calls]
    assert kinds == ["movejx", "movel", "movel"]        # J1 -10 -> 5, 15도라 경유 없음
    assert api.calls[0][2]["sol"] == 3
    assert api.calls[1][2]["ref"] == "BASE"             # 하강은 직선 그대로
    assert api.calls[2][1] == api.calls[0][1]           # 수직으로 올라온다


def test_pick_return_with_sol_uses_joint_interpolation(monkeypatch, fake_api, fake_gripper):
    """반환 쪽이 더 중요하다 — 끝나면 바로 페그를 잡으러 간다."""
    api = SolApi(now_j1=170.0)
    monkeypatch.setattr(MP, "_api", api)
    MP.pick_return(Pose("pick.holder", True, task=HOLDER), {}, sol=6, route=ROUTE)
    assert [c[0] for c in api.calls] == ["movejx", "movel", "movel"]
    assert api.calls[0][2]["sol"] == 6


def test_J1_이_뒤집히면_경유지를_지난다(monkeypatch, fake_api, fake_gripper):
    """
    J1 이 뒤집히는 전환은 **어느 쪽으로 가든** J1=90 을 거쳐야 한다.

    2026-09-21 사용자 확인 — J1 이 뒤집히는 전환은 어느 쪽으로 가든 J1=90 을
    거쳐야 기타에 안 부딪힌다. 반대값은 150~180도쯤 차이 나는 경우다.
    경유지를 지나면 via(frames.standby)는 건너뛴다 — 더 안전한 출발점이다.
    """
    api = SolApi(now_j1=-10.0)                          # 목표 sol 6 은 J1 170 -> 180도
    monkeypatch.setattr(MP, "_api", api)
    via = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, via=via, sol=6, route=ROUTE)
    assert api.calls[0][0] == "movej" and api.calls[0][1][0] == 90.0     # 경유지로 갔고
    assert [c[1] for c in api.calls].count([0, 0, 75, 0, 100, 0]) == 0   # via 는 안 들렀다


def test_J1_이_비슷하면_경유지를_안_지난다(monkeypatch, fake_api, fake_gripper):
    """
    Sol 번호가 달라도 J1 이 그대로면 경유가 낭비다.

    sol 은 어깨·팔꿈치·손목 세 비트라 팔꿈치나 손목만 바뀌는 전환이 있다.
    기타를 가로지르는 건 J1 이 뒤집힐 때뿐이다.
    """
    api = SolApi(now_j1=-10.0)                          # 목표 sol 2 도 J1 -10 -> 0도 차이
    monkeypatch.setattr(MP, "_api", api)
    via = Pose("frames.standby", True, joints=[0, 0, 75, 0, 100, 0])
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, via=via, sol=2, route=ROUTE)
    assert api.calls[0][0] == "movej"
    assert api.calls[0][1] == [0, 0, 75, 0, 100, 0]     # 경유지가 아니라 via 다


def test_230도_차이도_경유지를_지난다(monkeypatch, fake_api, fake_gripper):
    """Movej 는 관절 공간에서 곧이곧대로 돈다. 230도를 130도로 접어 보면 안 된다."""
    api = SolApi(now_j1=-60.0)                          # 목표 sol 6 은 J1 170 -> 230도
    monkeypatch.setattr(MP, "_api", api)
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, sol=6, route=ROUTE)
    assert api.calls[0][0] == "movej" and api.calls[0][1][0] == 90.0


def test_문턱을_yaml_로_바꿀_수_있다(monkeypatch, fake_api, fake_gripper):
    """실기에서 재 보고 조일 수 있어야 한다. 실측 전 임시값이다."""
    api = SolApi(now_j1=-10.0)                          # sol 6 까지 180도
    monkeypatch.setattr(MP, "_api", api)
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {"j1_flip_deg": 200.0},
               sol=6, route=ROUTE)
    assert api.calls[0][0] == "movejx"                  # 200도 문턱이면 안 뒤집힌 셈


def test_팔_모양을_바꿔야_하는데_경유지가_없으면_거부한다(monkeypatch, fake_api, fake_gripper):
    """J1=90 을 안 거치고 뒤집으면 기타에 부딪힌다. 조용히 넘어가면 안 된다."""
    api = SolApi(now_j1=-10.0)
    monkeypatch.setattr(MP, "_api", api)
    with pytest.raises(MP.MotionError, match="경유지가 없다"):
        MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, sol=6)
    assert api.calls == []                              # 움직이지 않았다


def test_J1_을_못_읽으면_경유지를_지난다(monkeypatch, fake_api, fake_gripper):
    """모르면서 뒤집는 것보다 한 번 더 들르는 쪽이 싸다."""
    class 깜깜(SolApi):
        def ikin(self, pos, sol):
            raise RuntimeError("ikin 서비스 없음")

    api = 깜깜(now_j1=-10.0)
    monkeypatch.setattr(MP, "_api", api)
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {}, sol=6, route=ROUTE)
    assert api.calls[0][0] == "movej" and api.calls[0][1][0] == 90.0


def test_pick_without_sol_keeps_the_straight_approach(fake_api, fake_gripper):
    """안 주면 예전 그대로 직선이다. 값이 null 인 동안 아무것도 안 바뀐다."""
    MP.pick_up(Pose("pick.holder", True, task=HOLDER), {})
    assert [c[0] for c in fake_api.calls] == ["movel"] * 3


# ---------------------------------------------------------- 손끝 속도 상한 (#58)
PEG6_J = [129.28, -47.87, -97.35, 48.66, -115.17, 28.98]    # 9/21 허공 회차 실측
TRANSIT_J = [90.0, 0.0, 0.0, 0.0, 0.0, 28.98]
STANDBY_J = [0.0, 0.0, 75.0, 0.0, 100.0, 0.0]


class PosjApi(FakeApi):
    """get_current_posj 를 흉내 낸다."""

    def __init__(self, joints):
        super().__init__()
        self.joints = list(joints)

    def get_current_posj(self):
        return list(self.joints)


def test_같은_관절속도가_팔_모양에_따라_손끝에서_크게_갈린다():
    """
    Movej 상한을 관절 속도로 거는 것 자체가 틀렸다는 근거다 (#58).

    경유지 자세는 팔이 수직이라 J1 축에서 6mm 밖에 안 떨어져 있고, 페그 자세는
    팔이 뻗어 있다. 같은 관절 속도가 손끝에서 두 자릿수로 갈린다.
    """
    near = MP._peak_tcp_mm_per_deg(TRANSIT_J, [v + 30 for v in TRANSIT_J[:1]] + TRANSIT_J[1:])
    far = MP._peak_tcp_mm_per_deg(PEG6_J, TRANSIT_J)
    assert far > near * 10


def test_팔이_뻗은_구간은_관절속도를_낮춘다(monkeypatch):
    """6번 페그에서 경유지로 가는 구간. 50deg/s 면 손끝이 1.2m/s 다."""
    api = PosjApi(PEG6_J)
    monkeypatch.setattr(MP, "_api", api)
    vel = MP._joint_vel_for(TRANSIT_J, 50.0)
    assert vel < 50.0
    peak = MP._peak_tcp_mm_per_deg(PEG6_J, TRANSIT_J)
    assert vel * peak <= MP.SPEED_CAP_TCP + 1e-6      # 손끝이 상한 안이다


def test_팔이_접힌_구간은_안_낮춘다(monkeypatch):
    """대기 자세 근처는 손끝이 느리다. 쓸데없이 조이지 않는다."""
    api = PosjApi(STANDBY_J)
    monkeypatch.setattr(MP, "_api", api)
    target = [-1.69, 44.18, 31.30, 0.13, 104.52, 90.34]   # pluck_standby
    assert MP._joint_vel_for(target, 50.0) == 50.0


def test_관절각을_못_읽으면_느린_쪽으로_떨어진다(fake_api):
    """기하를 모르면 느린 쪽이다 — FakeApi 에는 get_current_posj 가 없다."""
    assert MP._joint_vel_for(TRANSIT_J, 50.0) == MP.SPEED_CAP_JOINT_BLIND


def test_제자리_이동은_안_낮춘다(monkeypatch):
    """손끝이 안 움직이면 상한이 걸릴 이유가 없다."""
    api = PosjApi(STANDBY_J)
    monkeypatch.setattr(MP, "_api", api)
    assert MP._joint_vel_for(STANDBY_J, 50.0) == 50.0


def test_movej_가_가속도도_같은_비로_내린다(monkeypatch):
    """속도만 내리고 가속도를 두면 그 가속 구간에서 손끝이 상한을 넘는다."""
    api = PosjApi(PEG6_J)
    monkeypatch.setattr(MP, "_api", api)
    MP._movej(TRANSIT_J, {"vel": 50.0, "acc": 100.0})
    kw = api.calls[0][2]
    assert kw["vel"] < 50.0
    assert abs(kw["acc"] / kw["vel"] - 2.0) < 1e-6        # 비가 유지된다


# ---------------------------------------------------------- J1 뒤집힘 판단 (#58)
def test_j1_flip_문턱():
    """
    사용자 기준 — 반대값은 150~180도쯤, 230도도 옮긴다. 그 아래는 근사값이다.

    이 함수 하나가 판단을 쥔다. tools/sol_check.py 도 이걸 불러서 찍으므로
    도구가 보여주는 값이 곧 실기에서 코드가 내릴 판단이다.
    """
    m = {}
    assert MP.j1_flip(-10.0, 5.0, m)[0] is False        # 15도 — 근사값
    assert MP.j1_flip(0.0, 119.0, m)[0] is False        # 119도 — 아직 근사
    assert MP.j1_flip(0.0, 120.0, m)[0] is True         # 120도 — 문턱
    assert MP.j1_flip(-10.0, 170.0, m)[0] is True       # 180도 — 반대값
    assert MP.j1_flip(-60.0, 170.0, m)[0] is True       # 230도 — 반대값


def test_j1_flip_은_모듈러로_접지_않는다():
    """Movej 는 관절 공간에서 곧이곧대로 돈다. 230도를 130도로 보면 안 된다."""
    flip, turn = MP.j1_flip(-60.0, 170.0, {})
    assert turn == 230.0 and flip is True


def test_j1_flip_문턱은_설정으로_바꾼다():
    """실측 전 임시값이라 실기에서 조여야 한다."""
    assert MP.j1_flip(-10.0, 170.0, {"j1_flip_deg": 200.0})[0] is False
    assert MP.j1_flip(-10.0, 170.0, {"j1_flip_deg": 100.0})[0] is True


def test_j1_을_모르면_뒤집는_것으로_본다():
    """모르면서 뒤집는 것보다 한 번 더 들르는 쪽이 싸다."""
    assert MP.j1_flip(None, 170.0, {}) == (True, None)
    assert MP.j1_flip(-10.0, None, {}) == (True, None)
