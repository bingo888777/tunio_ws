"""
pluck_string 단위 테스트. 두산 API 를 가짜 객체로 바꿔 꽂고 호출 순서만 검증한다.

로봇·ROS 없이 돈다:  cd src/tunio_robot && python3 -m pytest test/test_pluck_string.py -q
"""
import sys
import types

import pytest

from tunio_robot import motion_primitives as MP
from tunio_robot.teach_points import Pose


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
