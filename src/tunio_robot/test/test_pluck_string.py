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
    """movej/movel 호출을 순서대로 기록한다."""

    DR_BASE = "BASE"
    DR_TOOL = "TOOL"
    DR_MV_MOD_ABS = "ABS"
    DR_MV_MOD_REL = "REL"

    def __init__(self):
        self.calls = []

    def movej(self, pos, **kw):
        self.calls.append(("movej", list(pos), kw))

    def movel(self, pos, **kw):
        self.calls.append(("movel", list(pos), kw))


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


def test_motion_overrides(fake_api):
    motion = {"pluck_up_mm": 25.0, "pluck_stroke_tool": [0, 6, 0, 0, 0, 0],
              "pluck_retract_tool": [0, 0, -10, 0, 0, 0], "pluck_vel_fine": 15}
    MP.pluck_string(_pose_x(CONTACT), motion)
    above, contact, stroke, retract, _ = fake_api.calls
    assert above[1][2] == pytest.approx(CONTACT[2] + 25.0)
    assert contact[2]["vel"] == 15
    assert stroke[1][:3] == [0, 6, 0]
    assert retract[1][:3] == [0, 0, -10]


def test_rejects_joint_only_pose(fake_api):
    with pytest.raises(MP.MotionError, match="x"):
        MP.pluck_string(Pose("pluck.6", True, joints=[0] * 6), {})
    assert fake_api.calls == []                                 # 움직이지 않았다


def test_rejects_short_retract(fake_api):
    with pytest.raises(MP.MotionError, match="5mm"):
        MP.pluck_string(_pose_x(CONTACT), {"pluck_retract_tool": [0, 0, -3, 0, 0, 0]})
    assert fake_api.calls == []


def test_requires_connect(monkeypatch):
    monkeypatch.setattr(MP, "_api", None)
    with pytest.raises(MP.MotionError, match="connect"):
        MP.pluck_string(_pose_x(CONTACT), {})
