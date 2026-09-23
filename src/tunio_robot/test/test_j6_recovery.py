"""J6 감시는 조회만, 회복은 접촉 이탈 경계에서만 수행한다."""
import math
import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from tunio_robot import motion_primitives as MP
from tunio_robot.robot_controller import RobotController
from tunio_robot.teach_points import Pose

MOTION = {"j6_recovery_enabled": True, "j6_recover_at_deg": 280.0,
          "peg_j6_limit_deg": 300.0}


@pytest.mark.parametrize("start,end", [(280, 100), (-280, -100), (290, 110), (-290, -110)])
def test_280도에서_양방향_180도_회복(monkeypatch, start, end):
    joints = [0., 0., 0., 0., 0., float(start)]
    monkeypatch.setattr(MP, "_api", SimpleNamespace(get_current_posj=lambda: joints[:]))
    monkeypatch.setattr(MP, "_node", None)
    moves = []

    def move(target, motion):
        moves.append(target)
        joints[:] = target
    monkeypatch.setattr(MP, "_movej", move)
    assert MP._recover_j6(MOTION) == end - start
    assert joints[5] == end
    assert len(moves) == 1


def test_회복미도달이면_실패(monkeypatch):
    monkeypatch.setattr(MP, "_api", SimpleNamespace(get_current_posj=lambda: [0] * 5 + [280]))
    monkeypatch.setattr(MP, "_movej", lambda *args: None)
    with pytest.raises(MP.MotionError, match="미도달"):
        MP._recover_j6(MOTION)


def test_감시콜백은_이동하지_않고_상태만_갱신한다(monkeypatch):
    monkeypatch.setattr(MP, "_j6_sample", None)
    move = Mock()
    monkeypatch.setattr(MP, "_movej", move)
    n = SimpleNamespace(points=SimpleNamespace(motion=MOTION), _j6_pending=False,
                        get_logger=lambda: Mock())
    RobotController._on_j6(n, SimpleNamespace(
        name=["joint_6"], position=[math.radians(285)]))
    assert n._j6_pending
    assert MP._j6_sample[1] == pytest.approx(285)
    move.assert_not_called()


@pytest.mark.parametrize("sample,goal", [(None, 0), ((0, 0), 0), ("fresh", 280)])
def test_감시끊김과_위험목표는_이동전_거부(monkeypatch, sample, goal):
    monkeypatch.setattr(MP, "_j6_motion", MOTION)
    monkeypatch.setattr(MP, "_j6_sample",
                        (time.monotonic(), 0) if sample == "fresh" else sample)
    with pytest.raises(MP.MotionError):
        MP._guard_j6_goal([0] * 5 + [goal])


def test_대칭목표_선택은_부가설정과_다음회전여유를_유지한다(monkeypatch):
    pose = Pose("pluck.2", True, task=[300, 60, 108, 90, -90, 0])
    pose.stroke_sign = -1
    pose.sol = 2
    monkeypatch.setitem(sys.modules, "DR_common2", SimpleNamespace(posx=list))
    monkeypatch.setattr(MP, "_api", SimpleNamespace(
        get_current_posj=lambda: [0] * 5 + [100],
        ikin=lambda task, sol: [0] * 5 + [260 if abs(task[5]) < 1 else 80]))
    selected, offset = MP._j6_pose(pose, MOTION, delta_deg=30)
    assert abs(offset) == 180
    assert selected.stroke_sign == -1
    assert selected.sol == 2
    assert pose.task[5] == 0


@pytest.mark.parametrize("names,positions", [
    ([], []), (["joint_6"], []), (["joint_6"], [float("nan")]),
])
def test_잘못된_관절메시지는_이전값을_재사용하지_않는다(monkeypatch, names, positions):
    monkeypatch.setattr(MP, "_j6_sample", (time.monotonic(), 100))
    n = SimpleNamespace(points=SimpleNamespace(motion=MOTION), _j6_pending=False,
                        get_logger=lambda: Mock())
    RobotController._on_j6(n, SimpleNamespace(name=names, position=positions))
    assert MP._j6_sample is None


def test_J5가_135도를_넘어도_J6_검사는_유지한다(monkeypatch):
    joints = [0., 0., 0., 0., 145., 100.]
    monkeypatch.setattr(MP, "_api", SimpleNamespace(get_current_posj=lambda: joints[:]))
    assert MP._read_peg_joints(MOTION) == joints
    joints[5] = 301.
    with pytest.raises(MP.MotionError, match="J6"):
        MP._read_peg_joints(MOTION)
