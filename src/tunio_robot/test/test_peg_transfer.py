"""피크 반환 후 높은 이동 경로. 실제 로봇 동작 없이 하강 순서와 거부를 확인한다."""
import sys
from types import SimpleNamespace

import pytest

from tunio_robot import kinematics as K
from tunio_robot import motion_primitives as MP
from tunio_robot import teach_points as TP


def geometry(z, joint_z=400):
    return {"tcp": [300, 160, z], "origins": [[0, 0, 134.5]] * 2
            + [[0, 0, joint_z]] * 4, "wrist_casing": [0, 0, joint_z]}


@pytest.mark.parametrize("part", ["tcp", "joint", "casing"])
def test_양끝이_높아도_중간에_낮아지면_거부한다(monkeypatch, part):
    def fk(j):
        f = geometry(300)
        if 4 < j[0] < 6:
            if part == "tcp":
                f["tcp"][2] = 200
            elif part == "joint":
                f["origins"][3] = [0, 0, 200]
            else:
                f["wrist_casing"][2] = 200
        return f
    monkeypatch.setattr(K, "fk", fk)
    with pytest.raises(MP.MotionError, match="높이 부족"):
        MP._check_peg_transfer_path([0] * 6, [10, 0, 0, 0, 0, 0], 214, 5)


def test_고정축_J1_J2는_높이_제한에서_제외한다():
    MP._check_peg_transfer_path([0] * 6, [90, 0, 0, 0, 0, 0], 214, 5)


@pytest.fixture
def transfer(monkeypatch):
    pose = TP.Pose("pegs.2", True, task=[300, 60, 108, 90, -90, 180])
    pose.sol = 2
    back = TP.backed_off(pose.task, 100)
    high = list(back)
    high[2] = 224
    sim = SimpleNamespace(joints=[0] * 6, tcp=[400, 0, 224, 0, 180, 0], calls=[])
    goal = [20, 10, 10, 10, 90, 0]
    monkeypatch.setitem(sys.modules, "DR_common2", SimpleNamespace(posx=list))
    monkeypatch.setattr(MP, "_api", SimpleNamespace(
        get_current_posj=lambda: list(sim.joints), ikin=lambda p, sol: list(goal)))
    monkeypatch.setattr(MP, "read_posx", lambda: (list(sim.tcp), 2))

    def fk(j):
        f = geometry(224)
        f["tcp"] = high[:3]
        return f
    monkeypatch.setattr(K, "fk", fk)

    def movej(j, motion):
        sim.calls.append(("high", list(j)))
        sim.joints = list(j)
        sim.tcp = list(high)
    monkeypatch.setattr(MP, "_movej", movej)
    monkeypatch.setattr(MP, "_movel_base", lambda p, v, a: sim.calls.append(("down", list(p))))
    return sim, pose, back


def test_높은_접근점_도착_후에만_수직_하강한다(transfer):
    sim, pose, back = transfer
    MP._pick_to_peg_approach(pose, "right", None, {})
    assert [c[0] for c in sim.calls] == ["high", "down"]
    assert sim.calls[-1][1] == back
    assert sim.tcp[:2] == back[:2]
    assert sim.tcp[2] == 224


@pytest.mark.parametrize("failure", ["height", "ik", "joint", "late_segment"])
def test_경로_검사_실패시_첫_이동도_하지_않는다(transfer, monkeypatch, failure):
    sim, pose, _ = transfer
    route = TP.Route("transit", [90, 0, 0, 0, 0, None], True)
    if failure == "height":
        sim.tcp[2] = 213
    elif failure == "ik":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [float("nan")] * 6)
    elif failure == "joint":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [0, 0, 0, 0, 140, 0])
    else:
        sim.joints[0] = 60  # +90 경유가 필요한 출발점

        def check(start, goal, floor, margin):
            if start[0] == 90:
                raise MP.MotionError("두 번째 구간 높이 부족")
        monkeypatch.setattr(MP, "_check_peg_transfer_path", check)
    with pytest.raises(MP.MotionError):
        MP._pick_to_peg_approach(pose, "right", route, {})
    assert sim.calls == []


def test_이동명령이_성공해도_높은_접근점_미도달이면_하강하지_않는다(transfer, monkeypatch):
    sim, pose, _ = transfer
    monkeypatch.setattr(MP, "_movej", lambda j, m: setattr(sim, "joints", list(j)))
    with pytest.raises(MP.MotionError, match="미도달"):
        MP._pick_to_peg_approach(pose, "right", None, {})
    assert sim.calls == []


def test_하강_확인을_거부하면_높은_접근점에_남는다(transfer):
    sim, pose, _ = transfer

    def confirm(message):
        if "하강" in message:
            raise RuntimeError("사용자 중단")
    with pytest.raises(RuntimeError, match="사용자 중단"):
        MP._pick_to_peg_approach(pose, "right", None, {}, confirm=confirm)
    assert [c[0] for c in sim.calls] == ["high"]
    assert sim.tcp[2] == 224
