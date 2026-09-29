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
    high = list(back[:3]) + [0, 180, 0]
    high[2] = 224
    sim = SimpleNamespace(joints=[0] * 6, tcp=[400, 0, 224, 0, 180, 0], calls=[])
    solutions = {}

    def ikin(task, sol):
        key = tuple(task)
        if key not in solutions:
            solutions[key] = [20, len(solutions) + 1, 10, 10, 90, 0]
        return list(solutions[key])
    monkeypatch.setitem(sys.modules, "DR_common2", SimpleNamespace(posx=list))
    monkeypatch.setattr(MP, "_api", SimpleNamespace(
        get_current_posj=lambda: list(sim.joints), ikin=ikin))
    monkeypatch.setattr(MP, "read_posx", lambda: (list(sim.tcp), 2))

    def fk(j):
        f = geometry(224)
        f["tcp"] = next((list(task[:3]) for task, joints in solutions.items()
                         if joints == list(j)), high[:3])
        return f
    monkeypatch.setattr(K, "fk", fk)

    def movej(j, motion):
        task = next((list(task) for task, joints in solutions.items() if joints == list(j)), high)
        sim.calls.append(("joint_orient" if task == back else "high", list(j)))
        sim.joints = list(j)
        sim.tcp = list(task)
    monkeypatch.setattr(MP, "_movej", movej)

    def movel(p, v, a):
        sim.calls.append(("down" if p[3:] == sim.tcp[3:] else "orient", list(p)))
        sim.tcp = list(p)
        sim.last_speed = (v, a)
    monkeypatch.setattr(MP, "_movel_base", movel)
    return sim, pose, back


def test_높은_접근점_도착_후에만_수직_하강한다(transfer):
    sim, pose, back = transfer
    MP._pick_to_peg_approach(pose, "right", None, {})
    assert [c[0] for c in sim.calls] == ["high", "down", "joint_orient"]
    assert sim.calls[1][1] == back[:3] + [0, 180, 0]
    assert sim.tcp == back
    assert sim.tcp[:2] == back[:2]
    assert sim.tcp[2] == 108


@pytest.mark.parametrize("failure", ["height", "ik", "joint", "j1"])
def test_경로_검사_실패시_첫_이동도_하지_않는다(transfer, monkeypatch, failure):
    sim, pose, _ = transfer
    route = TP.Route("transit", [90, 0, 0, 0, 0, None], True)
    if failure == "height":
        sim.tcp[2] = 213
    elif failure == "ik":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [float("nan")] * 6)
    elif failure == "joint":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [0, 0, 0, 0, 0, 361])
    else:
        sim.joints[0] = 85  # +90 경유가 필요한 출발점

        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [250, 0, 0, 0, 0, 0])
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


def test_실기진단은_검사를_통과해도_이동하지_않는다(transfer):
    sim, pose, _ = transfer
    plan = MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert plan["high_tcp"][2] == 224
    assert sim.calls == []


def test_목표_관절_초과_메시지에_실제값과_허용범위가_있다(transfer, monkeypatch):
    sim, pose, _ = transfer
    monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [0, 0, 0, 0, 0, 361])
    with pytest.raises(MP.MotionError, match=r"J6=\+361.00.*±360"):
        MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert sim.calls == []


def test_반환방향_높은_목표가_불가능하면_이동하지_않는다(transfer, monkeypatch):
    sim, pose, _ = transfer

    def ikin(task, sol):
        assert task[3:] == sim.tcp[3:]
        raise RuntimeError("IK 해 없음")
    monkeypatch.setattr(MP._api, "ikin", ikin)
    with pytest.raises(MP.MotionError, match="IK 실패"):
        MP._pick_to_peg_approach(pose, "right", None, {})
    assert sim.calls == []


def test_반환후_접근도_공통_이동속도배율을_사용한다(transfer, monkeypatch):
    sim, pose, _ = transfer
    original = MP._movej
    seen = []

    def movej(joints, motion):
        seen.append(motion["speed_scale"])
        return original(joints, motion)
    monkeypatch.setattr(MP, "_movej", movej)
    motion = {"speed_scale": 2.0}
    MP._pick_to_peg_approach(pose, "right", None, motion)
    assert seen == [2.0, 1.0]
    assert motion["speed_scale"] == 2.0


@pytest.mark.parametrize("start,end", [(-61, -59), (239, 241), (-60, -120), (300, 350)])
def test_헤드쪽_짧은_J1_구간은_90도를_안지나도_거부한다(start, end):
    with pytest.raises(MP.MotionError, match="J1은 -60~240"):
        MP._check_peg_transfer_path([start, 0, 0, 0, 0, 0], [end, 0, 0, 0, 0, 0], 214, 5)


def test_J1_긴_방향은_끝점_포함_허용한다():
    MP._check_peg_transfer_path([-60, 0, 0, 0, 0, 0], [240, 0, 0, 0, 0, 0], 214, 5)


def test_이동경로_높이로_차단하지_않는다(transfer, monkeypatch):
    sim, pose, _ = transfer

    def reject(*args):
        raise AssertionError("운영 접근에서 경로 높이 검사 금지")
    monkeypatch.setattr(MP, "_check_peg_transfer_path", reject)
    MP._pick_to_peg_approach(pose, "right", None, {})
    assert [kind for kind, _ in sim.calls] == ["high", "down", "joint_orient"]
