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
    high[2] = 299
    sim = SimpleNamespace(joints=[0] * 6, tcp=[400, 0, 224, 0, 180, 0], calls=[],
                          j5=lambda task: 90)  # 가짜 IK 의 J5. 테스트가 규칙을 바꾼다
    solutions = {}

    def ikin(task, sol):
        key = tuple(task)
        if key not in solutions:
            solutions[key] = [20, len(solutions) + 1, 10, 10, sim.j5(task), 0]
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
        sim.calls.append(("high", list(j)))
        sim.joints = list(j)
        sim.tcp = list(next(task for task, q in solutions.items() if q == list(j)))
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
    assert [c[0] for c in sim.calls] == ["high", "high", "down"]
    assert sim.calls[-1][1] == back
    assert sim.tcp[:2] == back[:2]
    assert sim.tcp[2] == 108


@pytest.mark.parametrize("failure", ["current", "ik", "joint", "late_segment"])
def test_경로_검사_실패시_첫_이동도_하지_않는다(transfer, monkeypatch, failure):
    sim, pose, _ = transfer
    route = TP.Route("transit", [90, 0, 0, 0, 0, None], True)
    if failure == "current":
        sim.tcp[2] = float("nan")
    elif failure == "ik":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [float("nan")] * 6)
    elif failure == "joint":
        monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [0, 0, 0, 0, 0, 361])
    else:
        sim.joints[0] = 85  # +90 경유가 필요한 출발점

        def check(start, goal):
            if start[0] == 90:
                raise MP.MotionError("두 번째 구간 관절 검사 실패")
        monkeypatch.setattr(MP, "_check_transfer_joints", check)
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
    assert [c[0] for c in sim.calls] == ["high", "high"]
    assert sim.tcp[2] == 299


def test_실기진단은_검사를_통과해도_이동하지_않는다(transfer):
    sim, pose, _ = transfer
    plan = MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert plan["high_tcp"][2] == 299
    assert sim.calls == []


def test_목표_관절_초과_메시지에_실제값과_허용범위가_있다(transfer, monkeypatch):
    sim, pose, _ = transfer
    monkeypatch.setattr(MP._api, "ikin", lambda p, sol: [0, 0, 0, 0, 0, 361])
    with pytest.raises(MP.MotionError, match=r"J6=\+361.00.*±360"):
        MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert sim.calls == []


def test_높은_파지방향_목표가_불가능하면_이동하지_않는다(transfer, monkeypatch):
    sim, pose, _ = transfer

    def ikin(task, sol):
        assert task[3:] == pose.task[3:]
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
    assert seen == [2.0, 2.0]
    assert motion["speed_scale"] == 2.0


@pytest.mark.parametrize("start,end", [(-61, -59), (239, 241), (-60, -120), (300, 350)])
def test_헤드쪽_짧은_J1_구간은_90도를_안지나도_거부한다(start, end):
    with pytest.raises(MP.MotionError, match="J1은 -60~240"):
        MP._check_peg_transfer_path([start, 0, 0, 0, 0, 0], [end, 0, 0, 0, 0, 0], 214, 5)


def test_J1_긴_방향은_끝점_포함_허용한다():
    MP._check_peg_transfer_path([-60, 0, 0, 0, 0, 0], [240, 0, 0, 0, 0, 0], 214, 5)


def test_반환페그의_기준높이는_관절점의_강제하한이_아니다(transfer, monkeypatch):
    sim, pose, _ = transfer
    original = K.fk

    def low_joint(j):
        f = original(j)
        f["origins"][3] = [0, 0, 180]
        return f
    monkeypatch.setattr(K, "fk", low_joint)
    MP._pick_to_peg_approach(pose, "right", None, {})
    assert [c[0] for c in sim.calls] == ["high", "high", "down"]


def test_실기_상단전용은_도달확인후_하강없이_종료한다(transfer):
    sim, pose, _ = transfer
    MP._pick_to_peg_approach(pose, "right", None, {}, high_only=True)
    assert [c[0] for c in sim.calls] == ["high", "high"]
    assert sim.tcp[2] == 299


# ---- 오른쪽 파지 방향은 상단이 아니라 J5 가 허용하는 높이에서 만든다 (2026-09-29)
def _peg_orientation_too_high(pose, above_z, j5=138.0):
    """페그 파지 방향(손목 수평)은 above_z 보다 높으면 J5 가 한계를 넘는 가짜 IK 규칙."""
    return lambda task: j5 if (task[3:] == pose.task[3:] and task[2] > above_z) else 90


def test_상단에서_J5가_넘으면_손목_아래로_내려온_뒤_회전한다(transfer):
    sim, pose, back = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=200)
    MP._pick_to_peg_approach(pose, "right", None, {})
    # 상단(손목 아래) → 회전 높이까지 수직 하강 → 파지 방향 회전 → 접근점 하강
    assert [c[0] for c in sim.calls] == ["high", "down", "high", "down"]
    turn = sim.calls[1][1]
    assert turn[:2] == back[:2] and turn[3:] == [0, 180, 0]   # 손목은 그대로
    assert 195 <= turn[2] <= 200                                # 5mm 간격으로 찾은 첫 높이
    assert sim.calls[-1][1] == back


def test_회전_높이는_IK가_정하고_계획조회에_드러난다(transfer):
    sim, pose, back = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=200)
    plan = MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert plan["pivot_tcp"][2] == 299 and plan["pivot_tcp"][3:] == [0, 180, 0]
    assert plan["high_tcp"][2] == plan["turn_low_tcp"][2] < 200
    assert plan["high_tcp"][3:] == pose.task[3:]
    assert all(abs(q[4]) <= 130 for q in plan["joint_plan"])
    assert sim.calls == []


def test_J5_여유는_한계에서_뺀_값으로_판정한다(transfer):
    sim, pose, _ = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=200, j5=132.0)  # 135 안이지만 여유 5 밖
    plan = MP._pick_to_peg_approach(pose, "right", None, {}, execute=False)
    assert plan["high_tcp"][2] < 200
    plan = MP._pick_to_peg_approach(pose, "right", None, {"j5_turn_margin_deg": 1.0},
                                    execute=False)
    assert plan["high_tcp"][2] == 299 and plan["turn_low_tcp"] is None


def test_접근점까지_내려와도_J5가_안되면_이동하지_않는다(transfer):
    sim, pose, _ = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=0)
    with pytest.raises(MP.MotionError, match="돌릴 높이가 없다"):
        MP._pick_to_peg_approach(pose, "right", None, {})
    assert sim.calls == []


def test_회전_높이_미도달이면_자세를_바꾸지_않는다(transfer, monkeypatch):
    sim, pose, _ = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=200)
    original = MP._movel_base

    def short(p, v, a):
        original(p, v, a)
        sim.tcp[2] += 10  # 덜 내려왔다
    monkeypatch.setattr(MP, "_movel_base", short)
    with pytest.raises(MP.MotionError, match="회전 높이 미도달"):
        MP._pick_to_peg_approach(pose, "right", None, {})
    assert [c[0] for c in sim.calls] == ["high", "down"]


def test_경유연결을_켜도_회전높이_수직하강_전후에는_멈춘다(transfer, monkeypatch):
    sim, pose, back = transfer
    sim.j5 = _peg_orientation_too_high(pose, above_z=200)
    monkeypatch.setattr(MP, "_cancel_check", lambda: None)
    original = MP._movej_path
    groups = []

    def path(targets, motion, radius, logger, confirm):
        groups.append((len(targets), radius))
        return original(targets, motion, radius, logger, confirm)
    monkeypatch.setattr(MP, "_movej_path", path)
    MP._pick_to_peg_approach(pose, "right", None, {"transit_blend_radius_mm": 10})
    # 각 관절 구간은 마지막 반경 0으로 끝난다. 하강을 넘어 이어 붙이지 않는다.
    assert groups == [(1, 10), (1, 10)]
    assert [c[0] for c in sim.calls] == ["high", "down", "high", "down"]
    assert sim.calls[-1][1] == back
