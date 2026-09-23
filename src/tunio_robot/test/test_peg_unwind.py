"""#70: 실제 관절각이 누적되는 대역으로 양방향 되감기와 실패 경로를 검증한다."""
from types import SimpleNamespace

import pytest
from tunio_robot import motion_primitives as MP
from tunio_robot import peg_state as PS
from tunio_robot.teach_points import Pose


@pytest.fixture
def rig(monkeypatch):
    pose = Pose("pegs.1", True, task=[300, 60, 108, 90, -90, 0], tcp="GripperDA_v1")
    sim = SimpleNamespace(joints=[53, 52, 85, -45, 122.3, -320.0], events=[],
                          at="away", grip="open", peg_angle=0.0, ignore_unwind=False,
                          entry_jump=0.0, fail_turn=False, fail_retreat=False)
    motion = {"peg_unwind_enabled": True, "approach_mm": 100.0}
    monkeypatch.setattr(MP, "_api",
                        SimpleNamespace(get_current_posj=lambda: list(sim.joints)))
    monkeypatch.setattr(MP, "ensure_tcp", lambda: None)

    def movejx(task, motion, sol=None):
        sim.at = "approach"
        sim.events.append("approach")

    def movel(task, motion):
        if task[:3] == pose.task[:3]:
            sim.at = "peg"
            sim.joints[5] += sim.entry_jump
        else:
            if sim.fail_retreat:
                raise MP.MotionError("후퇴 실패")
            sim.at = "approach"
        sim.events.append(sim.at)

    def grip(name):
        sim.grip = name
        sim.events.append(name)

    def movej(target, motion, relative=False):
        if relative:
            assert sim.at == "peg" and sim.grip == "peg"
            delta = target[5] / 2 if sim.fail_turn else target[5]
            sim.joints[5] += delta
            sim.peg_angle += delta
            sim.events.append("turn")
            if sim.fail_turn:
                raise MP.MotionError("부분 회전 실패")
        else:
            assert sim.at == "approach" and sim.grip == "open"
            assert target[:5] == sim.joints[:5]
            assert abs(target[5] - sim.joints[5]) == (
                180.0 if motion.get("j6_recovery_enabled") else 360.0)
            sim.events.append("unwind")
            if not sim.ignore_unwind:
                sim.joints[:] = target
        assert abs(sim.joints[5]) <= 355.0

    monkeypatch.setattr(MP, "_movejx", movejx)
    monkeypatch.setattr(MP, "_movel", movel)
    monkeypatch.setattr(MP, "_movej", movej)
    monkeypatch.setattr(MP, "_grip", grip)
    monkeypatch.setattr(MP, "_release", lambda: grip("open"))
    monkeypatch.setattr(MP, "read_posx", lambda: (list(pose.task), 2))
    return sim, pose, motion


@pytest.mark.parametrize("direction", [-1, 1])
@pytest.mark.parametrize("angle", [60.0, 90.0])
def test_양방향_12회_반복해도_한계_안에서_요청량만_페그를_돌린다(rig, tmp_path, direction, angle):
    sim, pose, motion = rig
    sim.joints[5] = direction * 320.0
    st = PS.PegState(str(tmp_path / "state.json"))
    st.register(1, 0.0)
    for _ in range(12):
        roll = st.roll_of(1)
        with st.turning(1, direction * angle):
            MP.adjust_peg(pose, 6, direction * angle, motion, roll_deg=roll)
        assert abs(sim.joints[5]) <= 355.0
    assert sim.peg_angle == direction * angle * 12  # 빈손 되감기는 누적에서 빠진다
    assert st.roll_of(1) == pytest.approx(0.0)
    assert sim.events.count("unwind") >= 2
    assert sim.events.index("unwind") < sim.events.index("peg")


def test_여유가_있으면_되감지_않는다(rig):
    sim, pose, motion = rig
    sim.joints[5] = 0.0
    MP.adjust_peg(pose, 6, -60, motion)
    assert "unwind" not in sim.events
    assert sim.peg_angle == -60


@pytest.mark.parametrize("failure", ["disabled", "read", "nan", "nan_j5", "j6", "unreached"])
def test_불확실하거나_한계_밖이면_물거나_돌리지_않는다(rig, monkeypatch, failure):
    sim, pose, motion = rig
    if failure == "disabled":
        motion["peg_unwind_enabled"] = False
    elif failure == "read":
        monkeypatch.setattr(MP._api, "get_current_posj", lambda: -1)
    elif failure == "nan":
        sim.joints[5] = float("nan")
    elif failure == "nan_j5":
        sim.joints[4] = float("nan")
    elif failure == "j6":
        sim.joints[5] = -363.3
    else:
        sim.ignore_unwind = True
    with pytest.raises(MP.MotionError):
        MP.adjust_peg(pose, 6, -60, motion)
    assert "peg" not in sim.events
    assert sim.peg_angle == 0.0


def test_진입으로_J6가_달라지면_페그_앞에서_되감지_않는다(rig):
    sim, pose, motion = rig
    sim.joints[5] = -250.0
    sim.entry_jump = -70.0
    with pytest.raises(MP.MotionError, match="J6 회전 여유 부족"):
        MP.adjust_peg(pose, 6, -60, motion)
    assert sim.grip == "open"
    assert "unwind" not in sim.events and "turn" not in sim.events


def test_회전실패후_현재자세도_못읽으면_추정_후퇴하지_않는다(rig, monkeypatch):
    sim, pose, motion = rig
    sim.joints[5] = 0.0
    sim.fail_turn = True
    monkeypatch.setattr(MP, "read_posx", lambda: (-1, 0))
    with pytest.raises(MP.MotionError, match="부분 회전 실패.*후퇴도 실패"):
        MP.adjust_peg(pose, 6, -60, motion)
    assert sim.events.count("approach") == 1
    assert sim.at == "peg" and sim.grip == "open"


def test_회전후_그리퍼_열기_실패시_후퇴하지_않는다(rig, monkeypatch):
    sim, pose, motion = rig
    sim.joints[5] = 0.0

    def fail_release():
        raise MP.MotionError("열기 실패")

    monkeypatch.setattr(MP, "_release", fail_release)
    with pytest.raises(MP.MotionError, match="열기 실패"):
        MP.adjust_peg(pose, 6, -60, motion)
    assert sim.at == "peg" and sim.grip == "peg"


@pytest.mark.parametrize("key,value", [
    ("peg_joint_margin_deg", float("nan")),
    ("peg_j6_limit_deg", 400),
    ("peg_unwind_tolerance_deg", -1),
])
def test_유효하지_않은_설정에서는_회전하지_않는다(rig, key, value):
    sim, pose, motion = rig
    motion[key] = value
    with pytest.raises(MP.MotionError):
        MP.adjust_peg(pose, 6, -60, motion)
    assert "unwind" not in sim.events and "turn" not in sim.events


@pytest.mark.parametrize("failure", ["turn", "retreat"])
def test_회전이나_후퇴_실패시_재시작해도_옛_각을_믿지_않는다(rig, tmp_path, failure):
    sim, pose, motion = rig
    sim.joints[5] = 0.0
    sim.fail_turn = failure == "turn"
    sim.fail_retreat = failure == "retreat"
    path = str(tmp_path / "state.json")
    st = PS.PegState(path)
    st.register(1, 20.0)
    st.save()
    with pytest.raises(MP.MotionError):
        with st.turning(1, -60):
            MP.adjust_peg(pose, 6, -60, motion, roll_deg=20.0)
    assert st.roll_of(1) is None
    assert PS.PegState(path).load().roll_of(1) is None


@pytest.mark.parametrize("fail_at", [1, 2])
def test_저장_실패는_동작전_차단하거나_미등록으로_남긴다(tmp_path, monkeypatch, fail_at):
    path = str(tmp_path / "state.json")
    st = PS.PegState(path)
    st.register(1, 10.0)
    st.save()
    save = st.save
    calls, moved = [], []

    def fail_save():
        calls.append(1)
        if len(calls) == fail_at:
            raise OSError("디스크 오류")
        return save()

    monkeypatch.setattr(st, "save", fail_save)
    with pytest.raises(OSError):
        with st.turning(1, 60):
            moved.append(True)
    assert bool(moved) == (fail_at == 2)
    assert st.roll_of(1) is None
    assert PS.PegState(path).load().roll_of(1) == (10.0 if fail_at == 1 else None)


def test_180도_회복을_진입후퇴에_반영하고_페그회전량에는_넣지_않는다(rig, monkeypatch):
    sim, pose, motion = rig
    motion.update(j6_recovery_enabled=True, peg_j6_limit_deg=300.0)
    sim.joints[5] = 285.0
    monkeypatch.setattr(MP, "_j6_pose", lambda pose, motion, delta_deg=0: (pose, 0.0))
    tasks = []
    original = MP._movel

    def movel(task, motion):
        tasks.append(list(task))
        original(task, motion)
    monkeypatch.setattr(MP, "_movel", movel)
    MP.adjust_peg(pose, 6, 60, motion)
    assert sim.peg_angle == 60
    assert sim.joints[5] == 165
    assert sim.events.count("unwind") == 1
    assert tasks[0][5] == pytest.approx(-180)
    assert tasks[-1][5] == pytest.approx(-120)
