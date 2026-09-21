"""
robot_controller 노드 테스트. dry_run 으로 돌려 로봇 없이 검증한다.

실제 Action 서버를 띄우고 goal 을 보내 Result 를 받는다.
두산 API는 부르지 않는다.
"""
import os
import threading

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from tunio_interfaces.action import AdjustPeg, PlayString
from tunio_robot.robot_controller import RobotController
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "teach_points.yaml")


def _taught_yaml(tmpdir):
    """교시를 마친 상태의 임시 교시점 파일을 만든다."""
    with open(CONFIG, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    for section in ("frames", "pegs", "pluck"):
        for key in d[section]:
            d[section][key]["taught"] = True
    path = os.path.join(tmpdir, "taught.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(d, f)
    return path


class Harness:
    """노드를 executor 에서 돌리고 goal 을 보내는 도우미."""

    def __init__(self, yaml_path):
        self.node = RobotController()
        self.node.set_parameters([
            rclpy.parameter.Parameter("teach_points", value=yaml_path),
        ])
        # 파라미터는 생성자에서 이미 읽었으므로 교시점을 직접 갈아끼운다
        from tunio_robot import teach_points as tp
        self.node.points = tp.load(yaml_path)

        self.client_node = rclpy.create_node("test_client")
        self.ex = MultiThreadedExecutor()
        self.ex.add_node(self.node)
        self.ex.add_node(self.client_node)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def send(self, action_type, name, goal):
        cli = ActionClient(self.client_node, action_type, name)
        assert cli.wait_for_server(timeout_sec=5.0), f"{name} 서버가 안 뜬다"
        handle = cli.send_goal(goal)
        return handle

    def shutdown(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.client_node.destroy_node()


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def taught(tmp_path):
    return _taught_yaml(str(tmp_path))


def test_교시_끝난_상태에서_페그_조절_goal_이_성공한다(ros, taught):
    h = Harness(taught)
    try:
        goal = AdjustPeg.Goal(string_number=6, direction=1, angle_deg=30.0)
        handle = h.send(AdjustPeg, "/robot/adjust_peg", goal)
        assert handle.result.success is True
        assert handle.result.message == ""
    finally:
        h.shutdown()


def _untaught_yaml(tmpdir):
    """모든 페그를 미교시로 되돌린 임시 파일."""
    with open(CONFIG, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    for key in d["pegs"]:
        d["pegs"][key]["taught"] = False
    path = os.path.join(tmpdir, "untaught.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(d, f)
    return path


def test_미교시_상태면_실패하고_사유가_담긴다(ros, tmp_path):
    """교시가 한 벌로 안 맞으면 taught 를 내린다. 그때 움직이지 않아야 한다."""
    h = Harness(_untaught_yaml(str(tmp_path)))
    try:
        goal = AdjustPeg.Goal(string_number=6, direction=1, angle_deg=30.0)
        handle = h.send(AdjustPeg, "/robot/adjust_peg", goal)
        assert handle.result.success is False
        assert "교시" in handle.result.message
    finally:
        h.shutdown()


def test_상한을_넘는_각도는_실패로_돌아온다(ros, taught):
    """서버 계산이 틀려도 줄이 끊어지지 않게 로봇에서 막는다."""
    h = Harness(taught)
    try:
        goal = AdjustPeg.Goal(string_number=6, direction=1, angle_deg=999.0)
        handle = h.send(AdjustPeg, "/robot/adjust_peg", goal)
        assert handle.result.success is False
        assert "상한" in handle.result.message
    finally:
        h.shutdown()


def test_대상_아닌_줄은_실패로_돌아온다(ros, taught):
    h = Harness(taught)
    try:
        goal = AdjustPeg.Goal(string_number=3, direction=1, angle_deg=10.0)
        handle = h.send(AdjustPeg, "/robot/adjust_peg", goal)
        assert handle.result.success is False
        assert "대상 줄" in handle.result.message
    finally:
        h.shutdown()


def test_발현_goal_도_같은_경로로_돈다(ros, taught):
    h = Harness(taught)
    try:
        handle = h.send(PlayString, "/robot/play_string",
                        PlayString.Goal(string_number=1))
        assert handle.result.success is True
    finally:
        h.shutdown()


# ─────────── Phase 0 를 노드가 스스로 돈다 (이슈 #31) ───────────
#
# 서버는 "작업 시작" 을 로봇에 알리지 않는다. 로봇이 아는 건 Action 세 개뿐이라,
# 피크 집기·반환처럼 **필요해지는 순간** 등록한다. 인터페이스가 안 바뀐다.
#
# 등록이 없으면 교시 자세로 들어가 물리는 순간 페그가 되돌려진다. 파지에 성공하고
# 성공으로 보고하면서 조용히 틀리는 게 #31 이다. 그래서 모르면 돌리지 않는다.

def _node(yaml_path):
    """노드만 만든다 (executor 없이). 등록 로직만 볼 때 쓴다."""
    node = RobotController()
    from tunio_robot import teach_points as tp
    node.points = tp.load(yaml_path)
    return node


def test_미등록이면_등록부터_한다(ros, taught, monkeypatch, tmp_path):
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        node.pegs.path = str(tmp_path / "state.json")
        node.pegs.forget()
        called = []
        monkeypatch.setattr(MP, "register_peg",
                            lambda *a, **k: called.append(k) or 37.0)
        assert node._ensure_registered(6) == pytest.approx(37.0)
        assert called, "register_peg 을 안 불렀다"
        assert node.pegs.roll_of(6) == pytest.approx(37.0)   # 기억한다
    finally:
        node.destroy_node()


def test_이미_등록돼_있으면_다시_안_한다(ros, taught, monkeypatch, tmp_path):
    """등록은 싸지 않다. 매번 돌면 사이클 타임에 그대로 얹힌다."""
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        node.pegs.path = str(tmp_path / "state.json")
        node.pegs.register(6, 12.0)
        called = []
        monkeypatch.setattr(MP, "register_peg", lambda *a, **k: called.append(1) or 0.0)
        assert node._ensure_registered(6) == pytest.approx(12.0)
        assert called == [], "이미 아는데 또 등록했다"
    finally:
        node.destroy_node()


def test_등록에_실패하면_그대로_올라간다(ros, taught, monkeypatch, tmp_path):
    """
    손잡이가 서 있으면 사람이 눕혀야 한다. 조용히 넘어가면 페그를 되돌린다.

    예외가 올라가면 _run() 이 잡아 goal 을 실패로 만들고, 메시지에 어느 줄인지가 남는다.
    """
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        node.pegs.path = str(tmp_path / "state.json")
        node.pegs.forget()

        def boom(*a, **k):
            raise MP.MotionError("pegs.6: 손잡이가 서 있어 못 물었다")
        monkeypatch.setattr(MP, "register_peg", boom)
        with pytest.raises(MP.MotionError, match="pegs.6"):
            node._ensure_registered(6)
        assert node.pegs.roll_of(6) is None      # 실패는 기억하지 않는다
    finally:
        node.destroy_node()


def test_폭_읽기는_한_번만_만든다(ros, taught):
    node = _node(taught)
    try:
        assert node._width_reader() is node._width_reader()
    finally:
        node.destroy_node()


# ─────────── 거치대에서 페그로 바로 가지 않는다 (이슈 #34) ───────────
#
# 피크를 반환하면 팔이 거치대 위에 남는다. _cross_to 는 "직전에 있던 쪽" 으로
# 판단하는데 거치대는 좌우 어느 쪽도 아니다. 그 기억을 안 지우면 직전 페그와 같은
# 쪽일 때 경유지를 건너뛰고 거치대에서 페그 접근점으로 관절 보간으로 바로 간다.
# 그 경로는 아무도 본 적이 없다 — 충돌 검증(2026-09-19)은 페그↔페그와 경유지뿐이다.

def test_피크를_반환하면_어디_있는지_모르는_상태가_된다(ros, taught):
    node = _node(taught)
    try:
        node._side = "left"
        node._holding_pick = True
        node.dry_run = True
        node._release_pick()
        assert node._side is None
    finally:
        node.destroy_node()


def test_쪽을_모르면_무조건_경유지를_탄다(ros, taught, monkeypatch):
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        went = []
        monkeypatch.setattr(MP, "goto_route", lambda r, m: went.append(r.name))
        node._side = None
        node._cross_to("left")
        assert went, "경유지를 안 탔다"
    finally:
        node.destroy_node()


def test_같은_쪽이면_경유지를_건너뛴다(ros, taught, monkeypatch):
    """같은 쪽 페그끼리는 37mm 라 매번 경유하면 사이클 타임만 는다."""
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        went = []
        monkeypatch.setattr(MP, "goto_route", lambda r, m: went.append(r.name))
        node._side = "left"
        node._cross_to("left")
        assert went == []
    finally:
        node.destroy_node()


def test_피크를_안_쥐고_있으면_쪽_기억을_안_지운다(ros, taught):
    """반환할 게 없으면 팔은 그 자리다. 괜히 경유지를 타면 사이클 타임만 는다."""
    node = _node(taught)
    try:
        node._side = "left"
        node._holding_pick = False
        node._release_pick()
        assert node._side == "left"
    finally:
        node.destroy_node()
