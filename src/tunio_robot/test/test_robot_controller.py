"""
robot_controller 노드 테스트. dry_run 으로 돌려 로봇 없이 검증한다.

실제 Action 서버를 띄우고 goal 을 보내 Result 를 받는다.
두산 API는 부르지 않는다.
"""
import os
import threading
from types import SimpleNamespace
from unittest import mock

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from tunio_interfaces.action import AdjustPeg, PlayString, StrumAll
from tunio_robot import motion_primitives as MP
from tunio_robot.robot_controller import RobotController
from tunio_robot.teach_points import Pose
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


# ---------------------------------------------------------------
# 피크 집기·반환 판단 (_holding_pick, _ensure_pick, _release_pick) — 이슈 #34
#
# 아래 둘은 기존 Harness 로 실제 Action 서버를 띄워 정상 흐름을 본다.
# dry_run 이라 MP 는 안 불리지만, _ensure_pick/_release_pick 의 분기·상태
# 갱신 자체는 그대로 실행된다.
# ---------------------------------------------------------------

def test_발현을_반복해도_피크는_한_번만_집는다(ros, taught):
    h = Harness(taught)
    try:
        assert h.node._holding_pick is False

        handle = h.send(PlayString, "/robot/play_string", PlayString.Goal(string_number=6))
        assert handle.result.success is True
        assert h.node._holding_pick is True

        # 두 번째 발현. 이미 쥐고 있으니 _ensure_pick() 이 아무 일도 안 하고
        # 그대로 True 를 유지해야 한다 — 다시 집으러 가면 안 된다
        handle2 = h.send(PlayString, "/robot/play_string", PlayString.Goal(string_number=5))
        assert handle2.result.success is True
        assert h.node._holding_pick is True
    finally:
        h.shutdown()


def test_페그_조절_전에_피크를_반환하고_발현하면_다시_집는다(ros, taught):
    h = Harness(taught)
    try:
        h.send(PlayString, "/robot/play_string", PlayString.Goal(string_number=6))
        assert h.node._holding_pick is True

        goal = AdjustPeg.Goal(string_number=6, direction=1, angle_deg=30.0)
        handle = h.send(AdjustPeg, "/robot/adjust_peg", goal)
        assert handle.result.success is True
        assert h.node._holding_pick is False    # 조절 전에 반환됨

        handle2 = h.send(PlayString, "/robot/play_string", PlayString.Goal(string_number=6))
        assert handle2.result.success is True
        assert h.node._holding_pick is True     # 반환했으니 다시 집는다
    finally:
        h.shutdown()


# ---------------------------------------------------------------
# _ensure_pick / _release_pick 단위 테스트 — Harness(Action·executor)로는
# 한 Action 실행 중간 상태(예외 직후, _side 갱신 시점)를 못 본다.
# RobotController 인스턴스 없이, 필요한 속성만 가진 대역 객체로
# 메서드를 바인드 없이 직접 부른다 (tunio_audio 의 _resolve_replay
# 테스트와 같은 방식 — self 에서 쓰는 것만 갖추면 노드·rclpy 없이 검증된다).
# ---------------------------------------------------------------

def _fake_node(holding, dry_run=False, side="left"):
    holder = Pose("pick.holder", True, task=[1.0, 2.0, 3.0, 0.0, 0.0, 0.0], tcp="GripperDA_v1")
    standby = Pose("frames.standby", True, joints=[0.0] * 6)
    points = SimpleNamespace(
        pick_holder=lambda: holder,
        frame=lambda name: standby,
        motion={},
    )
    return SimpleNamespace(
        _holding_pick=holding,
        _side=side,
        points=points,
        dry_run=dry_run,
        get_logger=lambda: mock.MagicMock(),
    )


def test_이미_쥐고_있으면_다시_집지_않는다(monkeypatch):
    node = _fake_node(holding=True)
    pick_up = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    RobotController._ensure_pick(node)
    pick_up.assert_not_called()
    assert node._holding_pick is True


def test_안_쥐고_있으면_집고_상태를_True로_바꾼다(monkeypatch):
    node = _fake_node(holding=False)
    pick_up = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    RobotController._ensure_pick(node)
    pick_up.assert_called_once()
    assert node._holding_pick is True


def test_집는_중_예외가_나면_모름_상태로_남는다(monkeypatch):
    """
    이슈 #34-1. pick_up() 예외 시 상태가 모름(None)으로 남는지 확인한다.

    그리퍼를 닫은 뒤(즉 물리적으로는 쥔 뒤) 예외가 나면 False 로 남아서는
    안 된다 — 다음 _ensure_pick() 이 pick_up() 을 다시 불러 그 첫 동작
    (_release, 열기)이 지금 팔 위치에서 피크를 놓는다.
    """
    node = _fake_node(holding=False)
    monkeypatch.setattr(MP, "pick_up", mock.MagicMock(side_effect=RuntimeError("접촉 실패")))
    with pytest.raises(RuntimeError):
        RobotController._ensure_pick(node)
    assert node._holding_pick is None


def test_모르는_상태면_집기를_거부한다(monkeypatch):
    node = _fake_node(holding=None)
    pick_up = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    with pytest.raises(MP.MotionError):
        RobotController._ensure_pick(node)
    pick_up.assert_not_called()


def test_모르는_상태면_반환도_거부한다(monkeypatch):
    node = _fake_node(holding=None)
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_return", pick_return)
    with pytest.raises(MP.MotionError):
        RobotController._release_pick(node)
    pick_return.assert_not_called()


def test_안_쥐고_있으면_반환하지_않고_side도_그대로다(monkeypatch):
    node = _fake_node(holding=False, side="left")
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_return", pick_return)
    RobotController._release_pick(node)
    pick_return.assert_not_called()
    assert node._side == "left"


def test_반환하면_side를_비워_경유지를_강제한다(monkeypatch):
    """
    이슈 #34-2. 반환 직후엔 무조건 경유지를 타도록 _side 를 비운다.

    거치대 위에서 페그로 바로 가는 경로는 충돌 검증이 없기 때문이다.
    """
    node = _fake_node(holding=True, side="left")
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_return", pick_return)
    RobotController._release_pick(node)
    pick_return.assert_called_once()
    assert node._holding_pick is False
    assert node._side is None


def test_dry_run이면_MP를_안_부르고도_상태는_바뀐다(monkeypatch):
    node = _fake_node(holding=False, dry_run=True)
    pick_up = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    RobotController._ensure_pick(node)
    pick_up.assert_not_called()
    assert node._holding_pick is True

    node2 = _fake_node(holding=True, dry_run=True, side="left")
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_return", pick_return)
    RobotController._release_pick(node2)
    pick_return.assert_not_called()
    assert node2._holding_pick is False
    assert node2._side is None


def test_strum_all은_미구현_상태라_ensure_pick을_안_부른다(ros, taught):
    """
    이슈 #34-3. strum_all() 미구현 상태라 _ensure_pick() 을 안 부르는지 확인한다.

    dry_run 이 아닌 실기에서 그 앞에 _ensure_pick() 이 있으면 예외로 죽기
    *전에* 진짜로 피크를 집어버려, 로봇이 피크를 쥔 채 반환도 못 하고
    멈춘다. dry_run 경로에서 _holding_pick 이 그대로인지로 확인한다.
    """
    h = Harness(taught)
    try:
        assert h.node._holding_pick is False
        handle = h.send(StrumAll, "/robot/strum_all", StrumAll.Goal(string_numbers=[]))
        assert handle.result.success is True
        assert h.node._holding_pick is False    # _ensure_pick() 이 안 불렸다
    finally:
        h.shutdown()
