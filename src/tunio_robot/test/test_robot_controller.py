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
from tunio_robot.teach_points import Route as TPRoute
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

    def send(self, action_type, name, goal, feedback_callback=None):
        cli = ActionClient(self.client_node, action_type, name)
        assert cli.wait_for_server(timeout_sec=5.0), f"{name} 서버가 안 뜬다"
        handle = cli.send_goal(goal, feedback_callback=feedback_callback)
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

def _fake_node(holding, dry_run=False, side="left", verify="skip"):
    """
    필요한 속성만 가진 대역 노드를 만든다.

    verify — _verify_pick 이 낼 값. 문자열이면 늘 그 값, 리스트면 회차별로 낸다.
             기본 "skip" 은 확인을 안 하는 예전 동작이다 (#39).
    """
    holder = Pose("pick.holder", True, task=[1.0, 2.0, 3.0, 0.0, 0.0, 0.0], tcp="GripperDA_v1")
    standby = Pose("frames.standby", True, joints=[0.0] * 6)
    # 거치대 팔 모양을 바꿀 때 지나는 J1=90 경유지 (#58). 실물과 같은 값이다
    route = TPRoute("routes.transit", [90.0, 0.0, 0.0, 0.0, 0.0, None], True)
    points = SimpleNamespace(
        pick_holder=lambda: holder,
        frame=lambda name: standby,
        route=lambda: route,
        motion={},
    )
    seq = list(verify) if isinstance(verify, (list, tuple)) else None

    def _verify():
        if seq is None:
            return verify
        return seq.pop(0) if seq else "empty"

    node = SimpleNamespace(
        _holding_pick=holding,
        _pick_fault=None,
        _side=side,
        points=points,
        dry_run=dry_run,
        get_logger=lambda: mock.MagicMock(),
        _verify_pick=_verify,
    )
    for name in ('_notify_pick', '_require_pick', '_try_pick', '_recover_pick',
                 '_check_pick_or_recover'):
        setattr(node, name, getattr(RobotController, name).__get__(node))
    return node


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


def test_strum_all_실기에서도_미리_쥔_피크는_반환한다(ros, taught, monkeypatch):
    """
    PR #38 리뷰 ③. strum_all() 이 예외를 던지기 전에 _release_pick() 을 부른다.

    play_string 이 앞서 쥐어 놓은 피크는 strum_all() 이 NotImplementedError 로
    죽기 전에 반환돼야 한다. 반환을 strum_all() 뒤에 두면 예외로 도달하지
    않는다 — 실기 경로(dry_run 아님)로 확인한다.
    """
    h = Harness(taught)
    try:
        h.node.dry_run = False
        h.node._holding_pick = True             # play_string 이 앞서 쥐어 놓은 상태
        pick_return = mock.MagicMock()
        monkeypatch.setattr(MP, "pick_return", pick_return)

        handle = h.send(StrumAll, "/robot/strum_all", StrumAll.Goal(string_numbers=[]))

        assert handle.result.success is False    # strum_all() 미구현이라 여전히 실패
        pick_return.assert_called_once()          # 그래도 반환은 이미 됐다
        assert h.node._holding_pick is False
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


@pytest.mark.parametrize("fails", [False, True])
def test_조절_Action은_동작전_등록을_지우고_성공시에만_요청각을_저장한다(
        ros, taught, monkeypatch, tmp_path, fails):
    from tunio_robot.peg_state import PegState
    h = Harness(taught)
    try:
        node = h.node
        node.dry_run = False
        node._side = "left"
        monkeypatch.setattr(MP, "_current_j1", lambda: 180.0)
        node.pegs = PegState(str(tmp_path / "state.json"))
        node.pegs.register(6, 20.0)
        node.pegs.save()

        def adjust(*args, **kwargs):
            assert kwargs["roll_deg"] == 20.0
            assert PegState(node.pegs.path).load().roll_of(6) is None
            if fails:
                raise MP.MotionError("후퇴 실패")

        monkeypatch.setattr(MP, "adjust_peg", adjust)
        result = h.send(AdjustPeg, "/robot/adjust_peg",
                        AdjustPeg.Goal(string_number=6, direction=1, angle_deg=30.0))
        assert result.result.success is not fails
        expected = None if fails else -10.0
        assert node.pegs.roll_of(6) == expected
        assert PegState(node.pegs.path).load().roll_of(6) == expected
    finally:
        h.shutdown()


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


# 반환 후 쪽 기록은 지우되, 페그 접근은 현재 J1으로 판단한다.

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
        monkeypatch.setattr(MP, "_current_j1", lambda: None)
        node._cross_to("left")
        assert went, "경유지를 안 탔다"
    finally:
        node.destroy_node()


def test_현재_J1이_해당_범위면_경유지를_건너뛴다(ros, taught, monkeypatch):
    """직전 쪽 기록 대신 실제 J1을 사용한다."""
    from tunio_robot import motion_primitives as MP
    node = _node(taught)
    try:
        went = []
        monkeypatch.setattr(MP, "goto_route", lambda r, m: went.append(r.name))
        node._side = None
        monkeypatch.setattr(MP, "_current_j1", lambda: 180.0)
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


# ---------------------------------------------------------- 힘 제어 (#51)
def test_힘_제어는_기본으로_꺼져_있다(ros):
    """
    통합 뒤로 미뤘다 (2026-09-21 결정).

    아직 실물 검증이 안 끝났다. 9/21 회차가 4N 으로 20초에 +7.5mm 밖에 못 갔고,
    스프링을 이기고 갈 수 있는 거리도 자세마다 달라진다. 통합 실기에서 여기서
    막히면 그 시간이 통째로 날아간다. 켜려면 런치에서 명시한다.
    """
    node = RobotController()
    try:
        assert node.get_parameter("peg_register_force").value is False
    finally:
        node.destroy_node()


# ------------------------------------------------------------ 복귀 보장 (#58)
def test_화음이_실패해도_복귀는_한다(ros, taught, monkeypatch):
    """
    Strum_all 이 예외를 던져도 return_home 은 돈다.

    전에는 바로 다음 줄인데도 안 돌았다 — 예외가 거기서 함수를 끝냈기 때문이다.
    서버는 화음 실패를 "연출이라 결과에 영향 없다" 며 경고로만 넘기므로 화면에는
    '조율 완료' 가 뜨는데, 실물은 팔이 마지막 페그 앞에 남았다. 아무도 모른다.
    """
    seen = []

    def 터진다(*a, **k):
        raise NotImplementedError("성현 담당 — 훑기 동작 교시 후 구현")

    monkeypatch.setattr(MP, "strum_all", 터진다)
    monkeypatch.setattr(MP, "return_home", lambda *a, **k: seen.append("home"))

    h = Harness(taught)
    try:
        h.node.dry_run = False
        h.node._holding_pick = False
        handle = h.send(StrumAll, "/robot/strum_all", StrumAll.Goal())
        assert seen == ["home"]                     # 복귀했다
        assert handle.result.success is False       # 그러면서 실패는 실패로 보고한다
        assert "NotImplementedError" in handle.result.message
    finally:
        h.shutdown()


def test_화음이_되면_복귀도_그대로_한다(ros, taught, monkeypatch):
    """구현이 들어온 뒤에도 순서는 같다 — 화음 다음 복귀."""
    seen = []
    monkeypatch.setattr(MP, "strum_all", lambda *a, **k: seen.append("strum"))
    monkeypatch.setattr(MP, "return_home", lambda *a, **k: seen.append("home"))

    h = Harness(taught)
    try:
        h.node.dry_run = False
        h.node._holding_pick = False
        handle = h.send(StrumAll, "/robot/strum_all", StrumAll.Goal())
        assert seen == ["strum", "home"]
        assert handle.result.success is True
    finally:
        h.shutdown()


def test_goal_마다_실행_기록이_쌓인다(ros, taught, tmp_path):
    """
    노드 경로가 아무 자료도 안 남기고 있었다 (#58).

    9/21 통합 실기가 끝난 뒤 볼 수 있는 게 peg_state.json 의 각도 두 줄뿐이었다.
    goal 이 몇 초 걸렸는지가 없어서 서버 타임아웃을 정할 근거가 안 나온다.
    """
    h = Harness(taught)
    try:
        h.node._runlog_path = str(tmp_path / "session.json")
        h.node._runlog = {"tool": "robot_controller", "goals": []}
        goal = AdjustPeg.Goal(string_number=6, direction=1, angle_deg=30.0)
        h.send(AdjustPeg, "/robot/adjust_peg", goal)

        import json
        with open(h.node._runlog_path, encoding="utf-8") as f:
            saved = json.load(f)
        assert len(saved["goals"]) == 1
        g = saved["goals"][0]
        assert g["action"] == "adjust_peg"
        assert g["success"] is True
        assert g["elapsed_sec"] >= 0.0
        assert g["request"]["string_number"] == 6      # 요청이 그대로 남는다
    finally:
        h.shutdown()


def test_등록_Action은_기등록을_건너뛰고_미등록만_등록한다(ros, taught, monkeypatch, tmp_path):
    from tunio_interfaces.action import RegisterPeg
    from tunio_robot.peg_state import PegState
    h = Harness(taught)
    try:
        n = h.node
        n.dry_run = False
        n.pegs = PegState(str(tmp_path / "pegs.json"))
        n.pegs.register(6, 12.0)
        seen = []
        monkeypatch.setattr(n, "_cross_to", lambda side: seen.append(side))
        monkeypatch.setattr(MP, "register_peg", lambda *a, **k: seen.append("register") or 30.0)
        for no in (6, 5):
            r = h.send(RegisterPeg, "/robot/register_peg", RegisterPeg.Goal(string_number=no))
            assert r.result.success
        assert seen == ["left", "register"]
        assert n.pegs.roll_of(6) == 12.0
        assert n.pegs.roll_of(5) == 30.0
        assert n._holding_pick is False
    finally:
        h.shutdown()


def test_복귀_Action은_피크_반환후_대기로_간다(ros, taught, monkeypatch):
    from tunio_interfaces.action import ReturnHome
    h = Harness(taught)
    try:
        h.node.dry_run = False
        h.node._holding_pick = True
        seen = []
        monkeypatch.setattr(MP, "pick_return", lambda *a, **k: seen.append("return"))
        monkeypatch.setattr(MP, "return_home", lambda *a, **k: seen.append("home"))
        res = h.send(ReturnHome, "/robot/return_home", ReturnHome.Goal())
        assert res.result.success
        assert seen == ["return", "home"]
        assert h.node._holding_pick is False
    finally:
        h.shutdown()


@pytest.mark.parametrize("side,j1,skip", [
    ("right", -30.0, True), ("right", 30.0, True),
    ("right", -30.01, False), ("right", 30.01, False),
    ("left", 150.0, True), ("left", 210.0, True),
    ("left", 149.99, False), ("left", 210.01, False),
    ("left", -180.0, False), ("left", 0.0, False),
    ("right", None, False), ("left", float("nan"), False),
])
def test_페그_경유는_실제_J1과_30도_경계로_결정한다(monkeypatch, side, j1, skip):
    went = []
    route = TPRoute("transit", [90, 0, 0, 0, 0, None], True)
    monkeypatch.setattr(MP, "_current_j1", lambda: j1)
    monkeypatch.setattr(MP, "goto_route", lambda r, m: went.append(r))
    MP.approach_peg_side(side, route, {})
    assert bool(went) is not skip


@pytest.mark.parametrize("route", [None, TPRoute("bad", [-90, 0, 0, 0, 0, None], True)])
def test_페그_경유는_양의_90도만_허용한다(monkeypatch, route):
    monkeypatch.setattr(MP, "_current_j1", lambda: 90.0)
    with pytest.raises(MP.MotionError, match=r"J1=\+90"):
        MP.approach_peg_side("left", route, {})


def test_미등록_회전요청은_등록이나_피크반환_없이_실패한다(ros, taught, monkeypatch, tmp_path):
    from tunio_robot.peg_state import PegState
    h = Harness(taught)
    try:
        n = h.node
        n.dry_run = False
        n.pegs = PegState(str(tmp_path / "empty.json"))
        with mock.patch.object(n, "_release_pick") as release, \
                mock.patch.object(n, "_ensure_registered") as register, \
                mock.patch.object(n, "_cross_to") as cross, \
                mock.patch.object(MP, "adjust_peg") as adjust:
            result = h.send(AdjustPeg, "/robot/adjust_peg",
                            AdjustPeg.Goal(string_number=1, direction=1, angle_deg=3.0))
            assert not result.result.success
            assert "register_peg" in result.result.message
            for operation in (release, register, cross, adjust):
                operation.assert_not_called()
    finally:
        h.shutdown()


@pytest.mark.parametrize("fails", [False, True])
def test_반환_후에는_높이검사_경로를_쓰고_실패해도_직행으로_바꾸지_않는다(
        ros, taught, monkeypatch, tmp_path, fails):
    from tunio_robot.peg_state import PegState
    h = Harness(taught)
    try:
        n = h.node
        n.dry_run = False
        n.pegs = PegState(str(tmp_path / "state.json"))
        n.pegs.register(2, 0.0)
        n._holding_pick = True
        monkeypatch.setattr(MP, "pick_return", lambda *a, **k: None)

        def adjust(*a, **kw):
            assert kw["from_pick"] is True
            assert kw["side"] == "right"
            if fails:
                raise MP.MotionError("경로 높이 부족")
        monkeypatch.setattr(MP, "adjust_peg", adjust)
        with mock.patch.object(n, "_cross_to") as cross:
            result = h.send(AdjustPeg, "/robot/adjust_peg",
                            AdjustPeg.Goal(string_number=2, direction=1, angle_deg=3.0))
            assert result.result.success is not fails
            cross.assert_not_called()
            assert n._from_pick is fails
    finally:
        h.shutdown()


# ---------------------------------------------------------------
# 피크 집기 확인 (#39) — 폭을 읽어 빈손이면 거치대를 거쳐 다시 집는다.
#
# 장수는 안 가린다 (2026-09-22 결정). "집었나 / 못 집었나" 만 본다.
# ---------------------------------------------------------------

def test_확인이_꺼져_있으면_예전처럼_한_번만_집는다(monkeypatch):
    """pick_stack 이 없거나 verify_pick=false 면 _verify_pick 이 skip 을 낸다."""
    node = _fake_node(holding=False, verify="skip")
    pick_up = mock.MagicMock()
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    monkeypatch.setattr(MP, "pick_return", pick_return)
    RobotController._ensure_pick(node)
    assert pick_up.call_count == 1
    pick_return.assert_not_called()
    assert node._holding_pick is True


def test_폭이_맞으면_한_번에_끝난다(monkeypatch):
    node = _fake_node(holding=False, verify="hold")
    pick_up = mock.MagicMock()
    pick_return = mock.MagicMock()
    monkeypatch.setattr(MP, "pick_up", pick_up)
    monkeypatch.setattr(MP, "pick_return", pick_return)
    RobotController._ensure_pick(node)
    assert pick_up.call_count == 1
    pick_return.assert_not_called()
    assert node._holding_pick is True


def test_빈손이면_재집기_두번후_홈으로_간다(monkeypatch):
    node = _fake_node(holding=False, verify="empty")
    events = []

    def pick_up(*args, **kwargs):
        events.append("닫힘")
        kwargs["verify_pick"]()
        events.append("들어올림")

    monkeypatch.setattr(MP, "pick_up", pick_up)
    monkeypatch.setattr(MP, "retreat_pick", lambda *a: events.append("수직후퇴"))
    monkeypatch.setattr(MP, "pick_return", lambda *a, **kw: events.append("거치대반환"))
    monkeypatch.setattr(MP, "return_home", lambda *a: events.append("홈"))
    with pytest.raises(MP.MotionError, match="홈 복귀 완료"):
        RobotController._ensure_pick(node)
    assert events == ["닫힘", "수직후퇴", "거치대반환", "닫힘", "수직후퇴",
                      "거치대반환", "닫힘", "수직후퇴", "홈"]
    assert node._holding_pick is False
    with pytest.raises(MP.MotionError):
        RobotController._ensure_pick(node)
    assert len(events) == 9


def test_큰_폭은_빈손이_아니므로_이동하지_않는다(monkeypatch):
    node = _fake_node(holding=True, verify="unknown")
    home = mock.MagicMock()
    monkeypatch.setattr(MP, "return_home", home)
    with pytest.raises(MP.MotionError, match="비정상"):
        node._check_pick_or_recover()
    assert node._holding_pick is None
    home.assert_not_called()


def test_발현후_빈손이면_홈으로_간다(monkeypatch):
    node = _fake_node(holding=True, verify="empty")
    monkeypatch.setattr(MP, "pick_return", mock.MagicMock())
    monkeypatch.setattr(MP, "retreat_pick", mock.MagicMock())
    monkeypatch.setattr(MP, "pick_up", lambda *a, **kw: kw["verify_pick"]())
    home = mock.MagicMock()
    monkeypatch.setattr(MP, "return_home", home)
    with pytest.raises(MP.MotionError, match="홈 복귀 완료"):
        node._check_pick_or_recover()
    home.assert_called_once()
    assert node._holding_pick is False


@pytest.mark.parametrize("width, expected", [
    (13.0, "empty"), (13.2, "hold"), (13.7, "hold"),
    (16.0, "hold"), (19.6, "unknown"), (38.0, "unknown"),
])
def test_실제_폭_판정(width, expected):
    from tunio_robot import teach_points as TP
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=True),
        points=TP.load(CONFIG),
        _width_reader=lambda: lambda: width,
        get_logger=lambda: mock.MagicMock(),
    )
    node._notify_pick = RobotController._notify_pick.__get__(node)
    assert RobotController._verify_pick(node) == expected


def test_통신_실패는_빈손이_아니다():
    from tunio_robot import grip_width as GW
    from tunio_robot import teach_points as TP
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=True),
        points=TP.load(CONFIG),
        _width_reader=lambda: mock.MagicMock(side_effect=GW.WidthError("연결 실패")),
        get_logger=lambda: mock.MagicMock(),
    )
    node._notify_pick = RobotController._notify_pick.__get__(node)
    assert RobotController._verify_pick(node) == "skip"


def test_재개는_기억한_피크를_다시_확인한다():
    node = _fake_node(holding=True, verify="hold")
    node._verify_pick = mock.MagicMock(return_value="hold")
    RobotController._verify_pick_on_resume(node)
    node._verify_pick.assert_called_once()


def test_재개때_읽지_못해도_동작을_계속한다():
    node = _fake_node(holding=True, verify="skip")
    RobotController._verify_pick_on_resume(node)
    assert node._holding_pick is True
    assert node._pick_fault is None


def test_피크를_반환한_상태에는_피크_문턱을_적용하지_않는다():
    node = _fake_node(holding=False)
    node._verify_pick = mock.MagicMock()
    RobotController._verify_pick_on_resume(node)
    node._verify_pick.assert_not_called()


def test_홈_복귀가_실패해도_피크_오류는_유지한다(monkeypatch):
    node = _fake_node(holding=True, verify="empty")
    monkeypatch.setattr(MP, "pick_return", mock.MagicMock())
    monkeypatch.setattr(MP, "retreat_pick", mock.MagicMock())
    monkeypatch.setattr(MP, "pick_up", lambda *a, **kw: kw["verify_pick"]())
    monkeypatch.setattr(MP, "return_home", mock.MagicMock(side_effect=MP.MotionError("거부")))
    with pytest.raises(MP.MotionError, match="홈 복귀 실패"):
        node._check_pick_or_recover()
    assert node._pick_fault


def test_확인_중_예외면_집기_상태는_모름이다(monkeypatch):
    node = _fake_node(holding=False)
    node._verify_pick = mock.MagicMock(side_effect=RuntimeError("읽기 예외"))
    monkeypatch.setattr(MP, "pick_up", lambda *a, **kw: kw["verify_pick"]())
    with pytest.raises(RuntimeError):
        RobotController._ensure_pick(node)
    assert node._holding_pick is None


@pytest.mark.parametrize('width, resumed, success, homes, strokes, reads', [
    (13.7, True, True, 0, 1, 2),
    (13.7, False, True, 0, 1, 1),
    (13.0, True, False, 1, 0, 1),
    (19.6, True, False, 0, 0, 1),
])
def test_피크_판정은_로봇내부_Action에서_수행한다(
        ros, taught, monkeypatch, width, resumed, success, homes, strokes, reads):
    h = Harness(taught)
    home = mock.MagicMock()
    stroke = mock.MagicMock(side_effect=lambda *a, **kw: kw['verify_pick']())
    monkeypatch.setattr(MP, 'return_home', home)
    monkeypatch.setattr(MP, 'pick_return', mock.MagicMock())
    monkeypatch.setattr(MP, 'retreat_pick', mock.MagicMock())
    monkeypatch.setattr(MP, 'pick_up', lambda *a, **kw: kw['verify_pick']())
    monkeypatch.setattr(MP, 'pluck_string', stroke)
    try:
        h.node.dry_run = False
        h.node._holding_pick = True
        h.node._read_width = mock.MagicMock(return_value=width)
        result = h.send(PlayString, '/robot/play_string',
                        PlayString.Goal(string_number=6, resumed=resumed)).result
        assert result.success == success
        assert home.call_count == homes
        assert stroke.call_count == strokes
        assert h.node._read_width.call_count == (3 if width == 13.0 else reads)
        if success:
            assert result.message == ''   # 정상 피크 상태를 서버에 별도 보고하지 않는다
        elif homes:
            assert '피크 없음' in result.message
    finally:
        h.shutdown()


@pytest.mark.parametrize('values, expected, warning', [
    ('recover_hold', 'hold', False),
    ('recover_empty', 'empty', False),
    ('unreadable', 'skip', True),
])
def test_폭읽기는_실패할때만_한번_더_시도한다(values, expected, warning):
    from tunio_robot import grip_width as GW
    from tunio_robot import teach_points as TP
    second = {'recover_hold': 13.7, 'recover_empty': 13.0,
              'unreadable': GW.WidthError('두번째 실패')}[values]
    reader = mock.MagicMock(side_effect=[GW.WidthError('첫번째 실패'), second])
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=True),
        points=TP.load(CONFIG), _width_reader=lambda: reader,
        get_logger=lambda: mock.MagicMock(), _pick_warnings=[],
    )
    node._notify_pick = RobotController._notify_pick.__get__(node)
    assert RobotController._verify_pick(node) == expected
    assert reader.call_count == 2
    assert bool(node._pick_warnings) == warning


def test_정상_폭은_한번만_읽는다():
    from tunio_robot import teach_points as TP
    reader = mock.MagicMock(return_value=13.7)
    node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=True),
        points=TP.load(CONFIG), _width_reader=lambda: reader,
        get_logger=lambda: mock.MagicMock(),
    )
    node._notify_pick = RobotController._notify_pick.__get__(node)
    assert RobotController._verify_pick(node) == 'hold'
    reader.assert_called_once()


@pytest.mark.parametrize('second, success, homes, strokes', [
    ('unreadable', True, 0, 1),
    (13.7, True, 0, 1),
    (13.0, False, 1, 0),
])
def test_재개_재읽기와_경고_Result_ROS왕복(
        ros, taught, monkeypatch, second, success, homes, strokes):
    from tunio_robot import grip_width as GW
    h = Harness(taught)
    home = mock.MagicMock()
    stroke = mock.MagicMock(side_effect=lambda *a, **kw: kw['verify_pick']())
    monkeypatch.setattr(MP, 'return_home', home)
    monkeypatch.setattr(MP, 'pick_return', mock.MagicMock())
    monkeypatch.setattr(MP, 'retreat_pick', mock.MagicMock())
    monkeypatch.setattr(MP, 'pick_up', lambda *a, **kw: kw['verify_pick']())
    monkeypatch.setattr(MP, 'pluck_string', stroke)
    try:
        h.node.dry_run = False
        h.node._holding_pick = True
        outcome = GW.WidthError('재읽기 실패') if second == 'unreadable' else second
        warnings = []
        feedback_received = threading.Event()

        def on_feedback(msg):
            warnings.append(msg.feedback.warning)
            feedback_received.set()

        # 재개 검사 두 번 + 발현 후 검사는 정상. 앞선 경고는 Result까지 보존한다.
        h.node._read_width = mock.MagicMock(
            side_effect=([GW.WidthError('최초 실패'), 13.0, 13.0, 13.0]
                         if second == 13.0 else [GW.WidthError('최초 실패'), outcome, 13.7]))
        result = h.send(PlayString, '/robot/play_string',
                        PlayString.Goal(string_number=6, resumed=True),
                        feedback_callback=on_feedback).result
        assert result.success == success
        assert home.call_count == homes
        assert stroke.call_count == strokes
        if second == 'unreadable':
            assert feedback_received.wait(1)
            assert warnings == [result.message]
            assert '피크 폭 확인 불가' in result.message
            assert h.node._holding_pick is True and h.node._pick_fault is None
            # 다음 정상 Action에는 과거 경고가 새어 나가지 않는다.
            h.node._read_width.side_effect = None
            h.node._read_width.return_value = 13.7
            next_result = h.send(PlayString, '/robot/play_string',
                                 PlayString.Goal(string_number=6)).result
            assert next_result.success and next_result.message == ''
        elif success:
            assert result.message == ''
        else:
            assert '피크 없음' in result.message
    finally:
        h.shutdown()


def test_폭경고가_있어도_실제_동작실패를_성공으로_바꾸지_않는다(ros, taught, monkeypatch):
    from tunio_robot import grip_width as GW
    h = Harness(taught)
    monkeypatch.setattr(MP, 'pluck_string', mock.MagicMock(side_effect=MP.MotionError('이동 거부')))
    try:
        h.node.dry_run = False
        h.node._holding_pick = True
        h.node._read_width = mock.MagicMock(side_effect=GW.WidthError('통신 실패'))
        result = h.send(PlayString, '/robot/play_string',
                        PlayString.Goal(string_number=6, resumed=True)).result
        assert not result.success
        assert '이동 거부' in result.message
        assert '피크 폭 확인 불가' in result.message
        assert h.node._read_width.call_count == 2
    finally:
        h.shutdown()


@pytest.mark.parametrize('stage', ['pickup', 'post_pluck', 'resume'])
@pytest.mark.parametrize('readings, recovery_count, fails, notice', [
    ([13.7], 0, False, False),
    ([13.0, 13.7], 1, False, False),
    ([13.0, 13.0, 13.7], 2, False, False),
    ([13.0, 13.0, 13.0], 2, True, True),
    (['error', 'error'], 0, False, True),
    ([13.0, 'error', 'error'], 1, False, True),
    ([13.0, 13.0, 'error', 'error'], 2, False, True),
])
def test_세구간_공통_실측판정_재집기_폭실패_분리(
        stage, readings, recovery_count, fails, notice, monkeypatch):
    from tunio_robot import grip_width as GW
    from tunio_robot import teach_points as TP
    node = _fake_node(holding=stage != 'pickup')
    node.points.pick_stack = TP.load(CONFIG).pick_stack
    node.get_parameter = lambda name: SimpleNamespace(value=True)
    reader = mock.MagicMock(side_effect=[
        GW.WidthError('읽기 실패') if x == 'error' else x for x in readings])
    node._width_reader = lambda: reader
    node._verify_pick = RobotController._verify_pick.__get__(node)
    events = []
    node._pick_warning_callback = lambda message: events.append(('notice', message))

    def pick(*args, **kwargs):
        events.append(('pick', None))
        kwargs['verify_pick']()
        events.append(('lift', None))

    monkeypatch.setattr(MP, 'pick_up', pick)
    monkeypatch.setattr(MP, 'pick_return', lambda *a, **kw: events.append(('holder', None)))
    monkeypatch.setattr(MP, 'retreat_pick', lambda *a: events.append(('retreat', None)))
    monkeypatch.setattr(MP, 'return_home', lambda *a: events.append(('home', None)))

    def run():
        if stage == 'pickup':
            RobotController._ensure_pick(node)
        elif stage == 'resume':
            RobotController._verify_pick_on_resume(node)
        else:
            node._check_pick_or_recover()

    if fails:
        with pytest.raises(MP.MotionError, match='재집기 2회 실패'):
            run()
    else:
        run()
    kinds = [e[0] for e in events]
    assert kinds.count('holder') == recovery_count
    assert kinds.count('pick') == recovery_count + (1 if stage == 'pickup' else 0)
    assert kinds.count('home') == int(fails)
    assert ('notice' in kinds) == notice
    assert reader.call_count == len(readings)
    if fails:
        assert kinds.index('notice') < kinds.index('home')
        assert node._holding_pick is False
        assert node._pick_fault
    else:
        assert node._holding_pick is True
        assert node._pick_fault is None
