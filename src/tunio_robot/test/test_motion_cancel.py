"""실제 로봇 없이 MoveStop 요청·IDLE 판정·후속 명령 차단을 검증한다."""
from concurrent.futures import Future
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_robot import motion_primitives as MP
from tunio_robot.motion_cancel import (
    CancelSession, GuardedAPI, MotionCancelled, request_and_confirm_stop)
from tunio_robot.robot_controller import RobotController


def client(responses):
    calls = []
    responses = iter(responses)

    def send(req):
        calls.append(req)
        f = Future()
        value = next(responses)
        if value is not None:
            f.set_result(value)
        return f
    return SimpleNamespace(service_is_ready=lambda: True, call_async=send), calls


def test_stop_mode_2_and_busy_then_idle():
    session = CancelSession()
    session.work_done.set()
    stop, calls = client([SimpleNamespace(success=True)] * 2)
    check, _ = client([SimpleNamespace(success=True, status=s) for s in (2, 0)])
    request_and_confirm_stop(stop, check, session, 1, .001)
    assert session.stop_done.is_set()
    assert session.error is None
    assert [r.stop_mode for r in calls] == [2, 2]


@pytest.mark.parametrize('kind', ['stop_failure', 'unknown', 'timeout', 'work_running'])
def test_failed_or_unconfirmed_stop_is_not_success(kind):
    session = CancelSession()
    if kind != 'work_running':
        session.work_done.set()
    stop, _ = client([SimpleNamespace(success=kind != 'stop_failure')] * 100)
    check, _ = client([None if kind == 'timeout' else
                      SimpleNamespace(success=True, status=99)] * 100)
    request_and_confirm_stop(stop, check, session, .02, .001)
    assert session.stop_done.is_set()
    assert session.error


def test_cancel_during_blocking_move_prevents_grip_and_cleanup_retreat():
    session = CancelSession()
    entered = threading.Event()
    stopped = threading.Event()
    raw = Mock()

    def move():
        entered.set()
        assert stopped.wait(1)
        return 0  # 두산 래퍼 성공 반환 규약
    raw.movej.side_effect = move
    api = GuardedAPI(raw, session.check)
    errors = []

    def work():
        try:
            api.movej()
            api.set_digital_output(1, 1)
        except Exception as exc:
            errors.append(exc)
            with pytest.raises(MotionCancelled):
                api.movel()
            api.release_force()
    thread = threading.Thread(target=work)
    thread.start()
    assert entered.wait(1)
    session.requested.set()
    stopped.set()
    thread.join(1)
    assert isinstance(errors[0], MotionCancelled)
    raw.set_digital_output.assert_not_called()
    raw.movel.assert_not_called()
    raw.release_force.assert_called_once()


def test_robot_cancel_stops_and_latches_recovery(monkeypatch):
    # MoveStop 대역이 기존 블로킹 이동을 풀어 주는 상황. 실제 API 호출 없음.
    entered = threading.Event()
    stopped = threading.Event()
    stop_calls = []

    def stop_request(stop, check, session, timeout, interval):
        stop_calls.append(True)
        stopped.set()
        assert session.work_done.wait(1)
        session.stop_done.set()
    monkeypatch.setattr('tunio_robot.robot_controller.request_and_confirm_stop', stop_request)
    monkeypatch.setattr(MP, '_cancel_check', None)
    monkeypatch.setattr(MP, '_pick_gripped', True)   # 쥐고 있는 상황
    plans = []
    node = SimpleNamespace(
        _lock=threading.Lock(), _running=None, _cancel_sessions={},
        _recovery_required=False, _prepared_pluck=None, _holding_pick=True,
        _recovery_plan=None, _recovering=False, _recovery_done=threading.Event(),
        _recovery_wait=.05, _lift_mm=30.0,
        _recover_after_cancel=lambda plan: plans.append(plan),   # 끝나지 않는 복구
        dry_run=False, _stop_timeout=1, _stop_interval=.001,
        _stop_client=Mock(), _motion_client=Mock(), get_logger=lambda: Mock(), _record=Mock())
    node._recovery_done.set()
    goal = Mock(is_active=True)
    goal.canceled.side_effect = lambda: setattr(goal, 'is_active', False)

    def result_type(**kw):
        return SimpleNamespace(**kw)

    results = []

    def work():
        entered.set()
        assert stopped.wait(1)
        MP._cancel_check()
    worker = threading.Thread(target=lambda: results.append(
        RobotController._run(node, 'play_string', goal, result_type, work)))
    worker.start()
    assert entered.wait(1)
    RobotController.on_cancel(node, goal)
    worker.join(2)
    assert not worker.is_alive()
    assert stop_calls == [True]
    goal.canceled.assert_called_once()
    goal.succeed.assert_not_called()
    assert not results[0].success
    assert '홈 복귀 중' in results[0].message
    assert node._holding_pick is None
    assert node._recovery_required
    # 취소 순간의 (하던 일, 쥐고 있었나) 가 복구 계획으로 넘어간다
    assert plans == [('play_string', True)]
    assert node._recovering and not node._recovery_done.is_set()
    # 복구 중엔 새 goal 을 거부하지 않는다 — 받아서 복구가 끝나길 기다린다
    assert RobotController.on_goal(node, None).name == 'ACCEPT'
    assert RobotController.on_cancel(node, goal).name == 'REJECT'
    # 복구가 안 끝나면 그 goal 은 기다리다 사유를 싣고 실패한다
    late = Mock(is_active=True)
    r = RobotController._run(node, 'register_peg', late, result_type, lambda: None)
    assert not r.success and '복구가 끝나지 않았다' in r.message
    late.abort.assert_called_once()
    assert node._running is None


@pytest.mark.parametrize('gripped', [True, False])
def test_cancel_during_pick_transition_uses_gripper_record(monkeypatch, gripped):
    # _ensure_pick 이 집으러 가는 동안 _holding_pick 은 None 이다. 그 사이 취소가 오면
    # pick_up 이 실제로 집었는지(MP._pick_gripped)로 복구 계획을 정한다
    monkeypatch.setattr('tunio_robot.robot_controller.request_and_confirm_stop',
                        lambda *a: a[2].stop_done.set())
    monkeypatch.setattr(MP, '_pick_gripped', gripped)
    node = SimpleNamespace(
        _lock=threading.Lock(), _running='prepare_pluck', _cancel_sessions={},
        _recovery_required=False, _prepared_pluck=None, _holding_pick=None,
        _recovery_plan=None, dry_run=False, _stop_timeout=1, _stop_interval=.001,
        _stop_client=Mock(), _motion_client=Mock(), get_logger=lambda: Mock())
    goal = Mock(is_active=True)
    assert RobotController.on_cancel(node, goal).name == 'ACCEPT'
    assert node._recovery_plan == ('prepare_pluck', gripped)


def test_release_clears_and_pick_up_grip_sets_the_record(monkeypatch):
    calls = []
    monkeypatch.setattr(MP, '_grip', lambda preset: calls.append(preset) or 0)
    monkeypatch.setattr(MP, '_pick_gripped', True)
    MP._release()
    assert calls == ['open'] and MP._pick_gripped is False


def _recovery_node(held, running='adjust_peg'):
    calls = []
    node = SimpleNamespace(
        _lock=threading.Lock(), _running=None, _cancel_sessions={},
        _recovery_required=True, _recovering=True, _recovery_plan=(running, held),
        _recovery_done=threading.Event(), _prepared_pluck=(6, 'x'),
        _holding_pick=None, _side='left', _from_pick=False, _lift_mm=30.0,
        _recovery_wait=1, dry_run=False, _stop_client=Mock(), _motion_client=Mock(),
        points=SimpleNamespace(motion={'m': 1}, frame=lambda name: f'frame:{name}'),
        get_logger=lambda: Mock())
    node._stop_client.service_is_ready.return_value = True
    node._motion_client.service_is_ready.return_value = True

    def release_pick(side=None):
        calls.append(('release_pick', node._holding_pick))
        node._holding_pick = False
    node._release_pick = release_pick
    return node, calls


@pytest.mark.parametrize('held', [True, False, None])
def test_recovery_after_cancel_lifts_then_returns_home_and_unlocks(monkeypatch, held):
    node, calls = _recovery_node(held)
    monkeypatch.setattr(MP, '_release', lambda: calls.append(('open',)))
    monkeypatch.setattr(MP, 'cancel_recover_lift',
                        lambda motion, mm, logger=None: calls.append(('lift', mm)))
    monkeypatch.setattr(MP, 'return_home', lambda pose, motion: calls.append(('home', pose)))

    RobotController._recover_after_cancel(node, node._recovery_plan)

    if held is True:
        # 피크를 쥔 채면 절대 열지 않는다 — 올라가서 반환하고 홈
        assert calls == [('lift', 30.0), ('release_pick', True), ('home', 'frame:standby')]
        assert node._holding_pick is False
    elif held is False:
        # 페그를 물고 있을 수 있다 — 먼저 열어 페그를 남기고 올라가 홈
        assert calls == [('open',), ('lift', 30.0), ('home', 'frame:standby')]
        assert node._holding_pick is False
    else:
        # 모르면 열지 않는다. 피크 상태는 '모름' 그대로 남긴다
        assert calls == [('lift', 30.0), ('home', 'frame:standby')]
        assert node._holding_pick is None
    assert not node._recovery_required and not node._recovering
    assert node._recovery_done.is_set() and node._prepared_pluck is None
    assert node._recovery_plan is None
    assert RobotController.on_goal(node, None).name == 'ACCEPT'


def test_recovery_failure_keeps_lock_until_restart(monkeypatch):
    node, calls = _recovery_node(True, running='play_string')
    monkeypatch.setattr(MP, 'cancel_recover_lift',
                        lambda motion, mm, logger=None: calls.append(('lift', mm)))

    def boom(pose, motion):
        raise MP.MotionError('movej 거부')
    monkeypatch.setattr(MP, 'return_home', boom)

    RobotController._recover_after_cancel(node, node._recovery_plan)

    assert node._recovery_required and not node._recovering
    assert node._recovery_done.is_set()
    assert node._holding_pick is None            # 어디서 멈췄는지 모른다 — 다시 '모름'
    assert RobotController.on_goal(node, None).name == 'REJECT'
    r = RobotController._run(node, 'register_peg', Mock(is_active=True),
                             lambda **kw: SimpleNamespace(**kw), lambda: None)
    assert not r.success and '노드 재시작' in r.message


def test_goal_arriving_during_recovery_waits_then_runs():
    node, _ = _recovery_node(False)
    node._record = Mock()
    entered = threading.Event()
    goal = Mock(is_active=True, is_cancel_requested=False)
    results = []

    def work():
        entered.set()
    t = threading.Thread(target=lambda: results.append(RobotController._run(
        node, 'register_peg', goal, lambda **kw: SimpleNamespace(**kw), work)))
    t.start()
    assert not entered.wait(.2)                   # 복구가 끝나기 전엔 시작하지 않는다
    with node._lock:                              # 복구 성공을 흉내 낸다
        node._recovery_required = False
        node._recovering = False
        node._recovery_done.set()
    t.join(2)
    assert entered.is_set() and results and results[0].success
    goal.succeed.assert_called_once()


def test_ros_cancel_reaches_canceled_result_without_real_robot():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from tunio_interfaces.action import ReturnHome

    rclpy.init()
    robot = RobotController()
    client_node = rclpy.create_node('cancel_test_client')
    executor = MultiThreadedExecutor(num_threads=4)
    entered = threading.Event()
    release = threading.Event()

    def hold():
        entered.set()
        assert release.wait(3)
    robot._return_to_home = hold
    # Action 자체는 실제 ROS로 왕복하고 work만 대역으로 교체한다.
    original_run = robot._run
    robot._run = lambda name, goal, result_type, work: original_run(
        name, goal, result_type, hold)
    executor.add_node(robot)
    executor.add_node(client_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        client = ActionClient(client_node, ReturnHome, '/robot/return_home')
        assert client.wait_for_server(timeout_sec=3)
        accepted = client.send_goal_async(ReturnHome.Goal())
        event = threading.Event()
        accepted.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        goal = accepted.result()
        assert goal.accepted and entered.wait(3)
        future = goal.get_result_async()
        cancel = goal.cancel_goal_async()
        event.clear()
        cancel.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        assert cancel.result().goals_canceling
        release.set()
        event.clear()
        future.add_done_callback(lambda _: event.set())
        assert event.wait(3)
        assert future.result().status == 5
        assert not future.result().result.success
    finally:
        release.set()
        executor.shutdown()
        thread.join(3)
        robot.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
