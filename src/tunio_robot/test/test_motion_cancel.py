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
    node = SimpleNamespace(
        _lock=threading.Lock(), _running=None, _cancel_sessions={},
        _recovery_required=False, _prepared_pluck=None, _holding_pick=True,
        dry_run=False, _stop_timeout=1, _stop_interval=.001,
        _stop_client=Mock(), _motion_client=Mock(), get_logger=lambda: Mock(), _record=Mock())
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
    assert node._holding_pick is None
    assert node._recovery_required
    assert RobotController.on_goal(node, None).name == 'REJECT'
    assert RobotController.on_cancel(node, goal).name == 'REJECT'


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
