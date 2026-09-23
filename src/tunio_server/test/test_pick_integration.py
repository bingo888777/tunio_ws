"""실제 ROS Action·재개/취소 Service 검증. 이동·폭·오디오는 대역이다."""
import threading
import time
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tunio_interfaces.action import MeasurePitch
from tunio_interfaces.msg import TuningState
from tunio_interfaces.srv import CommandTuning
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP
from tunio_robot.robot_controller import RobotController
from tunio_server.task_manager import Aborted, Cancelled, TaskManager


def wait_until(predicate, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate(), '상태 전환 시간 초과'


@pytest.mark.parametrize('stage', ['pickup', 'post_pluck', 'resume'])
@pytest.mark.parametrize('outcome', ['recovered', 'exhausted', 'unreadable', 'home_error'])
@pytest.mark.parametrize('cancel', [False, True])
def test_pick_refill_through_resume_service(stage, outcome, cancel, monkeypatch):
    monkeypatch.setattr(MP, 'connect', Mock(side_effect=AssertionError('실물 연결 금지')))
    rclpy.init()
    robot = RobotController()
    manager = TaskManager()
    audio = rclpy.create_node('pick_test_audio')
    executor = MultiThreadedExecutor(num_threads=4)
    events, results, errors = [], [], []

    def measure(goal):
        events.append('measure')
        goal.publish_feedback(MeasurePitch.Feedback(ready=True))
        req = goal.request
        if stage == 'post_pluck' and outcome != 'recovered' and events.count('measure') == 1:
            wait_until(lambda: goal.is_cancel_requested)
            events.append('audio_cancelled')
            goal.canceled()
        else:
            goal.succeed()
        return MeasurePitch.Result(success=True, job_id=req.job_id,
                                   measurement_id=req.measurement_id,
                                   string_number=req.string_number, frequency_hz=82.4)

    server = ActionServer(audio, MeasurePitch, '/audio/measure_pitch', measure,
                          cancel_callback=lambda goal: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    robot.dry_run = False
    monkeypatch.setattr(robot.pegs, 'roll_of', lambda no: 0.0)
    robot._holding_pick = stage != 'pickup'
    failed_round = [GW.WidthError('통신 실패')] * 6 if outcome == 'unreadable' else [13.0] * 3
    readings = ([13.0, 13.0, 13.7] if outcome == 'recovered' else failed_round * 2)
    robot._read_width = Mock(side_effect=readings + [13.7, 13.7])

    def pick(*args, **kwargs):
        events.append('pick')
        kwargs['verify_pick']()
        events.append('lift')

    def pluck(*args, **kwargs):
        if kwargs.get('prepare_only'):
            events.append('prepare')
            return
        events.append('pluck')
        kwargs['verify_pick']()

    def home(*args):
        events.append('home')
        if outcome == 'home_error':
            raise MP.MotionError('홈 이동 실패')

    monkeypatch.setattr(MP, 'pick_up', pick)
    monkeypatch.setattr(MP, 'pluck_string', pluck)
    monkeypatch.setattr(MP, 'retreat_pick', lambda *a: events.append('retreat'))
    monkeypatch.setattr(MP, 'pick_return', lambda *a, **kw: events.append('holder'))
    monkeypatch.setattr(MP, 'return_home', home)
    manager.job_id = 'pick-integration'
    manager.active_settings = manager.settings
    if stage == 'resume':
        manager._resumed_since_play.set()
    for node in (robot, manager, audio):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def run():
        try:
            results.append(manager._measure_once(6))
        except (Aborted, Cancelled) as exc:
            errors.append(exc)

    worker = threading.Thread(target=run)
    manager._worker = worker

    def command(name):
        client = audio.create_client(CommandTuning, '/tuning/' + name)
        try:
            assert client.wait_for_service(timeout_sec=3)
            future = client.call_async(CommandTuning.Request())
            wait_until(future.done)
            return future.result()
        finally:
            audio.destroy_client(client)

    try:
        assert manager.cli_prepare.wait_for_server(timeout_sec=5)
        assert manager.cli_play.wait_for_server(timeout_sec=5)
        assert manager.cli_measure.wait_for_server(timeout_sec=5)
        worker.start()
        if outcome in ('exhausted', 'unreadable'):
            for round_no in (1, 2):
                wait_until(lambda: manager.phase == TuningState.PHASE_PAUSED)
                assert events.count('home') == round_no
                assert '재보충하고 재개 버튼' in manager.robot_notice
                assert robot._holding_pick is False and robot._pick_refill_pending
                assert not manager._active_goals
                if stage == 'post_pluck':
                    assert 'audio_cancelled' in events
                snapshot = events[:]
                time.sleep(0.1)
                assert events == snapshot  # 입력 없는 대기에는 이동·측정이 없다
                assert not command('start').accepted
                if cancel:
                    assert command('cancel').accepted
                    break
                assert command('resume').accepted
                wait_until(lambda: events.count('pick') > snapshot.count('pick'))
            worker.join(5)
            assert not worker.is_alive()
            if cancel:
                assert len(errors) == 1 and isinstance(errors[0], Cancelled)
                assert not results
                assert events == snapshot
            else:
                assert not errors and results[0].ok
                assert events.count('home') == 2
                assert robot._holding_pick and not robot._pick_refill_pending
                assert robot._pick_fault is None
                assert manager.robot_notice == ''
                assert events.count('pluck') == (2 if stage == 'post_pluck' else 1)
        else:
            worker.join(5)
            assert not worker.is_alive()
            if outcome == 'home_error':
                assert len(errors) == 1 and isinstance(errors[0], Aborted)
                assert not robot._pick_refill_pending
                assert robot._pick_fault and robot._holding_pick is None
                assert manager.phase != TuningState.PHASE_PAUSED
                if stage == 'post_pluck':
                    wait_until(lambda: 'audio_cancelled' in events)
            else:
                assert not errors and results[0].ok
                assert 'home' not in events
                assert robot._holding_pick and robot._pick_fault is None
        assert not manager._active_goals
    finally:
        manager._cancel.set()
        manager._pause.clear()
        if worker.ident:
            worker.join(5)
        executor.shutdown()
        thread.join(3)
        server.destroy()
        for node in (robot, manager, audio):
            node.destroy_node()
        rclpy.shutdown()
