"""준비가 끝나기 전에는 마이크 타이머와 스트로크를 시작하지 않는다 (#89)."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_server import task_manager as TM


def scenario(ready=True, outcomes=(True,)):
    calls = []
    outcomes = iter(outcomes)
    node = SimpleNamespace(
        job_id='prepare-test', measurement_id=0, _registered_pegs={6},
        active_settings=SimpleNamespace(measure_timeout_sec=10., onset_timeout_sec=10.),
        _cancel=threading.Event(), _resumed_since_play=threading.Event(),
        _set_robot_notice=Mock(), cli_prepare=object(), cli_play=object(),
        _gate=Mock(), _set_phase=Mock(), _track=Mock(), _untrack=Mock(),
        _publish_state=Mock(), get_logger=lambda: Mock(),
        _p=lambda key: .01 if key == 'ready_wait_sec' else 30.,
        _wait=lambda value, *args: value)

    def send(goal, feedback_callback):
        calls.append('measure')
        if ready:
            calls.append('ready')
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(ready=True)))
        ok = next(outcomes)
        result = SimpleNamespace(result=SimpleNamespace(
            job_id=goal.job_id, string_number=goal.string_number,
            measurement_id=goal.measurement_id, success=ok, frequency_hz=82.4,
            failure_reason=TM.StringResult.FAIL_TIMEOUT if not ok else 0))
        return SimpleNamespace(accepted=True, get_result_async=lambda: result,
                               cancel_goal_async=lambda: calls.append('cancel_audio'))

    node.cli_measure = SimpleNamespace(send_goal_async=send)

    def action(client, goal, timeout, name, **kwargs):
        calls.append(name)
        if name == 'prepare_pluck':
            return SimpleNamespace(success=True, message='', preparation_id='one-use')
        assert goal.preparation_id == 'one-use'
        return SimpleNamespace(success=True, message='')

    node._run_action = action
    node._measure_attempt = TM.TaskManager._measure_attempt.__get__(node)
    node._measure_once = lambda n: TM.TaskManager._measure_once(node, n)
    return node, calls


def test_prepare_then_ready_then_exactly_one_play():
    node, calls = scenario()
    assert node._measure_once(6).ok
    assert calls == ['prepare_pluck', 'measure', 'ready', 'play_string']


@pytest.mark.parametrize('after_prepare', [False, True])
def test_resume_reaches_first_robot_action_after_pause(after_prepare):
    node, _ = scenario()
    action = node._run_action
    seen = []

    def record(client, goal, timeout, name, **kwargs):
        seen.append((name, goal.resumed))
        return action(client, goal, timeout, name, **kwargs)

    node._run_action = record
    if after_prepare:
        gates = iter([False, True])

        def gate():
            if next(gates):
                node._resumed_since_play.set()

        node._gate.side_effect = gate
    else:
        node._resumed_since_play.set()
    assert node._measure_once(6).ok
    assert seen == [('prepare_pluck', not after_prepare), ('play_string', after_prepare)]
    assert not node._resumed_since_play.is_set()


def test_slow_preparation_never_starts_microphone_early():
    node, calls = scenario()
    entered, release = threading.Event(), threading.Event()
    action = node._run_action
    results = []

    def slow(*args, **kwargs):
        if args[-1] == 'prepare_pluck':
            entered.set()
            assert release.wait(2)
        return action(*args, **kwargs)

    node._run_action = slow
    worker = threading.Thread(target=lambda: results.append(node._measure_once(6)))
    worker.start()
    try:
        assert entered.wait(1)
        assert calls == []  # 준비가 끝날 때까지 MeasurePitch 요청 자체를 보내지 않는다
    finally:
        release.set()
        worker.join(3)
    assert not worker.is_alive()
    assert results[0].ok
    assert calls == ['prepare_pluck', 'measure', 'ready', 'play_string']


@pytest.mark.parametrize('failure', ['reject', 'timeout', 'failed', 'missing_token'])
def test_prepare_failure_never_measures_or_plays(failure):
    node, calls = scenario()

    def fail(*args, **kwargs):
        if failure in ('reject', 'timeout'):
            raise TM.Aborted(failure)
        return SimpleNamespace(success=failure != 'failed', preparation_id='')

    node._run_action = fail
    with pytest.raises(TM.Aborted):
        node._measure_once(6)
    assert calls == []


def test_no_ready_means_no_play():
    node, calls = scenario(ready=False)
    assert not node._measure_once(6).ok
    assert calls == ['prepare_pluck', 'measure', 'cancel_audio']


def test_cancel_after_preparation_never_opens_microphone():
    node, calls = scenario()
    node._gate = Mock(side_effect=[None, TM.Cancelled()])
    with pytest.raises(TM.Cancelled):
        node._measure_once(6)
    assert calls == ['prepare_pluck']


def test_actual_measurement_failure_still_retries_with_new_preparation():
    node, calls = scenario(outcomes=(False, True))
    result, used = TM.TaskManager._measure_with_retry(node, 6, 3)
    assert result.ok and used == 1
    assert calls == ['prepare_pluck', 'measure', 'ready', 'play_string'] * 2


def test_after_adjustment_prepares_again_before_listening():
    node, calls = scenario(outcomes=(True, True))
    node._measure_once(6)
    calls.append('adjust_peg')
    node._measure_once(6)
    assert calls == (['prepare_pluck', 'measure', 'ready', 'play_string']
                     + ['adjust_peg', 'prepare_pluck', 'measure', 'ready', 'play_string'])


def test_ros_actions_keep_long_preparation_outside_audio_budget():
    """실제 ROS 왕복: 준비가 오디오 제한보다 길어도 첫 측정에 발현은 한 번이다."""
    import time

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter

    from tunio_server.fake_hardware import FakeHardware

    events = []

    class RecordedHardware(FakeHardware):
        def on_prepare_pluck(self, goal):
            events.append(('prepare_start', time.monotonic()))
            result = super().on_prepare_pluck(goal)
            events.append(('prepare_end', time.monotonic()))
            return result

        def on_measure(self, goal):
            events.append(('measure', time.monotonic()))
            return super().on_measure(goal)

        def on_play(self, goal):
            events.append(('play', time.monotonic()))
            return super().on_play(goal)

    rclpy.init()
    hardware = RecordedHardware()
    node = TM.TaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    worker = None
    try:
        hardware.set_parameters([
            Parameter('prepare_delay_sec', value=.3),
            Parameter('ready_delay_sec', value=.01),
            Parameter('measure_delay_sec', value=.01),
            Parameter('play_delay_sec', value=.01)])
        node.job_id = 'ros-prepare-test'
        node._registered_pegs = {6}
        node.active_settings = node.settings
        node.active_settings.onset_timeout_sec = .1
        node.active_settings.measure_timeout_sec = .1
        executor.add_node(hardware)
        executor.add_node(node)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()
        assert node._servers_not_ready() == []
        result, used = node._measure_with_retry(6, 3)
        assert result.ok and used == 0
        labels = [label for label, _ in events]
        assert labels == ['prepare_start', 'prepare_end', 'measure', 'play']
        assert events[1][1] - events[0][1] >= .3
    finally:
        executor.shutdown()
        if worker is not None:
            worker.join(3)
        node.destroy_node()
        hardware.destroy_node()
        rclpy.shutdown()
