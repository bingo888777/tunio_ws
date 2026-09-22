"""시작 전 등록은 측정과 분리하고, 실패하면 발현에 진입하지 않는다."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_server import task_manager as TM


@pytest.mark.parametrize('failure', [None, 5])
def test_registration_precedes_tuning_and_failure_stops_job(failure):
    calls = []

    def action(client, goal, timeout, name):
        calls.append(('register', goal.string_number))
        return SimpleNamespace(success=goal.string_number != failure, message='등록 실패')

    n = SimpleNamespace(
        job_id='test', cli_register=object(),
        get_logger=lambda: Mock(), _gate=lambda: None,
        _set_phase=lambda *args: None,
        _p=lambda key: 180.0 if key == 'register_timeout_sec' else 0,
        _run_action=action, _tune_string=lambda no: calls.append(('tune', no)),
        _verify_pass=lambda: None, _strum=lambda: None, _return_home=lambda: None,
        _save=lambda: None, _finish_error=lambda: calls.append(('error', 0)))
    n._register_pegs = lambda: TM.TaskManager._register_pegs(n)
    TM.TaskManager._run_job(n)
    if failure:
        assert calls == [('register', 6), ('register', 5), ('error', 0)]
    else:
        assert calls == [(kind, no) for kind in ('register', 'tune') for no in (6, 5, 2, 1)]


@pytest.mark.parametrize('ended', [True, False])
def test_robot_timeout_waits_for_terminal_result_before_continuing(ended):
    handle = Mock(accepted=True)
    late = (SimpleNamespace(status=4, result=SimpleNamespace(success=True, message=''))
            if ended else None)
    waits = Mock(side_effect=[handle, None, late])
    n = SimpleNamespace(
        _wait=waits, _p=lambda key: 120.0, _track=Mock(), _untrack=Mock(),
        get_logger=lambda: Mock())
    client = Mock()
    if ended:
        assert TM.TaskManager._run_action(
            n, client, object(), 120, 'adjust_peg') is late.result
    else:
        with pytest.raises(TM.Aborted, match='동작 종료 불명'):
            TM.TaskManager._run_action(n, client, object(), 120, 'adjust_peg')
    assert waits.call_count == 3
    n._untrack.assert_called_once_with(handle)
    handle.get_result_async.assert_called_once()


def test_unknown_goal_acceptance_does_not_continue():
    n = SimpleNamespace(_wait=lambda *a: None, _p=lambda key: 5.0)
    with pytest.raises(TM.Aborted, match='수락 여부 불명'):
        TM.TaskManager._run_action(n, Mock(), object(), 120, 'adjust_peg')


@pytest.mark.parametrize('late', [False, True])
@pytest.mark.parametrize('status,success', [(4, False), (5, True), (6, False)])
def test_failed_or_canceled_robot_result_aborts(status, success, late):
    handle = Mock(accepted=True)
    result = SimpleNamespace(status=status, result=SimpleNamespace(success=success, message='실패'))
    responses = [handle, None, result] if late else [handle, result]
    n = SimpleNamespace(_wait=Mock(side_effect=responses), _p=lambda key: 120,
                        _track=Mock(), _untrack=Mock(), get_logger=lambda: Mock())
    with pytest.raises(TM.Aborted, match='후속 동작 중단'):
        TM.TaskManager._run_action(n, Mock(), object(), 120, 'adjust_peg')
    n._untrack.assert_called_once_with(handle)


def test_rejected_robot_goal_aborts():
    n = SimpleNamespace(_wait=lambda *a: SimpleNamespace(accepted=False), _p=lambda key: 5)
    with pytest.raises(TM.Aborted, match='goal 거부'):
        TM.TaskManager._run_action(n, Mock(), object(), 120, 'play_string')


def test_next_robot_request_waits_for_success_result():
    from concurrent.futures import Future
    import threading

    arrived = threading.Event()
    result = Future()
    sent = []
    errors = []

    def get_result():
        arrived.set()
        return result

    def send(goal, **kwargs):
        sent.append(goal)
        response = Future()
        response.set_result(SimpleNamespace(accepted=True, get_result_async=get_result))
        return response

    n = SimpleNamespace(_p=lambda key: 2, _track=Mock(), _untrack=Mock(),
                        get_logger=lambda: Mock())
    n._wait = lambda *args: TM.TaskManager._wait(n, *args)
    client = SimpleNamespace(send_goal_async=send)

    def run():
        try:
            for goal in ('adjust_2', 'play_2'):
                TM.TaskManager._run_action(n, client, goal, 2, goal)
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=run)
    worker.start()
    try:
        assert arrived.wait(1)
        assert sent == ['adjust_2']
    finally:
        result.set_result(SimpleNamespace(
            status=4, result=SimpleNamespace(success=True, message='')))
        worker.join(3)
    assert not worker.is_alive()
    assert not errors
    assert sent == ['adjust_2', 'play_2']


def test_adjust_success_remeasures_same_string():
    calls = []
    measurements = iter([TM.Measurement(True, 230.0), TM.Measurement(True, 246.94)])

    def measure(no, budget):
        calls.append(('measure', no))
        return next(measurements), 0

    def adjust(no, error):
        calls.append(('adjust', no))
        return True
    n = SimpleNamespace(
        results={2: SimpleNamespace(target_hz=246.94)},
        active_settings=SimpleNamespace(
            max_measure_retry=3, max_adjust_count=5, tolerance_cents=10),
        _gate=lambda: None, _publish_state=lambda: None, get_logger=lambda: Mock(),
        _measure_with_retry=measure, _adjust_peg=adjust)
    TM.TaskManager._tune_string(n, 2)
    assert calls == [('measure', 2), ('adjust', 2), ('measure', 2)]


@pytest.mark.parametrize('home_ok', [True, False])
def test_job_skips_unimplemented_strum_and_requires_home_success_before_save(home_ok):
    from tunio_interfaces.msg import TuningState

    calls = []
    phases = []
    handle = Mock(accepted=True)
    wrapped = SimpleNamespace(
        status=4 if home_ok else 6,
        result=SimpleNamespace(success=home_ok, message='' if home_ok else '복귀 실패'))
    n = SimpleNamespace(
        job_id='test', get_logger=lambda: Mock(), _gate=lambda: None, _p=lambda key: 0,
        _register_pegs=lambda: calls.append('register'),
        _tune_string=lambda no: calls.append(('tune', no)),
        _verify_pass=lambda: calls.append('verify'),
        _strum=Mock(side_effect=NotImplementedError('화음 미구현')),
        _set_phase=lambda phase, no: phases.append(phase),
        cli_home=Mock(), _wait=Mock(side_effect=[handle, wrapped]),
        _track=Mock(), _untrack=Mock(),
        _save=lambda: calls.append('save'), _finish_error=lambda: calls.append('error'))
    n._run_action = lambda *a, **k: TM.TaskManager._run_action(n, *a, **k)

    def home():
        calls.append('home')
        TM.TaskManager._return_home(n)
    n._return_home = home
    TM.TaskManager._run_job(n)
    n._strum.assert_not_called()
    n.cli_home.send_goal_async.assert_called_once()
    assert calls == ['register', *[('tune', no) for no in (6, 5, 2, 1)],
                     'verify', 'home', 'save' if home_ok else 'error']
    assert (TuningState.PHASE_DONE in phases) is home_ok
