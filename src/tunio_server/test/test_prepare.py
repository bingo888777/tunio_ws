"""시작 전 등록은 측정과 분리하고, 실패하면 발현에 진입하지 않는다."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_server import task_manager as TM


@pytest.fixture(autouse=True)
def four_string_scenario(monkeypatch):
    # 현장 시험용 STRING_ORDER 변경과 별개로 네 줄 등록 시나리오를 검증한다.
    monkeypatch.setattr(TM.L, 'STRING_ORDER', [6, 5, 2, 1])


@pytest.mark.parametrize('registration_ok', [True, False])
@pytest.mark.parametrize('reregister', [True])
def test_start_waits_for_all_peg_angles_before_tuning(registration_ok, reregister):
    import threading

    calls = []
    phases = []
    registering = threading.Event()
    release = threading.Event()

    def action(client, goal, timeout, name):
        calls.append((name, goal.string_number))
        assert goal.force is reregister
        if goal.string_number == 1:
            registering.set()
            assert release.wait(3)
        return SimpleNamespace(success=registration_ok or goal.string_number != 1,
                               message='등록 실패')

    n = SimpleNamespace(
        _lock=threading.Lock(), _starting=False, _worker=None,
        _pause=threading.Event(), _cancel=threading.Event(),
        settings=TM.TuningSettings(), cli_register=object(),
        _registered_pegs={6, 5, 2, 1},  # 이전 작업의 완료 상태도 시작할 때 지워야 한다
        _servers_not_ready=lambda **kw: [], _reset_results=lambda: None,
        get_logger=lambda: Mock(), _gate=lambda: None,
        _p=lambda key: {'register_timeout_sec': 180}.get(key, 0),
        _set_phase=lambda phase, no: phases.append((phase, no)),
        _run_action=action, _tune_string=lambda no: calls.append(('tune', no)),
        _verify_pass=lambda: None, _return_home=lambda: None, _save=lambda: None,
        _finish_error=lambda: calls.append(('error', 0)))
    for name in ('_running', '_do_start', '_run_job', '_run_reregister', '_register_pegs'):
        setattr(n, name, getattr(TM.TaskManager, name).__get__(n))

    response = TM.TaskManager.on_start(n, None, SimpleNamespace(), reregister=reregister)
    try:
        assert response.accepted
        assert phases[0] == (TM.TuningState.PHASE_REGISTER_PEG, 6)
        assert registering.wait(2)
        assert calls == [('register_peg', no) for no in (6, 5, 2, 1)]
        assert n._registered_pegs == set()
    finally:
        release.set()
        n._worker.join(3)
    assert not n._worker.is_alive()
    expected = ([('tune', no) for no in (6, 5, 2, 1)] if registration_ok
                else [('error', 0)])
    assert calls[4:] == ([] if reregister else expected)
    assert n._registered_pegs == ({6, 5, 2, 1} if registration_ok else set())
    assert n._registration_ready is registration_ok


@pytest.mark.parametrize('failure', [None, 5])
def test_start_never_registers_even_if_registration_would_fail(failure):
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
    assert calls == [('tune', no) for no in (6, 5, 2, 1)]


@pytest.mark.parametrize('ended', [True, False])
def test_robot_timeout_waits_for_terminal_result_before_continuing(ended):
    handle = Mock(accepted=True)
    late = (SimpleNamespace(status=4, result=SimpleNamespace(success=True, message=''))
            if ended else None)
    waits = Mock(side_effect=[handle, None, late])
    n = SimpleNamespace(
        _wait=waits, _p=lambda key: 120.0, _cancel=threading.Event(),
        _track=Mock(), _untrack=Mock(),
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
                        _cancel=threading.Event(),
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

    n = SimpleNamespace(_p=lambda key: 2, _cancel=threading.Event(),
                        _track=Mock(), _untrack=Mock(),
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
        _cancel=threading.Event(),
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
    assert calls == [*[('tune', no) for no in (6, 5, 2, 1)],
                     'verify', 'home', 'save' if home_ok else 'error']
    assert (TuningState.PHASE_DONE in phases) is home_ok


@pytest.mark.parametrize('failure', [None, 'register', 'home', 'cancel'])
def test_reregister_forces_each_peg_and_never_tunes_or_saves(failure):
    calls = []
    phases = []

    def action(client, goal, timeout, name):
        calls.append((goal.string_number, goal.force))
        if failure == 'cancel':
            raise TM.Cancelled()
        return SimpleNamespace(success=failure != 'register', message='등록 실패')

    def home():
        calls.append('home')
        if failure == 'home':
            raise TM.Aborted('복귀 실패')

    n = SimpleNamespace(
        job_id='registration', cli_register=object(), get_logger=lambda: Mock(),
        _gate=lambda: None, _p=lambda key: 180, _run_action=action,
        _set_phase=lambda phase, no: phases.append(phase), _return_home=home,
        _tune_string=Mock(), _save=Mock())
    n._register_pegs = lambda **kw: TM.TaskManager._register_pegs(n, **kw)
    TM.TaskManager._run_reregister(n)
    assert calls == ([(6, True)] if failure in ('register', 'cancel') else
                     [(no, True) for no in (6, 5, 2, 1)] + ['home'])
    expected = (TM.TuningState.PHASE_CANCELED if failure == 'cancel' else
                TM.TuningState.PHASE_ERROR if failure else TM.TuningState.PHASE_IDLE)
    assert phases[-1] == expected
    n._tune_string.assert_not_called()
    n._save.assert_not_called()
    if failure is None:
        assert n.job_id == ''


@pytest.mark.parametrize('busy,starting', [(True, False), (False, True), (False, False)])
def test_reregister_uses_start_exclusion(busy, starting):
    import threading

    n = SimpleNamespace(_lock=threading.Lock(), _running=lambda: busy,
                        _starting=starting, _do_start=Mock(return_value='accepted'))
    n.on_start = TM.TaskManager.on_start.__get__(n)
    response = TM.TaskManager.on_reregister_pegs(n, None, SimpleNamespace())
    if busy or starting:
        assert not response.accepted
        n._do_start.assert_not_called()
    else:
        assert response == 'accepted'
        assert n._do_start.call_args.kwargs == {'reregister': True}
        assert not n._starting


def test_reregister_readiness_does_not_require_audio_or_pluck():
    n = SimpleNamespace(_p=lambda key: 0)
    for name in ('home', 'register', 'prepare', 'measure', 'play', 'adjust'):
        setattr(n, 'cli_' + name, Mock())
    n.cli_home.wait_for_server.return_value = True
    n.cli_register.wait_for_server.return_value = False
    assert TM.TaskManager._servers_not_ready(n, registration_only=True) == ['robot/register_peg']
    for name in ('prepare', 'measure', 'play', 'adjust'):
        getattr(n, 'cli_' + name).wait_for_server.assert_not_called()


def test_start_does_not_require_registration_server():
    n = SimpleNamespace(_p=lambda key: 0)
    for name in ('home', 'register', 'prepare', 'measure', 'play', 'adjust'):
        setattr(n, 'cli_' + name, Mock())
    n.cli_register.wait_for_server.return_value = False
    assert TM.TaskManager._servers_not_ready(n) == []
    n.cli_register.wait_for_server.assert_not_called()


def test_unregistered_start_rejected_before_any_action():
    n = SimpleNamespace(_registration_ready=False, _servers_not_ready=Mock())
    response = TM.TaskManager._do_start(n, SimpleNamespace())
    assert not response.accepted
    assert '재등록을 먼저' in response.message
    n._servers_not_ready.assert_not_called()


def test_missing_server_does_not_consume_registration():
    n = SimpleNamespace(_registration_ready=True,
                        _servers_not_ready=Mock(return_value=['audio/measure_pitch']))
    response = TM.TaskManager._do_start(n, SimpleNamespace())
    assert not response.accepted
    assert n._registration_ready


@pytest.mark.parametrize('status,success,expected', [
    (5, False, TM.Cancelled), (4, True, TM.Cancelled), (6, False, TM.Aborted)])
def test_cancel_requires_confirmed_terminal_result(status, success, expected):
    event = threading.Event()
    event.set()
    handle = Mock(accepted=True)
    result = SimpleNamespace(status=status, result=SimpleNamespace(success=success, message='응답'))
    n = SimpleNamespace(_cancel=event, _wait=Mock(side_effect=[handle, result]),
                        _p=lambda key: 1, _track=Mock(), _untrack=Mock(),
                        get_logger=lambda: Mock())
    with pytest.raises(expected):
        TM.TaskManager._run_action(n, Mock(), object(), 1, 'play_string')
    handle.cancel_goal_async.assert_called_once()


def test_user_cancel_with_unknown_robot_end_stays_error():
    event = threading.Event()
    event.set()
    handle = Mock(accepted=True)
    n = SimpleNamespace(_cancel=event, _wait=Mock(side_effect=[handle, None, None]),
                        _p=lambda key: 1, _track=Mock(), _untrack=Mock(),
                        get_logger=lambda: Mock())
    with pytest.raises(TM.Aborted, match='동작 종료 불명'):
        TM.TaskManager._run_action(n, Mock(), object(), 1, 'play_string')
