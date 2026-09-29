"""일시정지 = 로봇을 즉시 세우고 홈, 재개 = 그 줄을 처음부터 (2026-09-29)."""
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
import pytest

from tunio_server import task_manager as TM


@pytest.fixture(autouse=True)
def four_strings(monkeypatch):
    monkeypatch.setattr(TM.L, 'STRING_ORDER', [6, 5, 2, 1])


def _node(**extra):
    n = SimpleNamespace(
        _lock=threading.Lock(), _pause=threading.Event(), _cancel=threading.Event(),
        _active_goals=[], get_logger=lambda: Mock(),
        _p=lambda key: {'goal_response_sec': 1, 'register_timeout_sec': 5}.get(key, 1),
        _set_phase=lambda phase, no=0: None, _wait=lambda fut, t, what: fut,
        _registered_pegs=set())
    for name in ('_track', '_untrack', '_run_action', '_register_pegs', '_hold_after_pause',
                 '_run_job', '_request_goal_cancel', 'on_pause'):
        setattr(n, name, getattr(TM.TaskManager, name).__get__(n))
    for k, v in extra.items():
        setattr(n, k, v)
    return n


def _client(status, success=True):
    """send_goal_async → handle(.accepted) → get_result_async → wrapped(status, result)."""
    wrapped = SimpleNamespace(status=status, result=SimpleNamespace(success=success, message=''))
    handle = SimpleNamespace(accepted=True, cancel_goal_async=Mock(),
                             get_result_async=lambda: wrapped)
    return SimpleNamespace(send_goal_async=lambda goal, feedback_callback=None: handle), handle


def test_run_action_raises_paused_when_goal_canceled_under_pause():
    n = _node()
    n._pause.set()
    client, handle = _client(GoalStatus.STATUS_CANCELED, success=False)
    with pytest.raises(TM.Paused) as info:
        n._run_action(client, SimpleNamespace(), 1.0, 'prepare_pluck')
    assert info.value.what == 'prepare_pluck'
    handle.cancel_goal_async.assert_called()          # 수락 직후에도 취소를 전달한다
    assert n._active_goals == []


def test_run_action_still_raises_cancelled_when_user_cancelled():
    n = _node()
    n._cancel.set()
    n._pause.set()                                    # 둘 다 켜져도 취소가 우선
    client, _ = _client(GoalStatus.STATUS_CANCELED, success=False)
    with pytest.raises(TM.Cancelled):
        n._run_action(client, SimpleNamespace(), 1.0, 'adjust_peg')


def test_on_pause_requests_cancel_of_active_goals():
    cancelled = []
    n = _node(_running=lambda: True,
              _active_goals=[SimpleNamespace(cancel_goal_async=lambda: cancelled.append(1))])
    res = n.on_pause(None, SimpleNamespace())
    assert res.accepted and n._pause.is_set()
    assert cancelled == [1]
    assert '처음부터' in res.message and '그 자리' in res.message


@pytest.mark.parametrize('what, expect_reregister', [
    ('adjust_peg', True), ('register_peg', True),
    ('prepare_pluck', False), ('play_string', False), ('measure_pitch', False)])
def test_job_redoes_the_string_after_pause(what, expect_reregister):
    calls = []
    gates = []
    first = {6: True}

    def tune(no):
        calls.append(('tune', no))
        if first.pop(no, False):
            raise TM.Paused(what)
    n = _node(_tune_string=tune, _verify_pass=lambda: calls.append(('verify',)),
              _wipe=lambda: calls.append(('wipe',)),
              _return_home=lambda: calls.append(('home',)), _save=lambda: calls.append(('save',)),
              _register_pegs=lambda numbers, force=False: calls.append(
                  ('register', tuple(numbers), force)),
              _gate=lambda: gates.append(1), _sleep_gated=lambda s: None,
              _finish_cancelled=lambda: calls.append(('cancelled',)),
              _finish_error=lambda: calls.append(('error',)), job_id='j')
    n._run_job()
    expected = [('tune', 6)]
    if expect_reregister:
        expected.append(('register', (6,), True))
    expected += [('tune', 6), ('tune', 5), ('tune', 2), ('tune', 1),
                 ('verify',), ('wipe',), ('home',), ('save',)]
    assert calls == expected
    assert gates                                       # PAUSED 대기(_gate)를 거쳤다


def test_register_pegs_resumes_from_interrupted_string_with_force():
    goals = []
    once = {5: True}

    def action(client, goal, timeout, name):
        goals.append((goal.string_number, goal.force))
        if once.pop(goal.string_number, False):
            raise TM.Paused(name)
        return SimpleNamespace(success=True, message='')
    n = _node(_run_action=action, _gate=lambda: None, cli_register=object())
    n._register_pegs(force=False)
    assert goals == [(6, False), (5, False), (5, True), (2, False), (1, False)]
    assert n._registered_pegs == {6, 5, 2, 1}


def test_partial_reregister_keeps_other_strings_registered():
    n = _node(_run_action=lambda *a: SimpleNamespace(success=True, message=''),
              _gate=lambda: None, cli_register=object(), _registered_pegs={6, 5, 2, 1})
    n._register_pegs([5], force=True)
    assert n._registered_pegs == {6, 5, 2, 1}


def _cancel_node(**extra):
    calls = []
    n = _node(_active_goals=[], results={6: SimpleNamespace(result=0, failure_reason=0)},
              _return_home=lambda: calls.append('home'), error_message='',
              _set_phase=lambda phase, no=0: calls.append(('phase', phase)))
    n._finish_cancelled = TM.TaskManager._finish_cancelled.__get__(n)
    n._home_after_stop = TM.TaskManager._home_after_stop.__get__(n)
    for k, v in extra.items():
        setattr(n, k, v)
    return n, calls


def test_finish_cancelled_sends_robot_home_then_ends_canceled():
    n, calls = _cancel_node()
    n._cancel.set()
    n._pause.set()
    n._finish_cancelled()
    assert calls == ['home', ('phase', TM.TuningState.PHASE_CANCELED)]
    assert not n._cancel.is_set() and not n._pause.is_set()   # 복귀 goal 은 취소 대상이 아니다
    assert n.results[6].failure_reason == TM.StringResult.FAIL_CANCELED
    assert n.error_message == ''


def test_finish_cancelled_home_failure_is_logged_and_still_canceled():
    def fail():
        raise TM.Aborted('피크 반환·대기 복귀 실패')
    n, calls = _cancel_node(_return_home=fail)
    n._cancel.set()
    n._finish_cancelled()
    assert calls == [('phase', TM.TuningState.PHASE_CANCELED)]
    assert '복귀 실패' in n.error_message
