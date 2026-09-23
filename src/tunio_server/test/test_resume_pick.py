"""서버는 재개 사실만 전달하며, 폭 판정이나 피크 확인 서비스를 사용하지 않는다."""
import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from tunio_interfaces.msg import TuningState
from tunio_interfaces.srv import CommandTuning
from tunio_server.task_manager import Aborted, TaskManager


def measurement_node(resumed=False, play_success=True):
    handle = MagicMock(accepted=True)
    measured = SimpleNamespace(success=True, job_id='test', string_number=6,
                               measurement_id=1, frequency_hz=82.4)
    node = SimpleNamespace(
        job_id='test', measurement_id=0, _registered_pegs={6, 5, 2, 1},
        active_settings=SimpleNamespace(measure_timeout_sec=1.0, onset_timeout_sec=1.0),
        _resumed_since_play=threading.Event(), _cancel=threading.Event(), _p=lambda name: 1.0,
        _set_phase=MagicMock(), _track=MagicMock(), _untrack=MagicMock(),
        _set_robot_notice=MagicMock(), _gate=MagicMock(),
        get_logger=lambda: MagicMock(),
        _wait=MagicMock(side_effect=[handle, SimpleNamespace(result=measured)]),
        _run_action=MagicMock(return_value=SimpleNamespace(
            success=play_success, preparation_id='token',
            message='' if play_success else '피크 없음. 홈 복귀 완료')),
        cli_prepare=object(), cli_play=object(), cli_measure=MagicMock(),
    )
    if not play_success:
        node._run_action.side_effect = Aborted('피크 없음. 홈 복귀 완료')
    if resumed:
        node._resumed_since_play.set()

    def send(goal, feedback_callback):
        feedback_callback(SimpleNamespace(feedback=SimpleNamespace(ready=True)))
        return object()

    node.cli_measure.send_goal_async.side_effect = send
    return node, handle


@pytest.mark.parametrize('resumed', [True, False])
def test_서버는_첫_발현에_재개사실만_전달한다(resumed):
    node, _ = measurement_node(resumed)
    assert TaskManager._measure_once(node, 6).ok
    goal = node._run_action.call_args_list[0].args[1]
    assert goal.resumed == resumed
    assert not node._resumed_since_play.is_set()
    assert not hasattr(node, 'cli_check_pick')


def test_피크없음_Action실패는_재측정하지_않고_작업중단한다():
    node, handle = measurement_node(resumed=True, play_success=False)
    with pytest.raises(Aborted, match='피크 없음'):
        TaskManager._measure_once(node, 6)
    handle.cancel_goal_async.assert_not_called()
    node._untrack.assert_not_called()
    node._run_action.assert_called_once()


def test_재개_버튼은_로봇확인서비스를_호출하지_않는다():
    node = SimpleNamespace(_running=lambda: True, _pause=threading.Event(),
                           _resumed_since_play=threading.Event(), phase=TuningState.PHASE_PAUSED)
    node._pause.set()
    response = TaskManager.on_resume(node, CommandTuning.Request(), CommandTuning.Response())
    assert response.accepted
    assert not node._pause.is_set()
    assert node._resumed_since_play.is_set()


def test_동작중_재개는_거부한다():
    node = SimpleNamespace(_running=lambda: True, _pause=threading.Event(),
                           phase=TuningState.PHASE_PLUCK)
    node._pause.set()
    response = TaskManager.on_resume(node, CommandTuning.Request(), CommandTuning.Response())
    assert not response.accepted


def test_성공_Result의_폭확인불가_경고는_기록하고_측정을_계속한다():
    node, handle = measurement_node(resumed=True)
    warning = '피크 폭 확인 불가: 2회 읽기 실패'
    node._run_action.return_value.message = warning
    logger = MagicMock()
    node.get_logger = lambda: logger
    result = TaskManager._measure_once(node, 6)
    assert result.ok
    logger.warning.assert_called_once()
    assert warning in logger.warning.call_args.args[0]
    handle.cancel_goal_async.assert_not_called()
    assert node._run_action.call_count == 2  # 준비와 발현 각 한 번


def test_경고_Feedback을_받아도_동작을_중단하거나_중복경고하지_않는다():
    node, handle = measurement_node(resumed=True)
    logger = MagicMock()
    node.get_logger = lambda: logger
    warning = '피크 폭 확인 불가'

    def run(*args, **kwargs):
        kwargs['feedback_cb'](SimpleNamespace(feedback=SimpleNamespace(warning=warning)))
        return SimpleNamespace(success=True, message=warning, preparation_id='token')

    node._run_action.side_effect = run
    assert TaskManager._measure_once(node, 6).ok
    logger.warning.assert_called_once()
    handle.cancel_goal_async.assert_not_called()


def test_로봇_알람이_관제_상태로_전달된다():
    from tunio_server.web_node import _state_to_dict
    node = SimpleNamespace(_publish_state=MagicMock())
    message = '피크 재집기 2회 모두 빈손. 홈 복귀 후 종료'
    TaskManager._set_robot_notice(node, message)
    assert node.robot_notice == message
    node._publish_state.assert_called_once()
    msg = TuningState(robot_notice=message)
    assert _state_to_dict(msg)['robot_notice'] == message


def test_재집기가_길어져도_같은_Action결과를_기다린다():
    future = object()
    handle = MagicMock(accepted=True)
    handle.get_result_async.return_value = future
    result = SimpleNamespace(success=True, message='')
    node = SimpleNamespace(
        _wait=MagicMock(side_effect=[handle, None, SimpleNamespace(result=result, status=4)]),
        _p=lambda name: 120.0, _track=MagicMock(), _untrack=MagicMock(),
        get_logger=lambda: MagicMock(),
    )
    client = MagicMock()
    assert TaskManager._run_action(node, client, object(), 30.0, 'play_string') is result
    assert node._wait.call_args_list[1].args[0] is future
    assert node._wait.call_args_list[2].args[0] is future
    client.send_goal_async.assert_called_once()
