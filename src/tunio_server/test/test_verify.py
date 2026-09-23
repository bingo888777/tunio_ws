"""1차 실패 사유는 최종 검증에서 덮어쓰지 않는다."""
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_server import task_manager as TM


@pytest.mark.parametrize('reason', [
    TM.StringResult.FAIL_TIMEOUT, TM.StringResult.FAIL_ADJUST_EXHAUSTED,
    TM.StringResult.FAIL_NO_PITCH])
@pytest.mark.parametrize('has_tuned', [True, False])
def test_verify_only_success_preserves_failed_results(reason, has_tuned):
    failed = TM.StringResult(
        string_number=2, target_hz=246.94, result=TM.StringResult.RESULT_FAILED,
        failure_reason=reason, has_tuned_hz=has_tuned, tuned_hz=230.0,
        adjust_count=2, final_error_cents=-123.0)
    success = TM.StringResult(
        string_number=1, target_hz=329.63, result=TM.StringResult.RESULT_SUCCESS,
        has_tuned_hz=True, tuned_hz=329.63)
    before = deepcopy(failed)
    measure = Mock(return_value=(TM.Measurement(True, 329.63), 0))
    node = SimpleNamespace(
        results={2: failed, 1: success}, _gate=Mock(), _set_phase=Mock(),
        _publish_state=Mock(), get_logger=lambda: Mock(), _measure_with_retry=measure,
        active_settings=SimpleNamespace(max_measure_retry=3, tolerance_cents=10))
    TM.TaskManager._verify_pass(node, numbers=[2, 1])
    measure.assert_called_once_with(1, 3)
    assert failed == before
    assert success.has_verified_hz
    assert success.result == TM.StringResult.RESULT_SUCCESS


def test_success_that_detunes_is_still_failed_by_verify():
    result = TM.StringResult(
        target_hz=246.94, result=TM.StringResult.RESULT_SUCCESS, has_tuned_hz=True)
    node = SimpleNamespace(
        results={2: result}, _gate=Mock(), _set_phase=Mock(), _publish_state=Mock(),
        get_logger=lambda: Mock(),
        _measure_with_retry=lambda *args: (TM.Measurement(True, 240.84), 0),
        active_settings=SimpleNamespace(max_measure_retry=3, tolerance_cents=10))
    TM.TaskManager._verify_pass(node, numbers=[2])
    assert result.result == TM.StringResult.RESULT_FAILED
    assert result.failure_reason == TM.StringResult.FAIL_DETUNED_ON_VERIFY
