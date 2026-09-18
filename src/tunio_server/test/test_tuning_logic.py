"""tuning_logic 순수 함수 테스트 (로봇/마이크 없이 실행 가능)."""
from tunio_interfaces.msg import StringResult, TuningState

from tunio_server import tuning_logic as L

S = StringResult.RESULT_SUCCESS
F = StringResult.RESULT_FAILED
P = StringResult.RESULT_PENDING


def test_error_cents_zero():
    assert abs(L.error_cents(440.0, 440.0)) < 1e-9


def test_in_tune():
    assert L.is_in_tune(329.6, 329.6276, 5.0)
    assert not L.is_in_tune(340.0, 329.6276, 5.0)


def test_peg_direction():
    direction, _ = L.peg_angle(-10.0)   # 음이 낮음 -> 높여야(조임)
    assert direction == 1
    direction, _ = L.peg_angle(10.0)    # 음이 높음 -> 풀어야
    assert direction == -1


def test_peg_angle_is_positive_magnitude():
    for err in (-30.0, -1.0, 1.0, 30.0):
        _, angle = L.peg_angle(err)
        assert angle > 0


def test_overall():
    assert L.overall_result([S, S, S, S]) == TuningState.OVERALL_SUCCESS
    assert L.overall_result([S, F, S, S]) == TuningState.OVERALL_PARTIAL
    assert L.overall_result([F, F, F, F]) == TuningState.OVERALL_FAILED


def test_overall_pending():
    assert L.overall_result([]) == TuningState.OVERALL_PENDING
    assert L.overall_result([S, P, S, S]) == TuningState.OVERALL_PENDING
