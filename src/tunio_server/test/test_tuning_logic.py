"""tuning_logic 순수 함수 테스트 (로봇/마이크 없이 실행 가능)."""
from tunio_server import tuning_logic as L


def test_error_cents_zero():
    assert abs(L.error_cents(440.0, 440.0)) < 1e-9


def test_in_tune():
    assert L.is_in_tune(329.6, 329.6276, 5.0)
    assert not L.is_in_tune(340.0, 329.6276, 5.0)


def test_peg_direction():
    raise_pitch, _ = L.peg_angle(-10.0)   # 음이 낮음 -> 높여야
    assert raise_pitch is True


def test_overall():
    assert L.overall_result([1, 1, 1, 1]) == "성공"
    assert L.overall_result([1, 2, 1, 1]) == "부분성공"
    assert L.overall_result([2, 2, 2, 2]) == "실패"
