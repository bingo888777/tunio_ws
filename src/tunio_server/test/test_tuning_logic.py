"""tuning_logic 순수 함수 테스트 (로봇/마이크 없이 실행 가능)."""
import pytest

from tunio_interfaces.msg import StringResult, TuningState

from tunio_server import tuning_logic as L
from tunio_server.peg_calibration import StringCalibration

S = StringResult.RESULT_SUCCESS
F = StringResult.RESULT_FAILED
P = StringResult.RESULT_PENDING

# 테스트용 가짜 캘리브레이션. 실측값이 아니라 계산이 맞는지만 보기 위한 숫자다.
# 실제 값은 tools/peg_cal.py 로 재서 config/peg_calibration.yaml 에 넣는다.
CAL = StringCalibration(
    cents_per_deg=5.0,          # 1도당 5센트 -> 10센트는 2도
    min_effective_deg=0.5,
    max_angle_deg=20.0,
    backlash_release_cents=25.0,
)


def test_error_cents_zero():
    assert abs(L.error_cents(440.0, 440.0)) < 1e-9


def test_in_tune():
    assert L.is_in_tune(329.6, 329.6276, 5.0)
    assert not L.is_in_tune(340.0, 329.6276, 5.0)


def test_peg_direction():
    direction, _ = L.peg_angle(-10.0, CAL)   # 음이 낮음 -> 높여야(조임)
    assert direction == 1
    direction, _ = L.peg_angle(10.0, CAL)    # 음이 높음 -> 풀어야
    assert direction == -1


def test_peg_angle_is_positive_magnitude():
    for err in (-30.0, -1.0, 1.0, 30.0):
        _, angle = L.peg_angle(err, CAL)
        assert angle > 0


def test_peg_angle_uses_calibration_only():
    """회전량은 cents_per_deg 로만 나온다. 코드에 계수가 남아 있으면 이 값이 안 맞는다."""
    _, angle = L.peg_angle(-10.0, CAL)
    assert angle == pytest.approx(2.0)


def test_peg_angle_respects_min_effective():
    _, angle = L.peg_angle(-0.5, CAL)        # 0.1도가 나오지만 페그가 안 움직인다
    assert angle == pytest.approx(CAL.min_effective_deg)


def test_peg_angle_respects_max():
    _, angle = L.peg_angle(-1000.0, CAL)
    assert angle == pytest.approx(CAL.max_angle_deg)


def test_min_step_cents():
    assert L.min_step_cents(CAL) == pytest.approx(2.5)


def test_adjust_plan_flat_string_tightens_once():
    """음이 낮으면 이미 조이는 방향이라 goal 1개다."""
    plan = L.adjust_plan(-10.0, CAL)
    assert len(plan) == 1
    assert plan[0][0] == 1


def test_adjust_plan_sharp_string_releases_then_tightens():
    """음이 높으면 목표 아래로 풀었다가 조여 올린다 (CLAUDE.md 3.2)."""
    plan = L.adjust_plan(10.0, CAL)
    assert len(plan) == 2
    assert plan[0][0] == -1          # 먼저 푼다
    assert plan[-1][0] == 1          # 마지막은 반드시 조이는 방향
    # 푸는 양 = 오차 + 백래시 여유 = 35센트 = 7도
    assert plan[0][1] == pytest.approx(7.0)
    # 조여 올리는 양 = 백래시 여유 25센트 = 5도
    assert plan[1][1] == pytest.approx(5.0)


def test_adjust_plan_clamped_release_skips_tighten():
    """푸는 각도가 상한에 걸리면 그 회차는 풀기만 하고 끝낸다."""
    plan = L.adjust_plan(500.0, CAL)
    assert len(plan) == 1
    assert plan[0][0] == -1
    assert plan[0][1] == pytest.approx(CAL.max_angle_deg)


def test_overall():
    assert L.overall_result([S, S, S, S]) == TuningState.OVERALL_SUCCESS
    assert L.overall_result([S, F, S, S]) == TuningState.OVERALL_PARTIAL
    assert L.overall_result([F, F, F, F]) == TuningState.OVERALL_FAILED


def test_overall_pending():
    assert L.overall_result([]) == TuningState.OVERALL_PENDING
    assert L.overall_result([S, P, S, S]) == TuningState.OVERALL_PENDING
