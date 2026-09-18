"""
조율 판단 로직 (rclpy 비의존 순수 함수 모음).
task_manager가 이 함수들을 호출한다. 단위 테스트가 쉽도록 분리.

열거값은 정의를 중복하지 않고 tunio_interfaces 메시지에서 직접 가져온다.
메시지 모듈 import만 하므로 rclpy 초기화나 ROS 그래프 없이도 동작한다.
"""
import math

from tunio_interfaces.msg import StringResult, TuningState

# 목표 주파수 (A4=440, 12평균율). 기획서 2.2
TARGETS = {
    6: ("E2", 82.4069),
    5: ("A2", 110.0000),
    2: ("B3", 246.9417),
    1: ("E4", 329.6276),
}
STRING_ORDER = [6, 5, 2, 1]


def error_cents(measured_hz: float, target_hz: float) -> float:
    """센트 오차 = 1200 * log2(측정/목표)"""
    return 1200.0 * math.log2(measured_hz / target_hz)


def is_in_tune(measured_hz: float, target_hz: float, tol_cents: float) -> bool:
    return abs(error_cents(measured_hz, target_hz)) <= tol_cents


def peg_angle(err_cents: float) -> tuple:
    """
    오차 -> (direction, angle_deg).
    direction 은 AdjustPeg.Goal.direction 규약을 따른다: +1 조임(음 높이기) / -1 풀림.
    angle_deg 는 항상 양수 크기. 오차가 클수록 많이, 작을수록 조금 회전.
    계수는 테스트 후 확정(임시).
    """
    direction = 1 if err_cents < 0 else -1   # 음이 낮으면 높여야 함
    k = 0.15                                 # deg per cent (임시값, 실측 보정)
    angle = min(abs(err_cents) * k, 20.0)    # 안전 상한
    return direction, angle


def overall_result(results: list) -> int:
    """
    줄별 StringResult.RESULT_* -> 전체 TuningState.OVERALL_*.

    아직 처리 전인 줄이 하나라도 있으면 판정하지 않고 OVERALL_PENDING.
    """
    if not results or any(r == StringResult.RESULT_PENDING for r in results):
        return TuningState.OVERALL_PENDING
    ok = sum(1 for r in results if r == StringResult.RESULT_SUCCESS)
    if ok == len(results):
        return TuningState.OVERALL_SUCCESS
    if ok == 0:
        return TuningState.OVERALL_FAILED
    return TuningState.OVERALL_PARTIAL
