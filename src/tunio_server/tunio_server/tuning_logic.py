"""
조율 판단 로직 (ROS 비의존 순수 함수 모음).
task_manager가 이 함수들을 호출한다. 단위 테스트가 쉽도록 분리.
"""
import math

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
    오차 -> (raise_pitch, angle_deg).
    오차가 클수록 많이, 작을수록 조금 회전. 계수는 테스트 후 확정(임시).
    """
    raise_pitch = err_cents < 0            # 음이 낮으면 높여야 함
    k = 0.15                               # deg per cent (임시값, 실측 보정)
    angle = min(abs(err_cents) * k, 20.0)  # 안전 상한
    return raise_pitch, angle


def overall_result(statuses: list) -> str:
    """줄별 status(1=성공,2/3=실패) -> 전체 결과"""
    ok = sum(1 for s in statuses if s == 1)
    if ok == len(statuses):
        return "성공"
    if ok == 0:
        return "실패"
    return "부분성공"
