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
    6: ('E2', 82.4069),
    5: ('A2', 110.0000),
    2: ('B3', 246.9417),
    1: ('E4', 329.6276),
}
STRING_ORDER = [6, 5, 2, 1]


def error_cents(measured_hz: float, target_hz: float) -> float:
    """센트 오차 = 1200 * log2(측정/목표)."""
    return 1200.0 * math.log2(measured_hz / target_hz)


def is_in_tune(measured_hz: float, target_hz: float, tol_cents: float) -> bool:
    return abs(error_cents(measured_hz, target_hz)) <= tol_cents


def min_step_cents(cal) -> float:
    """
    이 줄에서 낼 수 있는 가장 작은 음정 변화량(센트).

    허용 센트가 이 값보다 작으면 원리상 수렴할 수 없다. task_manager 가 시작 시 경고한다.
    """
    return cal.min_effective_deg * cal.cents_per_deg


def peg_angle(err_cents: float, cal) -> tuple:
    """
    오차 -> (direction, angle_deg). cal 은 peg_calibration.StringCalibration.

    direction 은 AdjustPeg.Goal.direction 규약을 따른다: +1 조임(음 높이기) / -1 풀림.
    angle_deg 는 항상 양수 크기.

    회전량은 실측 상수 cents_per_deg 로만 계산한다. 코드에 계수를 두지 않는다.
    실측 전에는 호출자가 CalibrationMissing 을 먼저 만나므로 여기까지 오지 않는다.
    """
    direction = 1 if err_cents < 0 else -1   # 음이 낮으면 높여야 함
    angle = abs(err_cents) / cal.cents_per_deg

    # 최소 유효 각도보다 작으면 페그가 실제로 움직이지 않는다.
    # ⚠ 미확정(브리핑 A3-3): 지금은 최소 각도로 올려서 보낸다. 대안은 '조절 포기'다.
    #    실측 전이라 이 경로는 아직 실행되지 않는다. 확정되면 여기만 고치면 된다.
    if angle < cal.min_effective_deg:
        angle = cal.min_effective_deg

    return direction, min(angle, cal.max_angle_deg)


def adjust_plan(err_cents: float, cal) -> list:
    """
    오차 -> AdjustPeg goal 목록. 백래시 마무리를 여기서 결정한다 (CLAUDE.md 3.2).

    확정 사항이라 문서에서 그대로 옮긴 규칙:
      - 최종 접근은 항상 조이는 방향이다.
      - 음이 낮으면(조여야 하면) 이미 조이는 방향이므로 goal 1개로 끝난다.
      - 음이 높으면 목표보다 backlash_release_cents 만큼 아래로 풀었다가 조여 올린다.
        goal 2개가 되며, 이 묶음 전체가 '조절 1회'다 (CLAUDE.md 3.2 "조절 1회가
        adjust_peg goal 2번이 될 수 있다").
      - 두 goal 사이에는 재측정하지 않는다. 묶음이 끝난 뒤 한 번 잰다.

    풀기 각도가 max_angle_deg 에 잘리면 조이기 goal 을 붙이지 않고 그 회차를 끝낸다.
    목표 아래에 머물게 되지만 다음 회차가 조이는 방향으로 다시 잡으므로
    '최종 접근은 조이는 방향' 규칙은 깨지지 않는다.
    """
    if err_cents < 0:
        return [peg_angle(err_cents, cal)]

    release_cents = err_cents + cal.backlash_release_cents
    release_dir, release_deg = peg_angle(release_cents, cal)
    plan = [(release_dir, release_deg)]

    clamped = release_deg >= cal.max_angle_deg
    if not clamped:
        # 푼 만큼 중 목표 아래로 내려간 몫만 조여 올린다
        plan.append(peg_angle(-cal.backlash_release_cents, cal))
    return plan


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
