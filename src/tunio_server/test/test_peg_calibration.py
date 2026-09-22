"""
peg_calibration 로딩 테스트.

핵심은 '실측 전에는 값을 내주지 않는다'는 것이다.
임시 계수로 로봇이 페그를 돌리는 사고를 막는 관문이라 테스트로 고정해 둔다.
"""
from pathlib import Path

import pytest

from tunio_server import peg_calibration as PC

CONFIG = Path(__file__).resolve().parent.parent / 'config' / 'peg_calibration.yaml'

FILLED = {
    'cents_per_deg': 5.0,
    'min_effective_deg': 0.5,
    'max_angle_deg': 20.0,
    'backlash_release_cents': 25.0,
}


def _doc(measured, strings=None):
    block = {'measured': measured, 'measured_at': '2026-09-19'}
    for num in (6, 5, 2, 1):
        block['s%d' % num] = dict(strings or FILLED)
    return {'/task_manager': {'ros__parameters': {'peg_calibration': block}}}


def test_unmeasured_refuses_to_give_values():
    cal = PC.parse(_doc(False, {k: 0.0 for k in FILLED}))
    assert not cal.measured
    assert cal.missing_reason()
    with pytest.raises(PC.CalibrationMissing):
        cal.get(6)


def test_measured_true_with_empty_values_is_rejected():
    """'실측했다'고 표시해 놓고 값은 0인 상태로 로봇이 돌면 안 된다."""
    with pytest.raises(PC.CalibrationMissing):
        PC.parse(_doc(True, {k: 0.0 for k in FILLED}))


def test_measured_returns_per_string_values():
    cal = PC.parse(_doc(True))
    for num in (6, 5, 2, 1):
        assert cal.get(num).cents_per_deg == pytest.approx(5.0)
    assert cal.missing_reason() == ''


def test_unknown_string_number():
    cal = PC.parse(_doc(True))
    with pytest.raises(PC.CalibrationMissing):
        cal.get(3)          # 3번 줄은 이번 범위가 아니다


def test_missing_block():
    with pytest.raises(PC.CalibrationMissing):
        PC.parse({'/task_manager': {'ros__parameters': {}}})


def test_missing_file():
    with pytest.raises(PC.CalibrationMissing):
        PC.load('/존재하지/않는/경로.yaml')


def test_repo_config_is_measured():
    """저장소 파일은 2026-09-22 로 실측이 끝났다 (#8).

    그 전에는 'measured 가 false 여야 한다'를 지키는 테스트였다. 네 줄 16칸이
    다 차면서 뒤집었다. 지금 지키는 것은 **되돌아가지 않는 것**이다 — 누가
    값을 지우거나 measured 를 내리면 여기서 걸린다.
    """
    cal = PC.load(CONFIG)
    assert cal.measured
    assert cal.measured_at


def test_repo_config_all_fields_filled():
    """네 줄 x 네 칸이 다 0 보다 커야 한다.

    parse() 가 measured: true 일 때 이미 검사하지만, 여기서 한 번 더 본다.
    로드가 조용히 통과하고 조율 중에 0 으로 나누는 쪽이 더 나쁘기 때문이다.
    """
    cal = PC.load(CONFIG)
    for num in (6, 5, 2, 1):
        s = cal.get(num)
        assert s.cents_per_deg > 0, num
        assert s.min_effective_deg > 0, num
        assert s.max_angle_deg > 0, num
        assert s.backlash_release_cents > 0, num


def test_repo_config_converges_within_tolerance():
    """min_effective_deg x cents_per_deg <= 허용 센트(10) 여야 한다.

    한 번에 돌릴 수 있는 가장 작은 각도가 허용 오차보다 큰 음을 만들면 그 줄은
    영원히 진동한다. 수렴 한계라고 부르는 조건이고, 실측 때마다 손으로 확인하던
    것을 여기에 박아 둔다 (docs/페그_캘리브레이션_가이드.md).
    """
    TOLERANCE_CENTS = 10.0
    cal = PC.load(CONFIG)
    for num in (6, 5, 2, 1):
        s = cal.get(num)
        step = s.min_effective_deg * s.cents_per_deg
        assert step <= TOLERANCE_CENTS, (
            f'{num}번 줄: 최소 유효 각도 {s.min_effective_deg}도 x '
            f'{s.cents_per_deg} 센트/도 = {step:.1f}센트 > 허용 {TOLERANCE_CENTS}센트')


def test_repo_config_max_angle_exceeds_min_effective():
    """max_angle_deg 가 min_effective_deg 보다 커야 한다.

    작으면 peg_angle() 의 최소각 보정이 min(angle, max) 에 무효화되어
    페그가 안 움직이는 각도를 보낸다 (peg_calibration.yaml 주석).
    """
    cal = PC.load(CONFIG)
    for num in (6, 5, 2, 1):
        s = cal.get(num)
        assert s.max_angle_deg > s.min_effective_deg, num
