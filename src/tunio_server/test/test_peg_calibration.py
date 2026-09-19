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


def test_repo_config_is_unmeasured():
    """저장소에 들어 있는 파일은 실측 전이어야 한다. 값이 채워지면 이 테스트를 고친다."""
    cal = PC.load(CONFIG)
    assert not cal.measured
