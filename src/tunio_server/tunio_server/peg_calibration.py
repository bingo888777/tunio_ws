"""
페그 캘리브레이션 로딩 (rclpy 비의존).

config/peg_calibration.yaml 을 읽어 줄별 실측 상수를 준다.
실측 전에는 measured=false 이며, 이 상태에서 값을 꺼내면 CalibrationMissing 을 던진다.
임시값으로 로봇을 돌리지 않기 위한 것이다 (CLAUDE.md 11절).

yaml 구조는 ROS 2 파라미터 파일 형식 그대로다. 나중에 launch 에서
--params-file 로 넘기는 방식으로 바꿔도 같은 파일을 쓸 수 있게 했다.
"""
from dataclasses import dataclass
from pathlib import Path

import yaml


class CalibrationMissing(Exception):
    """실측 전이거나 값이 비어 있어 회전량을 계산할 수 없다."""


@dataclass(frozen=True)
class StringCalibration:
    """줄 하나의 페그 실측 상수. 단위는 yaml 주석 참조."""

    cents_per_deg: float
    min_effective_deg: float
    max_angle_deg: float
    backlash_release_cents: float


# yaml 안의 줄 키 ('s6') <-> 실제 줄 번호(6) 대응
def _key(string_number: int) -> str:
    return 's%d' % int(string_number)


class PegCalibration:
    """줄별 StringCalibration 묶음. measured 가 false 면 조회 시 예외를 던진다."""

    def __init__(self, measured: bool, measured_at: str, strings: dict):
        self.measured = bool(measured)
        self.measured_at = str(measured_at or '')
        self._strings = dict(strings)

    def get(self, string_number: int) -> StringCalibration:
        """해당 줄의 실측 상수. 실측 전이면 CalibrationMissing."""
        if not self.measured:
            raise CalibrationMissing(
                '페그 캘리브레이션 미실측 (measured: false). '
                'tools/peg_cal.py 로 재고 config/peg_calibration.yaml 을 채워야 한다')
        cal = self._strings.get(_key(string_number))
        if cal is None:
            raise CalibrationMissing('%d번 줄 캘리브레이션 항목이 없다' % string_number)
        return cal

    def missing_reason(self) -> str:
        """왜 못 쓰는지 사람이 읽을 사유. 쓸 수 있으면 빈 문자열."""
        if not self.measured:
            return '페그 캘리브레이션 미실측 (config/peg_calibration.yaml 의 measured: false)'
        return ''


def _as_float(raw, path: str) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        raise CalibrationMissing('%s 값이 숫자가 아니다: %r' % (path, raw))


def parse(doc: dict) -> PegCalibration:
    """
    파싱된 yaml dict -> PegCalibration.

    measured 가 true 인데 값이 비어 있으면(0 이하) 그 자리에서 예외를 던진다.
    '실측했다고 표시해 놓고 값은 0' 인 상태로 로봇이 도는 것을 막는다.
    """
    node = doc.get('/task_manager') or doc.get('task_manager') or {}
    params = node.get('ros__parameters', node) or {}
    block = params.get('peg_calibration')
    if block is None:
        raise CalibrationMissing('yaml 에 peg_calibration 항목이 없다')

    measured = bool(block.get('measured', False))
    strings = {}
    for num in (6, 5, 2, 1):
        key = _key(num)
        raw = block.get(key)
        if raw is None:
            if measured:
                raise CalibrationMissing('measured: true 인데 %s 항목이 없다' % key)
            continue
        cal = StringCalibration(
            cents_per_deg=_as_float(raw.get('cents_per_deg'), '%s.cents_per_deg' % key),
            min_effective_deg=_as_float(
                raw.get('min_effective_deg'), '%s.min_effective_deg' % key),
            max_angle_deg=_as_float(raw.get('max_angle_deg'), '%s.max_angle_deg' % key),
            backlash_release_cents=_as_float(
                raw.get('backlash_release_cents'), '%s.backlash_release_cents' % key),
        )
        if measured:
            for field, value in (('cents_per_deg', cal.cents_per_deg),
                                 ('min_effective_deg', cal.min_effective_deg),
                                 ('max_angle_deg', cal.max_angle_deg),
                                 ('backlash_release_cents', cal.backlash_release_cents)):
                if value <= 0.0:
                    raise CalibrationMissing(
                        'measured: true 인데 %s.%s 가 비어 있다 (%r). '
                        '실측값을 채우거나 measured 를 false 로 되돌려라'
                        % (key, field, value))
        strings[key] = cal

    return PegCalibration(measured, block.get('measured_at', ''), strings)


def load(path) -> PegCalibration:
    """YAML 파일 경로를 읽어 PegCalibration 을 만든다. 없으면 CalibrationMissing."""
    p = Path(path)
    if not p.is_file():
        raise CalibrationMissing('캘리브레이션 파일이 없다: %s' % p)
    with p.open('r', encoding='utf-8') as fh:
        doc = yaml.safe_load(fh) or {}
    return parse(doc)


def unmeasured() -> PegCalibration:
    """파일을 못 읽었을 때 쓰는 빈 캘리브레이션. 조회하면 예외를 던진다."""
    return PegCalibration(False, '', {})
