"""새 측정이 오기 전 이전 줄의 Hz를 현재 줄 값으로 발행하지 않는다."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from tunio_server import task_manager as TM


@pytest.mark.parametrize('next_string,phase', [
    (5, TM.TuningState.PHASE_MEASURE_READY),
    (6, TM.TuningState.PHASE_MEASURE_READY),
    (5, TM.TuningState.PHASE_VERIFY),
    (6, TM.TuningState.PHASE_VERIFY),
    (0, TM.TuningState.PHASE_RETURN),
])
def test_new_measurement_clears_live_value_but_keeps_string_results(next_string, phase):
    rec = TM.StringResult(string_number=6, tuned_hz=81.36, has_tuned_hz=True)
    before = deepcopy(rec)
    states = []
    node = SimpleNamespace(
        current_string=6, measured_hz=81.36, measurement_valid=True,
        adjust_count=0, results={6: rec})
    node._publish_state = lambda: states.append(
        (node.current_string, node.measured_hz, node.measurement_valid))
    TM.TaskManager._set_phase(node, phase, next_string)
    assert states == [(next_string, 0.0, False)]
    assert rec == before


def test_same_string_adjustment_keeps_its_measurement():
    node = SimpleNamespace(
        current_string=6, measured_hz=81.36, measurement_valid=True,
        _publish_state=lambda: None)
    TM.TaskManager._set_phase(node, TM.TuningState.PHASE_ADJUST_PEG, 6)
    assert node.measured_hz == 81.36
    assert node.measurement_valid
