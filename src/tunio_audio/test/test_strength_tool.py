"""#86 노드·설정·측정 도구의 분석 구간과 결과 표시를 맞춘다."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[3]


def strength_tool():
    spec = importlib.util.spec_from_file_location(
        'pluck_strength', ROOT / 'tools/pluck_strength.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_yaml_uses_node_skip_default():
    tree = ast.parse((ROOT / 'src/tunio_audio/tunio_audio/audio_analyzer.py').read_text())
    default = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'SKIP_HEAD_SEC' for t in n.targets))
    params = yaml.safe_load((ROOT / 'src/tunio_audio/config/audio_params.yaml').read_text())
    assert params['/audio_analyzer']['ros__parameters']['skip_head_sec'] == default


def test_strength_tool_discards_half_second(monkeypatch):
    tool = strength_tool()
    samples = np.arange(2600, dtype=float)
    recorder = SimpleNamespace(flush=lambda: None, read=lambda duration: samples, sample_rate=1000)
    monkeypatch.setattr(tool, 'find_onsets', lambda *args: [0.1])
    seen = []
    monkeypatch.setattr(tool.pitch, 'analyze', lambda segment, *a, **kw: seen.append(segment))
    tool.measure_once(recorder, 82.4, 2.0)
    np.testing.assert_array_equal(seen[0], samples[600:2100])


def test_failed_pitch_is_not_reported_as_passed(capsys):
    tool = strength_tool()
    tool.report_string(6, [{'level': '약', 'ok': False, 'rms_db': -25.0,
                           'prominence_db': 30.0, 'cents': None, 'reason': 3}])
    assert '관문 통과 0/1' in capsys.readouterr().out
