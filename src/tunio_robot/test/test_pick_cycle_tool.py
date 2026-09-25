"""단독 집기 도구의 단계 순서와 사용자 중단. 실제 로봇 호출 없음."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from tunio_robot import motion_primitives as MP, teach_points as TP

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location(
    'pick_cycle_check', ROOT / 'tools/pick_cycle_check.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.mark.parametrize('cancel_at', [None, 5])
def test_집기반환_순서와_중단후_자동개방금지(monkeypatch, cancel_at):
    points = TP.load(ROOT / 'src/tunio_robot/config/teach_points.yaml')
    events = []
    monkeypatch.setitem(sys.modules, 'DR_common2', SimpleNamespace(posx=list))
    api = SimpleNamespace(ikin=lambda task, sol: [166, 0, 0, 0, -100, -100],
                          get_current_solution_space=lambda: 7)
    monkeypatch.setattr(MP, '_require_api', lambda: api)
    monkeypatch.setattr(MP, '_read_peg_joints', lambda m: [166, 0, 0, 0, -100, -100])

    def choose(holder, motion):
        assert holder.sol == 7
        return holder, 0
    monkeypatch.setattr(MP, '_j6_pose', choose)
    monkeypatch.setattr(MP, '_movej', lambda *args: events.append('approach'))
    monkeypatch.setattr(MP, '_movel_base', lambda *args: events.append('vertical'))
    monkeypatch.setattr(MP, '_check_pick_return_position', lambda *args: None)
    monkeypatch.setattr(MP, '_release', lambda: events.append('open'))
    monkeypatch.setattr(MP, '_grip', lambda *args: events.append('grip'))
    monkeypatch.setattr(tool.time, 'sleep', lambda t: None)
    prompts = []

    def confirm(prompt):
        prompts.append(prompt)
        return 'stop' if len(prompts) == cancel_at else ''
    report = {}
    if cancel_at:
        with pytest.raises(RuntimeError, match='사용자 중단'):
            tool.run_cycle(points, 7, confirm, report)
        assert events == ['approach', 'open', 'vertical', 'grip']
    else:
        tool.run_cycle(points, 7, confirm, report)
        assert events == ['approach', 'open', 'vertical', 'grip',
                          'vertical', 'vertical', 'open', 'vertical']
        assert len(report['steps']) == 8
