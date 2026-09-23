"""실기 도구의 확인/무이동/운영 회복 호출을 하드웨어 없이 검증한다."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from tunio_robot import motion_primitives as MP

spec = importlib.util.spec_from_file_location(
    'j6_recovery_check', Path(__file__).resolve().parents[3] / 'tools/j6_recovery_check.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


@pytest.mark.parametrize('start,answer,status,end', [
    (300.5, 'RECOVER', 'recovered', 120.5),
    (-300.5, 'RECOVER', 'recovered', -120.5),
    (285, 'RECOVER', 'recovered', 105), (-285, 'RECOVER', 'recovered', -105),
    (100, 'RECOVER', 'not_needed', 100), (285, '', 'canceled', 285),
])
def test_실제_회복함수_재사용과_확인전_무이동(monkeypatch, start, answer, status, end):
    joints = [0., 0., 0., 0., 0., float(start)]
    moves = []
    monkeypatch.setattr(MP, '_api', SimpleNamespace(get_current_posj=lambda: joints[:]))
    monkeypatch.setattr(MP, '_node', None)

    def move(target, motion):
        moves.append(target)
        joints[:] = target
    monkeypatch.setattr(MP, '_movej', move)
    motion = {'j6_recovery_enabled': True, 'j6_recover_at_deg': 280,
              'peg_j6_limit_deg': 300}
    result = tool.recover_once(motion, confirm=lambda prompt: answer)
    assert result['status'] == status
    assert joints[5] == end
    assert len(moves) == (1 if status == 'recovered' else 0)


def test_확인중_자세변경시_이동하지_않는다(monkeypatch):
    states = iter([[0] * 5 + [285], [10] + [0] * 4 + [285]])
    monkeypatch.setattr(MP, '_read_peg_joints', lambda motion, **kwargs: next(states))
    with pytest.raises(MP.MotionError, match='관절각이 바뀌었다'):
        tool.recover_once({'j6_recovery_enabled': True}, confirm=lambda prompt: 'RECOVER')
