"""실기 도구의 기본 미연결과 공통 이동 호출 경로. 로봇 연결 없음."""
import importlib.util
from pathlib import Path

import pytest

from tunio_robot import motion_primitives as MP, teach_points as TP


@pytest.fixture
def tool(monkeypatch):
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root/'tools'))
    spec = importlib.util.spec_from_file_location('work_check', root/'tools/work_motion_check.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_실기옵션_없으면_연결도_하지_않는다(tool, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('로봇에 연결하면 안 된다')
    monkeypatch.setattr(MP, 'connect', forbidden)
    assert tool.main([]) == 0


@pytest.mark.parametrize('args', [['--speed-scale', 'nan'], ['--peg', '2'],
                                  ['--scene', 'holder', '--high-only']])
def test_잘못된_입력은_연결전에_거부한다(tool, args):
    with pytest.raises(SystemExit) as exc:
        tool.main(args)
    assert exc.value.code == 2


@pytest.mark.parametrize('high_only', [False, True])
def test_전환은_실제_공통함수에_실행확인과_하강선택을_전달한다(tool, monkeypatch, high_only):
    points = TP.load(tool.ROOT/'src/tunio_robot/config/teach_points.yaml')
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_current_j1', lambda: -14)
    monkeypatch.setattr(MP, '_j6_pose', lambda p, m: (p, 0))
    got = []
    monkeypatch.setattr(MP, '_pick_to_peg_approach', lambda *a, **k: got.append(k))

    def ask(message):
        pass
    tool.run(points, 'transition', 6, points.motion, execute=True,
             high_only=high_only, ask=ask)
    assert got == [{'confirm': ask, 'execute': True, 'high_only': high_only}]


def test_반대방향_출발을_자동보정하지_않는다(tool, monkeypatch):
    points = TP.load(tool.ROOT/'src/tunio_robot/config/teach_points.yaml')
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_current_j1', lambda: 170)
    with pytest.raises(MP.MotionError, match='오른쪽 작업 자세'):
        tool.run(points, 'transition', 6, points.motion, execute=True)


@pytest.mark.parametrize('scene', ['holder', 'pluck'])
def test_집기와_발현은_현재_sol_높이경로로_상단까지만_간다(tool, monkeypatch, scene):
    points = TP.load(tool.ROOT/'src/tunio_robot/config/teach_points.yaml')
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_j6_pose', lambda p, m: (p, 0))
    got = []
    monkeypatch.setattr(MP, '_transfer_above_floor', lambda *a, **k: got.append((a, k)))
    tool.run(points, scene, 6, points.motion, execute=False)
    assert got[0][0][0][2] == 224
    assert got[0][1]['execute'] is False
