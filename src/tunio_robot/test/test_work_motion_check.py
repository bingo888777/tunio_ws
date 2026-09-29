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


@pytest.mark.parametrize('args', [[], ['--scene', 'pick-pluck']])
def test_실기옵션_없으면_연결도_하지_않는다(tool, monkeypatch, args):
    def forbidden(*args, **kwargs):
        raise AssertionError('로봇에 연결하면 안 된다')
    monkeypatch.setattr(MP, 'connect', forbidden)
    assert tool.main(args) == 0


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


def test_탐색후에도_정지응답을_처리할_실행기에_노드가_남는다(tool, monkeypatch, tmp_path):
    import rclpy.executors
    original = rclpy.executors.SingleThreadedExecutor
    executors = []

    def create():
        executor = original()
        executors.append(executor)
        return executor

    def diagnostic(*args, **kwargs):
        assert any(n.get_name() == 'work_motion_check' for n in executors[0].get_nodes())
        return {}
    monkeypatch.setattr(rclpy.executors, 'SingleThreadedExecutor', create)
    # 실행기 소속 검사만 한다. 외부/다른 테스트의 ROS 노드 탐색에 의존하지 않는다.
    monkeypatch.setattr(rclpy.node.Node, 'get_node_names', lambda self: [self.get_name()])
    monkeypatch.setattr(tool, 'confirm', lambda message: None)
    monkeypatch.setattr(tool, 'run', diagnostic)
    monkeypatch.setattr(MP, 'connect', lambda *a, **k: None)
    monkeypatch.setattr(MP, '_node', None)
    monkeypatch.setattr(tool.RL, 'session_path', lambda *a: str(tmp_path/'check.json'))
    assert tool.main(['--check-only']) == 0


def test_집기발현_조회는_거치대계획만_읽고_그리퍼와_발현을_실행하지_않는다(tool, monkeypatch):
    points = TP.load(tool.ROOT/'src/tunio_robot/config/teach_points.yaml')
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_j6_pose', lambda p, m: (p, 0))
    seen = []

    def plan(*args, **kwargs):
        seen.append(kwargs['execute'])
        return {'joint_plan': []}

    def forbidden(*args, **kwargs):
        raise AssertionError('조회에서 집기/발현을 하면 안 된다')
    monkeypatch.setattr(MP, '_transfer_above_floor', plan)
    monkeypatch.setattr(MP, 'pick_up', forbidden)
    monkeypatch.setattr(MP, 'pluck_string', forbidden)
    result = tool.run(points, 'pick-pluck', 6, points.motion)
    assert result['holder_approach'] == {'joint_plan': []}
    assert seen == [False]


@pytest.mark.parametrize('stop_at', [1, 2, 3, None])
def test_집기_준비_발현_순서와_확인취소시_후속명령차단(tool, monkeypatch, stop_at):
    points = TP.load(tool.ROOT/'src/tunio_robot/config/teach_points.yaml')
    calls, questions = [], []
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, 'pick_up', lambda *a, **k: calls.append(('pick', k)))
    monkeypatch.setattr(MP, 'pluck_string', lambda *a, **k: calls.append(('pluck', k)))

    def ask(message):
        questions.append(message)
        if len(questions) == stop_at:
            raise RuntimeError('시험 중단')
    expected = [('pick', {'height_transfer': True}),
                ('pluck', {'height_transfer': True, 'prepare_only': True}),
                ('pluck', {'height_transfer': True, 'prepared': True})]
    if stop_at is not None:
        with pytest.raises(RuntimeError, match='시험 중단'):
            tool.run(points, 'pick-pluck', 6, points.motion, execute=True, ask=ask)
        assert calls == expected[:stop_at - 1]
    else:
        result = tool.run(points, 'pick-pluck', 6, points.motion, execute=True, ask=ask)
        assert calls == expected
        assert result['plucks_commanded'] == 1
        assert not result['home_commanded'] and not result['pick_returned']
