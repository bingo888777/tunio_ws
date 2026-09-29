"""단방향 전환의 경유 순서와 반대 방향 호환. 로봇/API 연결 없음."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from tunio_robot import motion_primitives as MP, teach_points as TP


def settings():
    return TP.load(Path(__file__).parents[1] / 'config/teach_points.yaml').motion


def test_접은채_J1을_먼저_넘긴_뒤_J3을_전환한다():
    start = [-14.24, 38.03, 37.04, 0, 104.92, 77.21]
    goal = [122.66, -39.39, -75.70, 60.55, -134.08, 53.17]
    path = MP.compact_left_path(start, goal, settings())
    assert path[0] == start and path[-1] == goal
    assert start[0] < path[1][0] < goal[0]
    assert path[1][1:5] == path[2][1:5]
    assert path[2][0] == path[3][0] == path[4][0]
    assert all(q[2] > 0 for q in path[:-1])
    assert path[1][5] == path[2][5] == start[5]
    assert path[3][5] == goal[5]
    assert not any(q[1:5] == [0, 0, 0, 0] for q in path)


@pytest.mark.parametrize('bad', ['reverse', 'nan', 'length', 'limits', 'fraction', 'j1'])
def test_역방향과_잘못된_접기설정은_거부한다(bad):
    start, goal = [0, 40, 70, 0, 90, 0], [120, -40, -70, 60, -130, 40]
    motion = settings()
    config = motion['peg_transition_left']
    if bad == 'reverse':
        start, goal = goal, start
    elif bad == 'nan':
        config['fold_j2_j5'][0] = float('nan')
    elif bad == 'length':
        config['ready_j2_j5'] = [0]
    elif bad == 'limits':
        config['fold_j2_j5'][0] = 100
    elif bad == 'fraction':
        config['j1_overlap_fraction'] = 2
    else:
        start[0] = -178
    with pytest.raises(MP.MotionError):
        MP.compact_left_path(start, goal, motion)


def test_좌측전환만_목표계획으로_미루고_우측전환은_기존경유한다(monkeypatch):
    calls = []
    monkeypatch.setattr(MP, '_current_j1', lambda: 0)
    monkeypatch.setattr(MP, 'goto_route', lambda r, m: calls.append(r))
    route = TP.Route('transit', [90, 0, 0, 0, 0, None], True)
    MP.approach_peg_side('left', route, settings())
    assert not calls
    monkeypatch.setattr(MP, '_current_j1', lambda: 170)
    MP.approach_peg_side('right', route, settings())
    assert calls == [route]


@pytest.mark.parametrize('operation', ['register', 'adjust'])
def test_피크반환이_아니어도_등록과_조절에서_좌측전환을_계획한다(monkeypatch, operation):
    monkeypatch.setattr(MP, '_require_api', lambda: object())
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_current_j1', lambda: 0)
    monkeypatch.setattr(MP, '_read_peg_joints', lambda m: [0, 40, 70, 0, 90, 0])
    monkeypatch.setattr(MP, '_j6_pose', lambda p, *a: (p, 0))
    monkeypatch.setattr(MP, '_grip', lambda name: None)
    points = TP.load(Path(__file__).parents[1] / 'config/teach_points.yaml')
    calls = []

    def planned(pose, side, route, motion, logger):
        calls.append((pose.name, side))
        raise MP.MotionError('계획 진입 확인')
    monkeypatch.setattr(MP, '_pick_to_peg_approach', planned)
    with pytest.raises(MP.MotionError, match='계획 진입 확인'):
        if operation == 'register':
            MP.register_peg(points.peg(6), points.motion, {'thresholds_mm': {}},
                            read_width=lambda: 20, side='left')
        else:
            MP.adjust_peg(points.peg(6), 6, 1, points.motion, side='left')
    assert calls == [('pegs.6', 'left')]


@pytest.mark.parametrize('index,value', [(0, 136.459), (1, -141.72)])
def test_실기에서_거부된_J5_목표를_계획에서_차단한다(index, value):
    start, goal = [0, 0, 75, 0, 100, 0], [122, -43, -78, 55, -134, 46]
    motion = settings()
    if index == 0:
        motion['peg_transition_left']['fold_j2_j5'][3] = value
    else:
        goal[4] = value
    with pytest.raises(MP.MotionError, match='J5='):
        MP.compact_left_path(start, goal, motion)


@pytest.mark.parametrize('peg', [6, 5])
@pytest.mark.parametrize('goal_j5', [-134.82, -141.72])
def test_홈에서_출발해도_상단250과_기존하강점을_사용한다(monkeypatch, peg, goal_j5):
    points = TP.load(Path(__file__).parents[1] / 'config/teach_points.yaml')
    start = [0, 0, 75, 0, 100, 0]
    pose = points.peg(peg)
    approach = TP.backed_off(pose.task, points.motion['approach_mm'])
    tasks = {}

    def ikin(task, sol):
        q = [122, -43, -78, 55, goal_j5 if task[2] == 250 else -124, 46]
        tasks[tuple(q)] = task[:3]
        return q

    monkeypatch.setitem(sys.modules, 'DR_common2', SimpleNamespace(posx=list))
    monkeypatch.setattr(MP, '_api', SimpleNamespace(
        get_current_posj=lambda: list(start), ikin=ikin))
    monkeypatch.setattr(MP, 'read_posx', lambda: ([382.61, 5.16, 313.02, 179, -175, 179], 2))
    from tunio_robot import kinematics as K
    monkeypatch.setattr(K, 'fk', lambda q: {'tcp': tasks[tuple(q)]})
    if goal_j5 < -135:
        with pytest.raises(MP.MotionError, match='J5='):
            MP._pick_to_peg_approach(pose, 'left', points.route(), points.motion, execute=False)
        return
    result = MP._pick_to_peg_approach(
        pose, 'left', points.route(), points.motion, execute=False)
    assert result['high_tcp'] == approach[:2] + [250] + approach[3:]
    assert result['lower_tcp'] == approach
    assert len(result['joint_plan']) == 4
    assert all(abs(q[4]) <= 135 for q in result['joint_plan'])
