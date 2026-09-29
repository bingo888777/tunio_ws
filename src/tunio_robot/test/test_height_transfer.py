"""고정 홈 대신 현재 sol로 높이 경로를 잇는다. 로봇 연결 없는 회귀 검사."""
import sys
from types import SimpleNamespace

import pytest

from tunio_robot import kinematics as K, motion_primitives as MP
from tunio_robot.teach_points import Pose


@pytest.fixture
def scene(monkeypatch):
    sim = SimpleNamespace(tcp=[400., 0., 190., 0., 180., 0.], sol=7,
                          calls=[], ik_calls=[], stall=None, reject=None)

    def joints(task):
        base = 170 if sim.sol == 7 else -15
        return [base + task[0] / 100, task[0] / 10, task[1] / 10, task[2] / 10, 90., 0.]

    sim.joints = joints(sim.tcp)

    def ikin(task, sol):
        sim.ik_calls.append(sol)
        assert sol == sim.sol
        return joints(task)

    def fk(q):
        z = q[3] * 10
        if sim.reject == 'midpoint' and 49 < q[1] < 51:
            z = 200
        return {'tcp': [q[1] * 10, q[2] * 10, z],
                'origins': [[0, 0, 134.5]] * 2 + [[0, 0, 400]] * 4,
                'wrist_casing': [0, 0, 400]}

    def movel(task, v, a):
        sim.calls.append(('movel', list(task)))
        if sim.stall != len(sim.calls):
            sim.tcp = list(task)
            sim.joints = joints(task)

    def movej(q, motion):
        sim.calls.append(('movej', list(q)))
        if sim.stall != len(sim.calls):
            sim.joints = list(q)
            sim.tcp[:3] = fk(q)['tcp']

    monkeypatch.setitem(sys.modules, 'DR_common2', SimpleNamespace(posx=list))
    monkeypatch.setattr(MP, '_api', SimpleNamespace(
        DR_BASE=0, ikin=ikin, get_current_posj=lambda: list(sim.joints),
        get_current_posx=lambda *a: (list(sim.tcp), sim.sol)))
    monkeypatch.setattr(MP, '_cancel_check', None)
    monkeypatch.setattr(MP, '_j6_motion', None)
    monkeypatch.setattr(K, 'fk', fk)
    monkeypatch.setattr(MP, '_movel_base', movel)
    monkeypatch.setattr(MP, '_movej', movej)
    monkeypatch.setattr(MP, '_movel_tool_rel', lambda *a, **k: None)
    monkeypatch.setattr(MP, 'ensure_tcp', lambda: None)
    monkeypatch.setattr(MP, '_gripper', SimpleNamespace(select=lambda name: None))
    sim.make_joints = joints
    return sim


@pytest.mark.parametrize('sol', [2, 7])
def test_양쪽_모두_현재_sol로_상승_이동_하강하고_홈을_안_간다(scene, sol):
    scene.sol = sol
    scene.joints = scene.make_joints(scene.tcp)
    start_j1 = scene.joints[0]
    target = [600, -20, 207, 0, 180, 0]
    MP._transfer_above_floor(target, {})
    assert [kind for kind, _ in scene.calls] == ['movel', 'movej', 'movel']
    assert scene.calls[0][1] == [400, 0, 224, 0, 180, 0]
    assert scene.calls[-1][1] == target
    assert scene.ik_calls == [sol] * 3
    assert abs(scene.joints[0] - start_j1) == pytest.approx(2)
    assert all(q[0] not in (0, 90) for kind, q in scene.calls if kind == 'movej')


@pytest.mark.parametrize('failure', ['ik', 'flip', 'nan'])
def test_IK_J1전환_검사실패는_상승전부터_차단한다(scene, monkeypatch, failure):
    scene.reject = failure
    if failure != 'midpoint':
        def bad_ik(task, sol):
            if failure == 'ik':
                raise ValueError('해 없음')
            q = scene.make_joints(task)
            q[0] = 0 if failure == 'flip' else float('nan')
            return q
        monkeypatch.setattr(MP._api, 'ikin', bad_ik)
    with pytest.raises(MP.MotionError):
        MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {})
    assert scene.calls == []


@pytest.mark.parametrize('stage', [1, 2, 3])
def test_이동성공반환이어도_미도달이면_다음단계로_안_간다(scene, stage):
    scene.stall = stage
    with pytest.raises(MP.MotionError, match='미도달'):
        MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {})
    assert len(scene.calls) == stage


def test_이미_높으면_먼저_낮추지_않고_목표위에서만_하강한다(scene):
    scene.tcp[2] = 300
    scene.joints = scene.make_joints(scene.tcp)
    MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {})
    assert [kind for kind, _ in scene.calls] == ['movej', 'movel']
    assert scene.calls[0][1][3] * 10 == 300
    assert scene.calls[-1][1][:3] == [600, 0, 224]


def test_집기_발현_반환_연속동작이_같은쪽에_남는다(scene):
    holder = Pose('pick.holder', True, task=[590, -140, 164, 0, 180, 0])
    pluck = Pose('pluck.6', True, task=[645, -29, 187.7, 0, 180, 0])
    home = Pose('frames.standby', True, joints=[0, 0, 75, 0, 100, 0])
    motion = {'pick_settle_sec': 0}
    MP.pick_up(holder, motion, standby=home, via=home, height_transfer=True)
    assert scene.tcp[:3] == [590, -140, 224]
    MP.pluck_string(pluck, motion, standby=home, height_transfer=True, prepare_only=True)
    assert scene.tcp[:3] == [645, -29, 224]
    before = len(scene.ik_calls)
    MP.pluck_string(pluck, motion, standby=home, height_transfer=True, prepared=True)
    assert len(scene.ik_calls) == before  # 준비된 발현은 전환 경로를 다시 계획하지 않는다
    assert scene.tcp[:3] == [645, -29, 224]
    MP.pick_return(holder, motion, via=home, height_transfer=True)
    assert scene.tcp[:3] == [590, -140, 224]
    assert all(170 < q[0] < 180 for kind, q in scene.calls if kind == 'movej')
    assert set(scene.ik_calls) == {7}


def test_IK목표의_동등각은_허용구간에서_고르되_실제출발각은_환산하지_않는다(scene, monkeypatch):
    def wrapped_ik(task, sol):
        q = scene.make_joints(task)
        q[0] -= 360
        return q
    monkeypatch.setattr(MP._api, 'ikin', wrapped_ik)
    MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {})
    assert scene.joints[0] == 176
    scene.joints[0] -= 360
    scene.calls.clear()
    with pytest.raises(MP.MotionError, match='J1 전환'):
        MP._transfer_above_floor([590, 0, 224, 0, 180, 0], {})
    assert scene.calls == []


def test_임의높이_기준을_밑돈다고_새_금지선처럼_차단하지_않는다(scene):
    scene.reject = 'midpoint'
    # 진단으로는 기준 미달이지만 실제 금지구역 판정과 혼동하지 않는다.
    a = scene.make_joints([400, 0, 224, 0, 180, 0])
    b = scene.make_joints([600, 0, 224, 0, 180, 0])
    with pytest.raises(MP.MotionError, match='높이 부족'):
        MP._check_peg_transfer_path(a, b, 214, 5)
    MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {})
    assert scene.tcp[:3] == [600, 0, 207]


def test_툴방향이_다르면_현재XY위에서_상승후_자세를_바꾼다(scene):
    scene.tcp[4] = 90
    MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {})
    assert scene.calls[0] == ('movel', [400, 0, 299, 0, 90, 0])
    assert scene.tcp[:3] == [600, 0, 224]
