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
        get_current_solution_space=lambda: sim.sol,
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


def _keep_j6_on_movel(scene, monkeypatch, shift=0.0):
    # 가짜 movel 은 관절을 IK 표로 되돌려 J6 가 0 이 된다. 실제 팔은 직선 이동에서 J6 를
    # 이어가므로 그 연속성을 흉내 낸다 (shift 로 한 바퀴 어긋난 경우도 만든다)
    orig = MP._movel_base
    pending = [shift]

    def movel(task, v, a):
        j6 = scene.joints[5]
        orig(task, v, a)
        scene.joints[5] = j6 + pending.pop() if pending else j6   # 어긋남은 첫 직선 이동 한 번만
    monkeypatch.setattr(MP, '_movel_base', movel)


def test_IK의_J6동등각은_실제손목에_가까운쪽으로_명령한다(scene, monkeypatch):
    # 9/29 실기: 팔 J6 -199.8 에 IK 가 +160.2 를 주자 도착 검사가 "미도달"(정확히 360도 차)
    scene.joints[5] = -199.8
    ik = scene.make_joints

    def ikin(task, sol):
        q = ik(task)
        q[5] = 160.2
        return q
    monkeypatch.setattr(MP._api, 'ikin', ikin)
    _keep_j6_on_movel(scene, monkeypatch)
    MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {})
    assert [kind for kind, _ in scene.calls] == ['movel', 'movej', 'movel']
    movej = [q for kind, q in scene.calls if kind == 'movej']
    assert all(q[5] == pytest.approx(-199.8) for q in movej)   # 한 바퀴 안 돌린다


def test_직선이동이_J6를_한바퀴_다르게_남겨도_실제기준으로_이어간다(scene, monkeypatch):
    scene.joints[5] = -199.8
    ik = scene.make_joints

    def ikin(task, sol):
        q = ik(task)
        q[5] = 160.2
        return q
    monkeypatch.setattr(MP._api, 'ikin', ikin)
    _keep_j6_on_movel(scene, monkeypatch, shift=360.0)     # 첫 상승 뒤 J6 가 +160.2 로 읽힘
    MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {})
    movej = [q for kind, q in scene.calls if kind == 'movej']
    assert movej and all(q[5] == pytest.approx(160.2) for q in movej)


def test_J6동등각이_한계를_넘으면_첫명령전_차단한다(scene, monkeypatch):
    scene.joints[5] = -190.0
    ik = scene.make_joints

    def ikin(task, sol):
        q = ik(task)
        q[5] = 100.0                                          # -190 에 가까운 표현은 -260
        return q
    monkeypatch.setattr(MP._api, 'ikin', ikin)
    with pytest.raises(MP.MotionError, match='J6'):
        MP._transfer_above_floor([600, 0, 207, 0, 180, 0], {'peg_j6_limit_deg': 200})
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


def test_높이_이동_계획조회는_실물명령을_보내지_않는다(scene):
    result = MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {}, execute=False)
    assert result['sol'] == scene.sol
    assert len(result['joint_plan']) == 3
    assert scene.calls == []


def test_높이_이동_단계확인_거부시_후속명령이_없다(scene):
    def reject(message):
        raise RuntimeError('단계 거부')
    with pytest.raises(RuntimeError, match='단계 거부'):
        MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {}, confirm=reject)
    assert scene.calls == []


@pytest.mark.parametrize('sol,raise_z', [(7, 250), (2, 245)])
def test_페그_자세로_제한높이까지_상승후_320에서_방향을_바꾼다(scene, sol, raise_z):
    scene.sol = sol
    scene.tcp[4] = 90
    scene.joints = scene.make_joints(scene.tcp)
    motion = {'work_orientation_raise_z_mm': 250, 'work_orientation_raise_z_right_mm': 245,
              'work_orientation_pivot_z_mm': 320}
    result = MP._transfer_above_floor([600, 0, 224, 0, 180, 0], motion, execute=False)
    assert result['task_plan'] == [
        [400, 0, raise_z, 0, 90, 0], [400, 0, 320, 0, 180, 0],
        [600, 0, 224, 0, 180, 0], [600, 0, 224, 0, 180, 0]]
    assert not scene.calls


@pytest.mark.parametrize('bad_step', [0, 1, 2])
def test_높이이동의_어느목표든_J5한계초과면_첫명령전_차단한다(scene, monkeypatch, bad_step):
    tasks = []

    def ikin(task, sol):
        q = scene.make_joints(task)
        if len(tasks) == bad_step:
            q[4] = -137.03
        tasks.append(task)
        return q
    monkeypatch.setattr(MP._api, 'ikin', ikin)
    with pytest.raises(MP.MotionError, match='J5=-137.030'):
        MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {})
    assert not scene.calls


@pytest.mark.parametrize('key', ['work_orientation_raise_z_mm', 'work_orientation_pivot_z_mm'])
def test_자세변경_높이_NaN은_이동전에_거부한다(scene, key):
    scene.tcp[4] = 90
    with pytest.raises(MP.MotionError, match='유한한 양수'):
        MP._transfer_above_floor([600, 0, 224, 0, 180, 0], {key: float('nan')})
    assert not scene.calls


def test_실기도구가_실제_집기발현함수로_홈없이_한번_발현한다(scene, monkeypatch):
    from pathlib import Path
    from tunio_robot import teach_points as TP
    root = Path(__file__).resolve().parents[3]
    monkeypatch.syspath_prepend(str(root/'tools'))
    import work_motion_check as tool
    points = TP.load(root/'src/tunio_robot/config/teach_points.yaml')
    motion = dict(points.motion, pick_settle_sec=0, speed_scale=1.0)
    grips, strokes, questions = [], [], []
    monkeypatch.setattr(MP._gripper, 'select', grips.append)
    monkeypatch.setattr(MP, '_movel_tool_rel',
                        lambda task, v, a, **k: strokes.append((task, v, a)))

    def no_home(*args, **kwargs):
        raise AssertionError('집기·발현 중 홈을 호출하면 안 된다')
    monkeypatch.setattr(MP, '_goto_standby', no_home)
    monkeypatch.setattr(MP, 'return_home', no_home)
    result = tool.run(points, 'pick-pluck', 6, motion, execute=True, ask=questions.append)
    assert grips == ['open', 'close']
    assert len(questions) == 3
    assert len(strokes) == 2  # 스트로크 한 번 + 즉시 후퇴 한 번
    assert all((v, a) == (motion['pluck_vel_stroke'], motion['pluck_acc_stroke'])
               for _, v, a in strokes)
    assert scene.tcp[:3] == points.pluck(6).task[:2] + [224]
    # 높이 경유로 하강 거리가 늘어도 최종 접촉 좌표는 교시점 그대로다.
    assert ('movel', points.pluck(6).task) in scene.calls
    assert result['pick_state'] == 'held_assumed'


def test_현재가_복구로_높아져도_목표위는_J5가_허용하는_높이로_낮춘다(scene):
    """9/29: 취소 복구 +30 → 254.2 에서 재개. 줄 위 254 는 J5 -136, 224 는 통과."""
    scene.tcp = [591.8, -143.8, 254.2, 165.6, -180., -102.9]
    scene.joints = scene.make_joints(scene.tcp)
    target = [645.23, -19.14, 224., 31.81, 180., 124.62]
    base_ikin = MP._api.ikin

    def ikin(task, sol):
        q = base_ikin(task, sol)
        if abs(task[0] - target[0]) < 1e-6 and task[2] > 230:   # 줄 위 254 만 한계 초과
            q[4] = -136.183
        return q
    MP._api.ikin = ikin
    result = MP._transfer_above_floor(target, {}, execute=False)
    raised, high, final = result['task_plan']
    assert all(abs(q[4]) <= 135 for q in result['joint_plan'])
    # 현재 위는 그대로 254.2, 목표 위는 224~230 사이로 내려온다
    assert raised[2] == pytest.approx(254.2)
    assert 224 <= high[2] <= 230 and final == target


def test_목표위를_바닥까지_낮춰도_J5가_안되면_이동하지_않는다(scene):
    scene.tcp = [591.8, -143.8, 254.2, 165.6, -180., -102.9]
    scene.joints = scene.make_joints(scene.tcp)
    target = [645.23, -19.14, 224., 31.81, 180., 124.62]
    base_ikin = MP._api.ikin

    def ikin(task, sol):
        q = base_ikin(task, sol)
        if abs(task[0] - target[0]) < 1e-6 and task[2] > 223:
            q[4] = -136.183
        return q
    MP._api.ikin = ikin
    with pytest.raises(MP.MotionError, match="높이 이동 목표 위"):
        MP._transfer_above_floor(target, {}, execute=False)
    assert scene.calls == []


def test_목표위_IK가_엉뚱한_해를_주면_그_높이도_낮춘다(scene):
    """9/29: 줄 위 254 에서 IK/FK 위치 불일치. J5 가 아니라 대조 실패로도 내려와야 한다."""
    scene.tcp = [591.6, -143.8, 254.0, 165.5, -180., -103.1]
    scene.joints = scene.make_joints(scene.tcp)
    target = [645.85, -29.08, 224., 18.53, -180., 111.67]
    base_ikin = MP._api.ikin

    def ikin(task, sol):
        q = base_ikin(task, sol)
        if abs(task[0] - target[0]) < 1e-6 and task[2] > 230:   # fk 가 다른 자리를 가리킴
            q[3] += 3.0
        return q
    MP._api.ikin = ikin
    result = MP._transfer_above_floor(target, {}, execute=False)
    raised, high, final = result['task_plan']
    assert raised[2] == pytest.approx(254.0)
    assert 224 <= high[2] <= 230 and final == target
