"""경유점 근처 조기 반환을 흉내 내고 종료·실패·취소 경계를 검사한다. 실물 연결 없음."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from tunio_robot import kinematics as K, motion_primitives as MP, teach_points as TP
from tunio_robot.motion_cancel import CancelSession, MotionCancelled
from tunio_robot.robot_controller import RobotController


@pytest.fixture
def rig(monkeypatch):
    session = CancelSession()
    sim = SimpleNamespace(q=[0., 0., 0., 0., 90., 0.], calls=[], reads=[], reject=None,
                          idle=0, missed=False, cancel=False, wait_ret=0)

    def movej(q, **kw):
        sim.calls.append((list(q), kw))
        if sim.reject == len(sim.calls):
            return -1  # 설치된 두산 이동 래퍼의 거부 반환값
        if kw['radius']:
            # radius>0은 아직 목표에 도착하지 않아도 반환한다.
            sim.q = [(a + b) / 2 for a, b in zip(sim.q, q)]
            if sim.cancel:
                session.requested.set()
        elif not sim.missed:
            sim.q = list(q)
        return 0

    def read():
        sim.reads.append(len(sim.calls))
        return list(sim.q)

    api = SimpleNamespace(movej=movej, get_current_posj=read, DR_MV_MOD_ABS=0,
                          DR_MV_RA_OVERRIDE=0, mwait=lambda t: sim.wait_ret,
                          check_motion=lambda: sim.idle)
    monkeypatch.setitem(sys.modules, 'DR_common2', SimpleNamespace(posj=list, posx=list))
    monkeypatch.setattr(MP, '_api', api)
    monkeypatch.setattr(MP, '_cancel_check', session.check)
    monkeypatch.setattr(MP, '_blend_in_flight', False)
    monkeypatch.setattr(MP, '_j6_motion', None)
    monkeypatch.setattr(K, 'fk', lambda q: {'tcp': [q[0] * 10, q[1], q[2]]})
    sim.session = session
    sim.motion = {'transit_tcp_vel': 100, 'transit_tcp_acc': 100, 'speed_scale': 2.5}
    sim.path = [[10., 0., 0., 0., 90., 0.], [30., 0., 0., 0., 90., 0.]]
    return sim


def test_설정별반경과_조기반환_최종정지_도착확인(rig):
    motion = TP.load(Path(__file__).parents[1] / 'config/teach_points.yaml').motion
    assert MP._blend_radius(motion) == 10
    assert MP._blend_radius(motion, compact=True) == 2
    MP._movej_path(rig.path, rig.motion, radius=10)
    assert [kw['radius'] for _, kw in rig.calls] == [10, 0]
    assert all(kw['vel'] == pytest.approx(25) for _, kw in rig.calls)  # 손끝 250mm/s
    assert rig.reads == [0, 2]  # 중간점에 아직 못 왔다고 실패하거나 서비스 조회로 지연하지 않음
    assert rig.q == rig.path[-1]
    assert not MP._blend_in_flight


@pytest.mark.parametrize('radius', [0, 2, 100])
def test_반경0은정지_작은반경은유지_큰반경은짧은구간에맞춰축소(rig, radius):
    MP._movej_path(rig.path, rig.motion, radius=radius)
    assert [kw['radius'] for _, kw in rig.calls] == [min(radius, 25), 0]
    assert not MP._blend_in_flight


@pytest.mark.parametrize('failure', ['reject', 'busy', 'wait', 'missed', 'cancel'])
def test_실패취소미도달은_성공으로넘기지않는다(rig, failure):
    if failure == 'reject':
        rig.reject = 2
    elif failure == 'busy':
        rig.idle = 2
    elif failure == 'wait':
        rig.wait_ret = -1
    elif failure == 'missed':
        rig.missed = True
    else:
        rig.cancel = True
    with pytest.raises((MP.MotionError, MotionCancelled)):
        MP._movej_path(rig.path, rig.motion, radius=2)
    if failure == 'cancel':
        assert len(rig.calls) == 1
    assert MP._blend_in_flight == (failure != 'missed')  # 종료 미확인이면 독립 정지 필요
    if MP._blend_in_flight:
        with pytest.raises(MP.MotionError, match='종료 미확인'):
            MP._grip('open')


@pytest.mark.parametrize('bad', [-1, float('nan'), float('inf')])
def test_반경과_뒷경유점도_첫이동전에검사한다(rig, bad):
    with pytest.raises(MP.MotionError):
        MP._movej_path(rig.path, rig.motion, radius=bad)
    rig.path[-1][0] = 241
    with pytest.raises(MP.MotionError, match='J1'):
        MP._movej_path(rig.path, rig.motion, radius=2)
    assert rig.calls == []


def test_단독도구와_단계확인은_정지경유를유지한다(rig, monkeypatch):
    MP._movej_path(rig.path, rig.motion, radius=10, confirm=lambda _: None)
    assert all(kw['radius'] == 0 for _, kw in rig.calls)
    monkeypatch.setattr(MP, '_cancel_check', None)
    assert MP._blend_radius({'transit_blend_radius_mm': 10}) == 0


@pytest.mark.parametrize('stop_error', [None, '응답 없음'])
def test_연속경유_오류는_정지후보고하고_정지실패면잠근다(rig, monkeypatch, stop_error):
    def stop_request(stop, check, session, timeout, interval):
        assert session.work_done.is_set()
        session.error = stop_error
        session.stop_done.set()
    stop = Mock(side_effect=stop_request)
    monkeypatch.setattr('tunio_robot.robot_controller.request_and_confirm_stop', stop)
    node = SimpleNamespace(_lock=threading.Lock(), _running=None, _cancel_sessions={},
                           _recovery_required=False, _recovering=False, _prepared_pluck=None,
                           dry_run=False, _stop_client=Mock(), _motion_client=Mock(),
                           _stop_timeout=.1, _stop_interval=.001,
                           _pick_fault=None, _pick_refill_pending=False,
                           get_logger=lambda: Mock(), _record=Mock())
    rig.reject = 2
    goal = Mock()
    res = RobotController._run(node, 'adjust_peg', goal, SimpleNamespace,
                               lambda: MP._movej_path(rig.path, rig.motion, radius=2))
    assert not res.success
    stop.assert_called_once()
    goal.abort.assert_called_once()
    assert node._recovery_required == bool(stop_error)
    assert MP._blend_in_flight == bool(stop_error)


@pytest.mark.parametrize('radius', [0, 2])
def test_접근경로의_별도상한은_정지연속_모두지킨다(rig, radius):
    MP._movej_path(rig.path, rig.motion, radius=radius, cap=12)
    assert all(kw['vel'] == 12 for _, kw in rig.calls)
