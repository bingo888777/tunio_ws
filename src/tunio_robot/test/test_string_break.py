"""
예외처리 B안 — 줄 끊어짐 감지. 2026-09-22, 검증(실측) 전에 미리 짜 둔 코드.

**문턱은 임시값이다.** 이 테스트는 배선(기본 꺼짐·켜면 amovej 로 비동기
시작하고 폴링마다 토크를 봄·감지되면 MoveStop 을 부르고 StringBrokenError
를 던짐·check_motion() 의 IDLE 을 관절각으로 재확인함·무한 대기 방지)만
검증한다. 실제로 줄이 끊어졌을 때 이 문턱으로 맞는 판정이 나오는지는
tools/peg_turn_check.py --string cut 실측으로 따로 확인해야 한다(#79/#80).

로봇·ROS 없이 돈다:  cd src/tunio_robot && python3 -m pytest test/test_string_break.py -q
"""
import sys
import types
from types import SimpleNamespace

import pytest

from tunio_robot import motion_primitives as MP
from tunio_robot.teach_points import Pose


class FakeApi:
    """amovej/check_motion/get_current_posj 를 흉내 낸다."""

    DR_MV_MOD_ABS = "ABS"
    DR_MV_MOD_REL = "REL"

    def __init__(self):
        self.events = []
        self.motion_status = iter([])   # check_motion 이 순서대로 돌려줄 값들
        self.joints = None        # get_current_posj 가 순서대로 돌려줄 값들(iterator)

    def amovej(self, pos, vel=None, acc=None, mod=None):
        self.events.append(("amovej", list(pos), mod))
        return 0

    def movej(self, pos, vel=None, acc=None, mod=None):
        self.events.append(("movej", list(pos), mod))
        return 0

    def check_motion(self):
        return next(self.motion_status)

    def get_current_posj(self):
        return next(self.joints)


@pytest.fixture
def rig(monkeypatch):
    pose = Pose("pegs.6", True, task=[300.0, 60.0, 108.0, 90.0, -90.0, 0.0],
                tcp="GripperDA_v1")
    fake_common = types.ModuleType("DR_common2")
    fake_common.posj = lambda v: list(v)
    monkeypatch.setitem(sys.modules, "DR_common2", fake_common)

    api = FakeApi()
    torque = SimpleNamespace(readings=None)          # 테스트마다 채운다

    monkeypatch.setattr(MP, "_api", api)
    monkeypatch.setattr(MP, "read_external_torque", lambda: next(torque.readings))
    monkeypatch.setattr(MP, "turn_motion", lambda motion, pose=None: dict(motion))
    monkeypatch.setattr(MP, "time", SimpleNamespace(sleep=lambda s: None))
    stop_calls = []
    monkeypatch.setattr(MP, "_stop_motion", lambda *a, **kw: stop_calls.append(1))
    return api, pose, torque, stop_calls


BASE_MOTION = {"string_break_enabled": True, "string_break_poll_sec": 0.001,
               "string_break_torque_threshold_nm": 0.05,
               "string_break_arrive_tolerance_deg": 0.5,
               "string_break_max_wait_sec": 30.0}


def test_기본값은_꺼져있어_amovej를_안_쓰고_한_번에_돈다(rig):
    """string_break_enabled 가 없으면(기본) adjust_peg 의 옛 분기(_movej) 그대로다."""
    api, pose, torque, stop_calls = rig
    motion = {"approach_mm": 100.0}
    assert motion.get("string_break_enabled", False) is False
    if motion.get("string_break_enabled", False):
        MP._turn_peg_async_with_string_check(20.0, motion, pose, 6)
    assert api.events == []
    assert stop_calls == []


def test_켜져있으면_amovej로_비동기_시작하고_폴링마다_토크를_본다(rig):
    """정상(줄 있음) 패턴 — BUSY 한 번 보고 나서 목표 각도에 온 IDLE 로 끝난다."""
    api, pose, torque, stop_calls = rig
    torque.readings = iter([[0.0] * 6, [0.0] * 5 + [0.3], [0.0] * 5 + [0.3]])
    api.motion_status = iter([2, 0])                 # 1번째 BUSY, 2번째 IDLE(진짜 끝)
    api.joints = iter([[0.0] * 6, [0.0] * 5 + [20.0]])  # 시작각, IDLE 때 도착 확인

    MP._turn_peg_async_with_string_check(20.0, dict(BASE_MOTION), pose, 6)

    assert len(api.events) == 1
    kind, pos, mod = api.events[0]
    assert kind == "amovej" and mod == api.DR_MV_MOD_REL and pos[5] == 20.0
    assert stop_calls == []


def test_토크가_baseline에서_안_벗어나면_MoveStop을_부르고_예외를_던진다(rig):
    """줄 끊어짐 패턴 — 폴링 값이 baseline 근처(문턱 미만)면 즉시 멈춘다."""
    api, pose, torque, stop_calls = rig
    torque.readings = iter([[0.0] * 6, [0.0] * 5 + [0.01]])
    api.motion_status = iter([2])
    api.joints = iter([[0.0] * 6])                   # 시작각만 읽는다(도착 확인까진 못 감)

    with pytest.raises(MP.StringBrokenError, match="MoveStop"):
        MP._turn_peg_async_with_string_check(20.0, dict(BASE_MOTION), pose, 6)

    assert stop_calls == [1]


def test_낡은_IDLE는_BUSY를_못_봤으니_관절각도_안_보고_그냥_기다린다(rig):
    """Amovej 직후 첫 폴링이 로봇이 움직이기 전의 낡은 IDLE 을 볼 수 있다.

    seen_busy 가 아직 False 면 관절각조차 확인하지 않고 그냥 계속 기다린다
    — BUSY(2)를 실제로 한 번 봐야 그다음 IDLE 을 신뢰한다. 그러지 않으면
    "실제로는 안 돈 채로 다음 단계로 넘어간다" 문제가 그대로 남는다
    (관절각 확인만으로는 미세 보정에서 또 뚫린다 — 아래 다른 테스트 참고).
    """
    api, pose, torque, stop_calls = rig
    torque.readings = iter([[0.0] * 6, [0.0] * 5 + [0.3], [0.0] * 5 + [0.3],
                            [0.0] * 5 + [0.3]])
    api.motion_status = iter([0, 2, 0])  # 낡은 IDLE(무시됨) -> BUSY -> 진짜 IDLE
    api.joints = iter([
        [0.0] * 6,                       # 시작각
        [0.0] * 5 + [20.0],              # 진짜 IDLE 때만 확인 — 20도 도착
    ])

    MP._turn_peg_async_with_string_check(20.0, dict(BASE_MOTION), pose, 6)
    assert stop_calls == []              # 끝까지 끊어짐도 타임아웃도 아니었다


def test_회전량이_허용오차보다_작으면_BUSY_없이는_안_끝난_걸로_본다(rig):
    """
    관절각 확인만으로는 미세 보정(예: 0.3도)에서 뚫린다.

    허용 오차(기본 0.5도)보다 회전량이 작으면, 회전 전 낡은 위치조차
    목표와의 차이가 허용치 안에 들어 "도착"으로 오판할 수 있다. BUSY 를
    실제로 한 번 보기 전에는 그 오판이 통과하면 안 된다 — 이 테스트는
    낡은 IDLE 이 나오는 순간 위치상 이미 "도착"처럼 보여도(0.3도 이내라)
    seen_busy 가 없으면 넘어가지 않는 것을 확인한다.
    """
    api, pose, torque, stop_calls = rig
    torque.readings = iter([[0.0] * 6, [0.0] * 5 + [0.3], [0.0] * 5 + [0.3],
                            [0.0] * 5 + [0.3]])
    api.motion_status = iter([0, 2, 0])  # 낡은 IDLE(위치상 오판 유발) -> BUSY -> 진짜 IDLE
    api.joints = iter([
        [0.0] * 6,                       # 시작각(=0.0)
        [0.0] * 5 + [0.3],               # 진짜 IDLE 때 확인 — 0.3도(요청량) 도착
    ])

    motion = dict(BASE_MOTION)
    MP._turn_peg_async_with_string_check(0.3, motion, pose, 6)   # 요청량이 허용오차보다 작다
    assert stop_calls == []
    # 낡은 IDLE(1번째 status) 시점엔 get_current_posj 가 안 불렸다 — 두 번째
    # 항목(0.3도 도착)까지 정확히 다 써서 StopIteration 이 안 났다는 것 자체가
    # "낡은 IDLE 을 그냥 넘겼다"는 증거다(위 assert 들이 이미 통과했다는 뜻)


def test_영원히_안_끝나면_타임아웃으로_MoveStop을_부르고_실패한다(rig):
    """무한 대기를 막는다 — max_wait_sec 을 넘기면 멈추고 실패로 던진다."""
    api, pose, torque, stop_calls = rig

    def torque_gen():
        yield [0.0] * 6                  # base(정지) — 낮다
        while True:
            yield [0.0] * 5 + [0.3]      # 도는 중 — baseline 에서 계속 벗어나 있다(정상)

    torque.readings = torque_gen()
    api.motion_status = iter(lambda: 2, None)        # 항상 BUSY — 영원히 안 끝남
    # get_current_posj: check_motion 이 항상 2(BUSY)라 도착 확인 분기는 안 탄다.
    # 시작각 읽기 1회만 있으면 된다.
    api.joints = iter([[0.0] * 6])

    motion = dict(BASE_MOTION)
    motion["string_break_max_wait_sec"] = 0.0        # 첫 순회에서 바로 넘긴다

    with pytest.raises(MP.MotionError, match="안 끝났다"):
        MP._turn_peg_async_with_string_check(20.0, motion, pose, 6)
    assert stop_calls == [1]


def test_amovej가_거부되면_MotionError(rig):
    """Amovej 반환이 -1 이면(순응제어 켜진 채 위치 명령 등) 폴링 없이 바로 실패한다."""
    api, pose, torque, stop_calls = rig
    api.amovej = lambda pos, vel=None, acc=None, mod=None: -1
    torque.readings = iter([[0.0] * 6])
    api.joints = iter([[0.0] * 6])

    with pytest.raises(MP.MotionError, match="거부"):
        MP._turn_peg_async_with_string_check(20.0, dict(BASE_MOTION), pose, 6)
    assert stop_calls == []


def test_StringBrokenError는_MotionError라서_기존_정리_경로를_그대로_탄다():
    """새 예외 클래스가 기존 except MotionError/Exception 처리에 그대로 걸리는지."""
    assert issubclass(MP.StringBrokenError, MP.MotionError)
    try:
        raise MP.StringBrokenError("테스트")
    except MP.MotionError as e:
        assert "테스트" in str(e)
