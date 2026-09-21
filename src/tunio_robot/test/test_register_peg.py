"""
Phase 0 페그 초기 등록 단위 테스트 (이슈 #31). 로봇·Modbus 없이 돈다.

지키려는 것 —
  · 들이받기 전에 멈춘다. 열기 폭 38mm 로는 서 있는 손잡이를 못 지난다
  · 걸리면 **어느 줄인지 알리고** 물러난다. 사람이 그 줄만 손대면 된다
  · 빈손이면 멈춘다. 물린 척하고 진행하면 서버가 조용히 틀린 각도를 계산한다
  · 순응은 켰으면 반드시 끈다. 켠 채로 다니면 이동이 흐물거린다
"""
import pytest
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP

# 2026-09-21 6번 줄 실측 (tools/peg_grip_probe.py)
HANDLE = {
    "measured": True, "requires_prealign": True,
    "thickness_mm": 19.6, "length_mm": 32.8, "empty_mm": 13.0,
    "probe_preset": "close", "insert_force_n": 3.0, "contact_force_n": 2.0,
    "arrive_tol_mm": 2.0, "insert_timeout_sec": 5.0,
    "thresholds_mm": {"empty_below": 16.3, "dead_above": 26.2},
}
MOTION = {"peg_joint": 6, "approach_mm": 100.0,
          "compliance_stx": [1000.0, 200.0, 1000.0, 50.0, 50.0, 50.0]}
PEG6 = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]


class _Peg:
    kind, taught, tcp, sol, name = "task", True, "GripperDA_v1", 7, "pegs.6"

    def __init__(self):
        self.task = list(PEG6)


class _Api:
    """순응제어·힘 제어 호출을 기록하는 가짜 두산 API."""

    DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z = 0, 1, 2
    DR_BASE, DR_MV_MOD_ABS, DR_FC_MOD_REL = 0, 0, 1
    DR_COND_NONE = -10000

    def __init__(self, blocked=False):
        self.compliance = []
        self.force = []
        self.blocked = blocked          # True 면 막혀서 목표에 영영 못 닿는다

    def task_compliance_ctrl(self, stx):
        self.compliance.append(("on", list(stx)))
        return 0

    def release_compliance_ctrl(self):
        self.compliance.append(("off",))
        return 0

    # 두산 시그니처를 그대로 흉내 낸다. 이름이 내장함수와 겹치는 건 원본 탓이다
    def set_desired_force(self, fd, dir, mod=None):        # noqa: A002
        self.force.append(("push", list(fd), list(dir)))
        return 0

    def release_force(self, time=0):
        self.force.append(("release",))
        return 0

    # **0 이 조건 충족, -1 이 미충족이다.** 두산 래퍼가 그렇게 돌려준다.
    # 진릿값으로 쓰면 뒤집힌다 — 2026-09-21 실물에서 그래서 진입을 안 했다
    def check_position_condition(self, axis, min=None, max=None,   # noqa: A002
                                 ref=None, mod=None):
        return -1 if self.blocked else 0        # 안 막혔으면 도달

    def check_force_condition(self, axis, max=None, min=None, ref=None):  # noqa: A002
        return 0 if self.blocked else -1


class _Log(list):
    """호출 기록 + 가짜 API 를 같이 들고 다닌다 (list 에는 속성을 못 붙인다)."""

    api = None


@pytest.fixture
def spy(monkeypatch):
    log, api = _Log(), _Api()
    monkeypatch.setattr(MP, "_api", api)
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda n: log.append(("grip", n)))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: log.append(("movejx", list(t))))
    monkeypatch.setattr(MP, "_movel", lambda t, m: log.append(("movel", list(t))))
    log.api = api
    return log


def _widths(*seq):
    it = iter(seq)
    return lambda: next(it)


# ------------------------------------------------------------ 정상
def test_물리면_0도를_돌려준다(spy):
    """사람이 눕혀 뒀으면 교시 자세 그대로 물린다."""
    assert MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6)) == pytest.approx(0.0)


def test_꽉_무는_프리셋으로_잰다(spy):
    """작업용 peg(21mm)는 손잡이(19.6)에 닿지도 않아 각이 안 잡힌다."""
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert ("grip", "close") in spy
    assert ("grip", "peg") not in spy


# ------------------------------------------------------------ 힘 제어 진입
def test_접근축으로_민다(spy):
    """
    네 페그 접근축이 전부 base ±Y 다. 지배축을 골라 그 방향으로만 힘을 준다.

    6번은 툴 Z 가 +Y 라 +Y 로 밀어야 한다. 부호가 뒤집히면 뒤로 간다.
    """
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    push = next(e for e in spy.api.force if e[0] == "push")
    fd, dir_ = push[1], push[2]
    assert dir_ == [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]      # Y 축만
    assert fd[1] == pytest.approx(HANDLE["insert_force_n"])


def test_미는_힘이_상한이다(spy):
    """기타가 몇 N 에서 밀리는지 몰라도, 이보다 세게는 못 민다."""
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    fd = next(e for e in spy.api.force if e[0] == "push")[1]
    assert max(abs(v) for v in fd) == pytest.approx(HANDLE["insert_force_n"])


def test_막히면_멈추고_어느_줄인지_알린다(spy):
    """
    열기 38mm 로는 59도쯤 선 손잡이(38.2mm 필요)를 못 지난다.

    폭을 키우면 옆 페그를 친다. 그래서 막히면 사람에게 알린다.
    """
    spy.api.blocked = True
    with pytest.raises(MP.MotionError) as e:
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert "pegs.6" in str(e.value)
    assert "눕힌" in str(e.value)


def test_막혀도_힘_제어를_푼다(spy):
    """켠 채로 두면 다음 동작이 전부 흐물거린다."""
    spy.api.blocked = True
    with pytest.raises(MP.MotionError):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert spy.api.force[-1] == ("release",)


def test_막히면_물러난다(spy):
    spy.api.blocked = True
    with pytest.raises(MP.MotionError):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert abs(spy[-1][1][1] - PEG6[1]) > 50.0


def test_상한이_미는_힘보다_낮다():
    """상한이 미는 힘 이상이면 막혀도 영영 안 걸린다."""
    assert HANDLE["contact_force_n"] < HANDLE["insert_force_n"]


# ------------------------------------------------------------ 폭 판정
def test_사점이면_돌리지_말고_멈춘다(spy):
    """돌려서 빠져나올 공간이 없다. 사람이 눕혀야 한다."""
    with pytest.raises(MP.MotionError) as e:
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(32.8))
    assert "눕힌" in str(e.value)


def test_빈손이면_멈춘다(spy):
    with pytest.raises(MP.MotionError, match="빈손"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(13.0))


def test_실패해도_놓고_순응을_끈다(spy):
    """페그를 문 채 두거나 순응을 켠 채 다니면 다음 동작이 전부 망가진다."""
    with pytest.raises(MP.MotionError):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(13.0))
    assert ("grip", "open") in spy
    assert spy.api.compliance[-1] == ("off",)


# ------------------------------------------------------------ 순응제어
def test_순응을_켜고_끈다(spy):
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert spy.api.compliance[0][0] == "on"
    assert spy.api.compliance[0][1] == MOTION["compliance_stx"]
    assert spy.api.compliance[-1] == ("off",)


def test_삽입축이_제일_무르다():
    """
    네 페그의 접근축이 전부 base ±Y 다(실측).

    걸렸을 때 물러나려면 그 축이 제일 물러야 한다. X·Z 보다 단단하면 기타를 민다.
    """
    x, y, z = MOTION["compliance_stx"][:3]
    assert y < x and y < z


# ------------------------------------------------------------ 폭 판정 함수
def test_폭_하나로_셋이_갈린다():
    t = HANDLE["thresholds_mm"]
    assert GW.classify(19.6, t) == "hold"
    assert GW.classify(32.8, t) == "dead"
    assert GW.classify(13.0, t) == "empty"


def test_문턱_경계():
    t = HANDLE["thresholds_mm"]
    assert GW.classify(16.2, t) == "empty"
    assert GW.classify(16.4, t) == "hold"
    assert GW.classify(26.1, t) == "hold"
    assert GW.classify(26.3, t) == "dead"


def test_폭이_없으면_판정_안_한다():
    with pytest.raises(GW.WidthError):
        GW.classify(None, HANDLE["thresholds_mm"])


def test_조건_반환은_0이_충족이다():
    """
    두산 래퍼가 `0 if success else -1` 로 돌려준다.

    파이썬에서 0 은 거짓, -1 은 참이라 진릿값으로 쓰면 정확히 뒤집힌다.
    2026-09-21 실물에서 진입이 곧바로 "도달" 로 끝나 로봇이 페그로 들어가지도
    않고 그리퍼만 여닫았다. 그 회귀를 막는다.
    """
    assert MP._cond_met(0) is True
    assert MP._cond_met(-1) is False
    assert bool(0) is False and bool(-1) is True     # 진릿값은 반대다
