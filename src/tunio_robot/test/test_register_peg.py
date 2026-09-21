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
    "probe_preset": "close", "contact_force_n": 8.0, "insert_steps": 4,
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
    """순응제어 호출만 기록하는 가짜 두산 API."""

    def __init__(self):
        self.compliance = []

    def task_compliance_ctrl(self, stx):
        self.compliance.append(("on", list(stx)))
        return 0

    def release_compliance_ctrl(self):
        self.compliance.append(("off",))
        return 0


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
    monkeypatch.setattr(MP, "read_force", lambda: [0.0, 0.0, 0.0, 0, 0, 0])
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


def test_나눠서_들어간다(spy):
    """한 번에 직선으로 들어가면 도중에 걸렸는지 볼 지점이 없다."""
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    ins = [e for e in spy if e[0] == "movel"]
    assert len(ins) >= HANDLE["insert_steps"]


# ------------------------------------------------------------ 접촉 감지
def test_걸리면_멈추고_어느_줄인지_알린다(spy, monkeypatch):
    """
    열기 38mm 로는 59도쯤 선 손잡이(38.2mm 필요)를 못 지난다.

    폭을 키우면 옆 페그를 친다. 그래서 들이받기 전에 멈추고 사람에게 알린다.
    """
    monkeypatch.setattr(MP, "read_force", lambda: [9.0, 0.0, 0.0, 0, 0, 0])
    with pytest.raises(MP.MotionError) as e:
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert "pegs.6" in str(e.value)
    assert "눕힌" in str(e.value)


def test_걸리면_물러난다_밀지_않는다(spy, monkeypatch):
    monkeypatch.setattr(MP, "read_force", lambda: [9.0, 0.0, 0.0, 0, 0, 0])
    with pytest.raises(MP.MotionError):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    assert abs(spy[-1][1][1] - PEG6[1]) > 50.0       # 마지막 이동이 물러난 지점


def test_문턱_아래면_그냥_들어간다(spy, monkeypatch):
    monkeypatch.setattr(MP, "read_force", lambda: [7.9, 0.0, 0.0, 0, 0, 0])
    assert MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6)) == pytest.approx(0.0)


def test_힘을_못_읽어도_진행한다(spy, monkeypatch):
    """감시가 안 되는 건 아쉽지만, 그것 때문에 등록 자체를 막지는 않는다."""
    def boom():
        raise RuntimeError("센서 없음")
    monkeypatch.setattr(MP, "read_force", boom)
    assert MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6)) == pytest.approx(0.0)


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
