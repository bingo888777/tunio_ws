"""
Phase 0 페그 초기 등록 단위 테스트 (이슈 #31). 로봇·Modbus 없이 돈다.

지키려는 것 셋 —
  · 사점이면 **열고 나서** 돌린다. 문 채로 돌리면 옆힘으로 안전모드가 걸린다
  · 돌리는 각이 충분해야 한다. 조금 돌리면 카밍이 다시 사점으로 끌고 간다
  · 빈손이면 멈춘다. 물린 척하고 진행하면 조용히 틀린다
"""
import pytest
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP

# 2026-09-21 6번 줄 실측 (tools/peg_grip_probe.py)
HANDLE = {
    "measured": True,
    "thickness_mm": 19.6, "length_mm": 32.8, "empty_mm": 13.0,
    "probe_preset": "close", "retry_roll_deg": 45.0, "max_attempts": 3,
    "thresholds_mm": {"empty_below": 16.3, "dead_above": 26.2},
}
PEG6 = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]


class _Peg:
    kind, taught, tcp, sol, name = "task", True, "GripperDA_v1", 7, "pegs.6"

    def __init__(self):
        self.task = list(PEG6)


@pytest.fixture
def spy(monkeypatch):
    log = []
    monkeypatch.setattr(MP, "_api", object())
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda n: log.append(("grip", n)))
    monkeypatch.setattr(MP, "_release", lambda: log.append(("release",)))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: log.append(("movejx", list(t))))
    monkeypatch.setattr(MP, "_movel", lambda t, m: log.append(("movel", list(t))))
    monkeypatch.setattr(MP, "_movej",
                        lambda j, m, relative=False: log.append(("movej", list(j), relative)))
    return log


def _widths(*seq):
    """차례로 이 폭들을 돌려주는 가짜 측정기."""
    it = iter(seq)
    return lambda: next(it)


def test_한_번에_물리면_0도를_돌려준다(spy):
    roll = MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(19.6))
    assert roll == pytest.approx(0.0)


def test_사점이면_45도_돌려서_다시_문다(spy):
    """32.8 은 사점이다. 한 번 돌리고 다시 물어 19.6 이 나오면 45도가 답이다."""
    roll = MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 19.6))
    assert roll == pytest.approx(45.0)


def test_두_번_사점이면_90도까지_간다(spy):
    roll = MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 30.0, 19.6))
    assert roll == pytest.approx(90.0)


def test_돌리기_전에_반드시_연다(spy):
    """
    사점에서 멈춘 상태는 손가락이 손잡이 양 끝을 40N 으로 누르고 있는 상태다.

    그대로 돌리면 그게 안전모드 걸리는 조합이다. 열고 돌려야 한다.
    """
    MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 19.6))
    turn = next(i for i, e in enumerate(spy) if e[0] == "movej" and e[2])
    before = [e for e in spy[:turn] if e[0] == "grip"]
    assert before[-1] == ("grip", "open")        # 회전 직전이 '열기' 여야 한다


def test_돌리는_각이_분수령을_넘는다():
    """
    카밍은 p(θ) 를 줄이는 쪽으로 비튼다. p 최대는 대각선(약 59도)이다.

    사점(90도)에서 그 아래로 내려오려면 90-59 = 31도를 넘겨야 한다.
    덜 돌리면 카밍이 다시 90도로 끌고 가 영영 못 빠져나온다.
    """
    import math
    T, W = HANDLE["thickness_mm"], HANDLE["length_mm"]
    watershed = math.degrees(math.atan2(W, T))
    assert HANDLE["retry_roll_deg"] > 90.0 - watershed


def test_빈손이면_멈춘다(spy):
    """물린 척하고 진행하면 서버가 조용히 틀린 각도를 계산한다."""
    with pytest.raises(MP.MotionError, match="빈손"):
        MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(13.0))


def test_계속_사점이면_포기한다(spy):
    with pytest.raises(MP.MotionError, match="사점"):
        MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 32.8, 32.8))


def test_실패해도_놓고_물러난다(spy):
    """페그를 문 채로 두면 다음 동작이 전부 막힌다."""
    with pytest.raises(MP.MotionError):
        MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(13.0))
    assert ("release",) in spy
    assert spy[-1][0] == "movel"                 # 후퇴까지 했다


def test_꽉_무는_프리셋으로_잰다(spy):
    """
    작업용 peg(21mm)는 손잡이 두께(19.6)보다 넓어 닿지도 않는다.

    각이 안 잡히고 물림·빈손이 똑같이 읽힌다. probe 프리셋이어야 한다.
    """
    MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(19.6))
    assert ("grip", "close") in spy
    assert ("grip", "peg") not in spy


# ----------------------------------------------------------- 폭 판정
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


def test_돌리기_전에_물러난다(spy):
    """
    페그 앞에서 돌리면 벌어진 손가락이 옆 페그 손잡이를 휘두르며 친다.

    간격이 38mm 인데 이웃 손잡이가 16.4mm 뻗어와 실효 21.6mm 뿐이다.
    접근점까지 빠져서 돌고 다시 들어간다.
    """
    MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 19.6))
    turn = next(i for i, e in enumerate(spy) if e[0] == "movej" and e[2])
    moves = [e for e in spy[:turn] if e[0] == "movel"]
    # 회전 직전 이동이 페그(Y -75.38)가 아니라 물러난 지점이어야 한다
    assert abs(moves[-1][1][1] - PEG6[1]) > 50.0
    # 회전 뒤에는 다시 들어간다
    after = [e for e in spy[turn:] if e[0] == "movel"]
    assert abs(after[0][1][1] - PEG6[1]) < 1e-6


def test_다시_들어갈_때_돌린_자세다(spy):
    """돌려놓고 원래 자세로 들어가면 돌린 의미가 없다."""
    MP.register_peg(_Peg(), {"peg_joint": 6}, HANDLE, _widths(32.8, 19.6))
    turn = next(i for i, e in enumerate(spy) if e[0] == "movej" and e[2])
    entry = next(e for e in spy[turn:] if e[0] == "movel")
    assert entry[1][5] == pytest.approx(PEG6[5] + HANDLE["retry_roll_deg"])
