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
    "arrive_tol_mm": 2.0, "insert_timeout_sec": 5.0, "search_mm": 25.0,
    "thresholds_mm": {"empty_below": 16.3, "dead_above": 26.2},
}
MOTION = {"peg_joint": 6, "approach_mm": 100.0,
          "compliance_stx": [1000.0, 200.0, 1000.0, 50.0, 50.0, 50.0]}
PEG6 = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]


class _Peg:
    kind, taught, tcp, sol, name = "task", True, "GripperDA_v1", 7, "pegs.6"

    def __init__(self):
        self.task = list(PEG6)


class _Log(list):
    """호출 기록 + 가짜 API 를 같이 들고 다닌다 (list 에는 속성을 못 붙인다)."""

    api = None


class _Api:
    """순응제어·힘 제어 호출을 기록하는 가짜 두산 API."""

    DR_AXIS_X, DR_AXIS_Y, DR_AXIS_Z = 0, 1, 2
    DR_BASE, DR_MV_MOD_ABS, DR_FC_MOD_REL = 0, 0, 1
    DR_COND_NONE = -10000

    def __init__(self, blocked=False):
        self.compliance = []
        self.force = []
        self.blocked = blocked          # True 면 막혀서 못 들어간다
        self.pushing = False            # set_desired_force 가 걸려 있나

    def task_compliance_ctrl(self, stx):
        self.compliance.append(("on", list(stx)))
        return 0

    def release_compliance_ctrl(self):
        self.compliance.append(("off",))
        return 0

    # 두산 시그니처를 그대로 흉내 낸다. 이름이 내장함수와 겹치는 건 원본 탓이다
    def set_desired_force(self, fd, dir, mod=None):        # noqa: A002
        self.force.append(("push", list(fd), list(dir)))
        self.pushing = True
        return 0

    def release_force(self, time=0):
        self.force.append(("release",))
        self.pushing = False
        return 0

    def get_current_posj(self):
        return [0.0] * 6


@pytest.fixture
def spy(monkeypatch):
    log, api = _Log(), _Api()
    monkeypatch.setattr(MP, "_api", api)
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda n: log.append(("grip", n)))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: log.append(("movejx", list(t))))
    monkeypatch.setattr(MP, "_movel", lambda t, m: log.append(("movel", list(t))))

    # 막혔으면 물러난 자리에 머무르고 힘이 오른다. 아니면 목표까지 들어간다
    def posx(ref=None):
        y = PEG6[1] - 100.0 if api.blocked else PEG6[1]
        return ([PEG6[0], y, PEG6[2], 0, 0, 0], 7)

    def force(ref=None):
        """
        실기와 같은 모양으로 낸다 (2026-09-21 6번 실측).

        전에는 막히면 **처음부터** 3.0N 을 냈다. 그러면 기준선을 읽는 첫 호출이
        이미 3.0 이라, 증가분으로 판정하는 코드가 영영 0 을 본다.
        실물은 이렇다 — 밀기 전에도 툴 무게 몫으로 1.2N 쯤 읽히고, 막혀서
        버티기 시작하면 그 위에 접촉력이 얹힌다.
        """
        base = 1.2                              # 안 밀어도 읽히는 몫
        if not api.pushing:
            return [0.0, base, 0.0, 0, 0, 0]
        return [0.0, base + (3.0 if api.blocked else 0.2), 0.0, 0, 0, 0]

    monkeypatch.setattr(MP, "read_posx", posx)
    monkeypatch.setattr(MP, "read_force", force)
    log.api = api
    return log


def _widths(*seq):
    it = iter(seq)
    return lambda: next(it)


def test_먼_구간은_순응을_켜기_전에_간다(monkeypatch):
    """
    순응을 켠 뒤에 위치 명령을 주면 안 간다 (2026-09-21 실기 회귀).

    스프링이 흡수해서 팔이 그 자리에 선다. search_mm 8 을 줬는데 힘 제어가
    99.8mm 를 가려 했고, 4N 으로 20초에 7.5mm 밖에 못 갔다. 먼 구간은
    딱딱할 때 끝내야 한다. 순서가 뒤집히면 여기서 걸린다.
    """
    order = []
    monkeypatch.setattr(MP, "_api", _Api())
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda n: order.append(f"grip:{n}"))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: order.append("movejx"))
    monkeypatch.setattr(MP, "_movel", lambda t, m: order.append("movel"))
    monkeypatch.setattr(MP, "_retreat", lambda p, m: order.append("retreat"))
    monkeypatch.setattr(MP, "compliance",
                        lambda m, on=True: order.append(f"compliance:{'on' if on else 'off'}"))
    monkeypatch.setattr(MP, "_insert_force_controlled",
                        lambda *a, **k: order.append("force"))

    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))

    assert order.index("movel") < order.index("compliance:on"), order
    assert order.index("compliance:on") < order.index("force"), order


# ------------------------------------------------------------ 힘 판정
def test_순응_스프링_반력만으로는_막혔다고_하지_않는다(monkeypatch):
    """
    2026-09-21 실기에서 **허공인데 2mm 만에 "막혔다"** 고 했다. 그 회귀 테스트다.

    순응제어 Y 강성이 200 N/m = 0.2 N/mm 다. 밀고 들어가면 스프링이 밀어내므로
    닿지 않아도 힘이 오른다 — 25mm 면 5N 이라 문턱 2N 을 한참 넘는다.
    거기에 밀기 전에도 읽히는 기준선(실기 1.2N)까지 얹히면 예산이 통째로 없어진다.

    그래서 판정은 **읽은값 - 기준선 - 스프링 몫**, 즉 순수 접촉력으로 한다.
    """
    log, api = _Log(), _Api()
    monkeypatch.setattr(MP, "_api", api)
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda n: log.append(("grip", n)))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: None)
    monkeypatch.setattr(MP, "_movel", lambda t, m: None)

    state = {"y": PEG6[1] - HANDLE["search_mm"]}          # 페그 앞 25mm 에서 출발

    def posx(ref=None):
        if api.pushing:                                    # 밀면 5mm 씩 들어간다
            state["y"] = min(PEG6[1], state["y"] + 5.0)
        return ([PEG6[0], state["y"], PEG6[2], 0, 0, 0], 7)

    def force(ref=None):
        base = 1.2
        gone = state["y"] - (PEG6[1] - HANDLE["search_mm"])
        spring = 0.2 * gone if api.pushing else 0.0        # 닿지 않아도 오른다
        return [0.0, base + spring, 0.0, 0, 0, 0]

    monkeypatch.setattr(MP, "read_posx", posx)
    monkeypatch.setattr(MP, "read_force", force)

    # 스프링 반력이 문턱(2.0N)을 넘는데도 끝까지 들어가야 한다
    assert 0.2 * HANDLE["search_mm"] > HANDLE["contact_force_n"]
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))


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


# ------------------------------------------------------------ 뒷정리가 사유를 덮지 않는다 (#52)
def test_후퇴가_실패해도_원래_사유가_남는다(spy, monkeypatch):
    """
    뒷정리에서 터진 예외가 원래 예외를 대체한다.

    하필 제일 알고 싶은 때 겹친다 — 순응이 안 풀리는 상황과 진입이 실패하는
    상황은 원인이 같아서 같이 일어난다. 진단이 "빈손이다" 에서 "movel 가
    거부됐다" 로 뒤바뀌면 사람이 엉뚱한 데를 본다.
    """
    def 거부(pose, motion):
        raise MP.MotionError("movel 가 거부됐다 (-1)")

    monkeypatch.setattr(MP, "_retreat", 거부)
    with pytest.raises(MP.MotionError, match="빈손"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(13.0))


def test_성공했는데_후퇴가_실패하면_그건_올린다(spy, monkeypatch):
    """덮을 원래 사유가 없다. 팔이 페그에 붙은 채인데 성공이라고 하면 안 된다."""
    def 거부(pose, motion):
        raise MP.MotionError("movel 가 거부됐다 (-1)")

    monkeypatch.setattr(MP, "_retreat", 거부)
    with pytest.raises(MP.MotionError, match="거부"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))


def test_순응이_안_풀리면_후퇴를_안_한다(spy):
    """
    순응이 켜진 채로는 위치 명령이 -1 을 돌려준다 (#31).

    그 상태로 후퇴를 부르면 그 -1 이 원래 사유를 덮는다. 아예 안 부르고 알린다.
    """
    spy.api.release_compliance_ctrl = lambda: -1
    with pytest.raises(MP.MotionError, match="빈손"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(13.0))
    assert not [m for m in spy if m[0] == "movel" and abs(m[1][1] - PEG6[1]) > 50.0]


def test_순응이_안_풀린_채_성공으로_끝내지_않는다(spy):
    """팔이 페그에 붙어 있고 순응이 켜진 상태다. 다음 동작이 거기서 출발한다."""
    spy.api.release_compliance_ctrl = lambda: -1
    with pytest.raises(MP.MotionError, match="순응제어가 안 풀렸다"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))


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


def test_힘_판정은_값을_직접_읽는다(spy):
    """
    check_force_condition 을 안 쓴다. 두 번 속았다 (2026-09-21).

      1. 반환이 `0 if success else -1` 이라 진릿값으로 쓰면 뒤집힌다
      2. `max=2.0` 은 "2N 을 넘었다" 가 아니라 **"힘이 2N 이하다"** 는 뜻이다.
         허공에서 늘 참이라, 로봇이 가만히 서 있는 채로 "막혔다" 고 보고했다

    그래서 get_tool_force 값을 직접 읽어 문턱과 견준다.
    """
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))       # 힘 0.2N -> 통과
    assert not hasattr(spy.api, "condition_calls")


def test_허공에서는_막혔다고_하지_않는다(spy):
    """이게 2026-09-21 실물 증상이다. 안 움직인 채로 '막혔다' 고 했다."""
    spy.api.blocked = False
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))       # 예외가 안 난다


def test_먼_구간은_위치_제어로_간다(spy):
    """
    힘 제어는 접촉 탐색용이다. 3N 으로 100mm 를 가면 느리기만 하고 얻는 게 없다.

    손잡이와 부딪힐 수 있는 건 페그 바로 앞뿐이라, 거기까지는 그냥 간다.
    """
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6))
    moves = [e for e in spy if e[0] == "movel"]
    assert moves, "위치 제어 구간이 없다"
    # 첫 movel 이 페그에서 search_mm 만큼 뒤여야 한다 (접근점 100mm 가 아니라)
    y = moves[0][1][1]
    assert abs(abs(y - PEG6[1]) - HANDLE["search_mm"]) < 1.0


def test_탐색_구간이_접근_거리보다_짧다():
    """같거나 길면 위치 제어 구간이 없어져 처음부터 힘 제어로 기어간다."""
    assert HANDLE["search_mm"] < MOTION["approach_mm"]


# ───────── 측정 세션용 — 힘 제어를 끈다 (#13, #8) ─────────
#
# 힘 제어는 **초기 등록에만** 필요하다. 손잡이가 어느 각인지 모르는 그 한 번뿐이고,
# 등록되고 나면 각을 알고 들어가므로 막힐 일이 없다. adjust_peg 는 원래부터 안 쓴다.
# 사람이 손잡이를 눕혀 놓고 지켜보는 측정 세션이면 등록에서도 뺄 수 있다.

def test_힘_제어를_끄면_그냥_들어간다(spy):
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6), use_force=False)
    assert spy.api.force == []                      # set_desired_force 를 안 불렀다
    assert [e for e in spy if e[0] == "movel"]      # 대신 직선으로 들어갔다


def test_힘_제어를_꺼도_순응은_켠다(spy):
    """
    둘은 다른 것이다 (PR #37 리뷰, 정동휘).

      힘 제어  "이 힘까지만 민다"   — 미검증이라 끌 수 있게 했다
      순응제어  "걸리면 물러난다"    — 끄면 로봇이 뻣뻣해져 **오히려 더 세게 민다**

    use_force 의 목적은 미검증 경로를 안 태우는 것이지 안전망을 걷는 게 아니다.
    """
    MP.register_peg(_Peg(), MOTION, HANDLE, _widths(19.6), use_force=False)
    assert spy.api.compliance[0][0] == "on"
    assert spy.api.compliance[-1] == ("off",)       # 끝나면 반드시 끈다


def test_힘_제어를_꺼도_폭은_본다(spy):
    """진입 방식과 파지 판정은 별개다. 빈손은 여전히 걸러야 한다."""
    with pytest.raises(MP.MotionError, match="빈손"):
        MP.register_peg(_Peg(), MOTION, HANDLE, _widths(13.0), use_force=False)


def test_폭도_끄면_물렸다고_본다(spy):
    """Modbus 가 안 붙는 PC 에서 측정만 할 때. 사람이 눈으로 본다는 전제다."""
    roll = MP.register_peg(_Peg(), MOTION, HANDLE, _widths(), use_force=False,
                           assume_grip=True)
    assert roll == pytest.approx(0.0)
    assert ("grip", "close") in spy                 # 물기는 한다 (각이 확정돼야 하니까)


def test_adjust_peg_는_힘_제어를_안_쓴다(spy, monkeypatch):
    """
    각을 아는 뒤로는 막힐 일이 없다. 조절 경로에는 힘 제어가 처음부터 없었다.

    측정 세션(#13)이 이걸 반복해서 돌린다.
    """
    monkeypatch.setattr(MP, "_goto_pose", lambda *a: None)
    monkeypatch.setattr(MP, "_release", lambda: None)
    monkeypatch.setattr(MP, "_retreat", lambda *a: None)
    monkeypatch.setattr(MP, "_movej", lambda *a, **k: None)
    MP.adjust_peg(_Peg(), 6, -30.0, MOTION, roll_deg=45.0)
    assert spy.api.force == []
    assert spy.api.compliance == []


# ------------------------------------------------------------ 이동 속도 배율 (#58)
def test_travel_속도는_직선과_회전_상한이_다르다():
    """
    travel_vel 은 [직선 mm/s, 회전 deg/s] 쌍이다 — 한 리스트에 단위가 둘이다.

    상한을 하나로 먹이면 회전이 직선 상한(250deg/s)까지 올라간다. 칸마다 다르게 자른다.
    """
    caps = (MP.SPEED_CAP_LINEAR, MP.SPEED_CAP_JOINT)
    out = MP._fast({"speed_scale": 10.0}, [50.0, 30.0], caps)
    assert out == [MP.SPEED_CAP_LINEAR, MP.SPEED_CAP_JOINT]   # 500 -> 250, 300 -> 60


def test_상한에_안_닿으면_배율_그대로다():
    """상한은 천장이지 목표가 아니다."""
    caps = (MP.SPEED_CAP_LINEAR, MP.SPEED_CAP_JOINT)
    assert MP._fast({"speed_scale": 2.0}, [50.0, 20.0], caps) == [100.0, 40.0]
    assert MP._fast({}, [50.0, 30.0], caps) == [50.0, 30.0]   # 배율 없으면 그대로


def test_페그_회전은_배율을_안_탄다(spy):
    """turn_motion 사본이 speed_scale 을 1.0 으로 못박는다. 장력을 거는 동작이다."""
    out = MP.turn_motion({"speed_scale": 10.0, "turn_vel": 2.0, "turn_acc": 4.0})
    assert (out["speed_scale"], out["vel"], out["acc"]) == (1.0, 2.0, 4.0)
