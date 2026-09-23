"""
페그 손잡이 각 기억 단위 테스트 (이슈 #31). 로봇 없이 돈다.

지키려는 것: **직전에 돌린 만큼 돌아간 상태로 들어가 잡는다.**
교시 자세 그대로 들어가면 손가락이 손잡이와 어긋난 채로 물리고, 여유 파지라
페그가 그리퍼 쪽으로 끌려오면서 **직전 회전이 그대로 롤백된다.**
"""
import os
from types import SimpleNamespace

import pytest
from tunio_robot import kinematics as K
from tunio_robot import peg_state as PS


# ----------------------------------------------------------- 각도 접기
def test_180도_대칭이라_접힌다():
    """손잡이는 납작한 막대다. θ 와 θ+180 은 같은 자세다."""
    assert PS.fold(190.0) == pytest.approx(10.0)
    assert PS.fold(-190.0) == pytest.approx(-10.0)
    assert PS.fold(90.0) == pytest.approx(-90.0)     # 경계는 아래쪽으로
    assert PS.fold(0.0) == pytest.approx(0.0)


def test_접은_값은_손목이_덜_돈다():
    for raw in (170.0, 350.0, -260.0, 1000.0):
        assert -90.0 <= PS.fold(raw) < 90.0


# ----------------------------------------------------------- 등록·누적
def _state(tmp_path, now=1000.0):
    clock = {"t": now}
    st = PS.PegState(path=str(tmp_path / "peg_state.json"),
                     max_age_sec=100.0, clock=lambda: clock["t"])
    return st, clock


def test_등록_전에는_각을_안_준다(tmp_path):
    """None 이면 '모른다' 다. 0 으로 넘기면 교시 자세로 들어가 헛문다."""
    st, _ = _state(tmp_path)
    assert st.roll_of(6) is None
    assert st.registered() == []


def test_등록하면_읽힌다(tmp_path):
    st, _ = _state(tmp_path)
    st.register(6, 37.0)
    assert st.roll_of(6) == pytest.approx(37.0)
    assert st.registered() == [6]


def test_돌린_만큼_누적된다(tmp_path):
    """이게 핵심이다. 30도씩 두 번 돌리면 손잡이는 60도 돌아가 있다."""
    st, _ = _state(tmp_path)
    st.register(6, 0.0)
    st.add_turn(6, 30.0)
    st.add_turn(6, 30.0)
    assert st.roll_of(6) == pytest.approx(60.0)


def test_누적도_접힌다(tmp_path):
    st, _ = _state(tmp_path)
    st.register(6, 80.0)
    st.add_turn(6, 30.0)                              # 110 -> -70
    assert st.roll_of(6) == pytest.approx(-70.0)


def test_등록_안_된_줄에는_누적하지_않는다(tmp_path):
    """기준 없이 0 에서 누적하면 등록된 것처럼 보여 더 위험하다."""
    st, _ = _state(tmp_path)
    with pytest.raises(KeyError):
        st.add_turn(6, 30.0)


def test_오래되면_모르는_것으로_본다(tmp_path):
    """사람이 그사이 손으로 돌렸을 수 있다. 낡은 값은 거짓말이 된다."""
    st, clock = _state(tmp_path)
    st.register(6, 40.0)
    clock["t"] += 99.0
    assert st.roll_of(6) == pytest.approx(40.0)
    clock["t"] += 2.0                                 # max_age_sec=100 초과
    assert st.roll_of(6) is None
    assert st.is_stale(6)


def test_누적하면_시각도_새로_찍힌다(tmp_path):
    st, clock = _state(tmp_path)
    st.register(6, 0.0)
    clock["t"] += 99.0
    st.add_turn(6, 10.0)
    clock["t"] += 99.0
    assert st.roll_of(6) is not None                  # 갱신됐으니 아직 안 낡았다


def test_잊으면_사라진다(tmp_path):
    st, _ = _state(tmp_path)
    st.register(6, 10.0)
    st.register(5, 20.0)
    st.forget(6)
    assert st.roll_of(6) is None and st.roll_of(5) is not None
    st.forget()
    assert st.registered() == []


# ----------------------------------------------------------- 파일
def test_저장했다_읽으면_같다(tmp_path):
    st, clock = _state(tmp_path)
    st.register(6, 37.0)
    st.register(1, -12.5)
    st.save()

    again = PS.PegState(path=st.path, max_age_sec=100.0, clock=lambda: clock["t"])
    again.load()
    assert again.roll_of(6) == pytest.approx(37.0)
    assert again.roll_of(1) == pytest.approx(-12.5)


def test_파일이_없으면_빈_상태다(tmp_path):
    """그러면 Phase 0 를 돈다. 없는 걸 있다고 하는 것보다 낫다."""
    st = PS.PegState(path=str(tmp_path / "없는파일.json"))
    st.load()
    assert st.registered() == []


def test_파일이_깨져도_안_터진다(tmp_path):
    p = tmp_path / "peg_state.json"
    p.write_text("{망가진 json", encoding="utf-8")
    st = PS.PegState(path=str(p))
    st.load()
    assert st.registered() == []


def test_한_줄이_깨져도_나머지는_살린다(tmp_path):
    p = tmp_path / "peg_state.json"
    p.write_text('{"pegs": {"6": {"roll_deg": 20, "at": 1000},'
                 ' "5": {"roll_deg": "이상함"}}}', encoding="utf-8")
    st = PS.PegState(path=str(p), max_age_sec=1e9, clock=lambda: 1000.0)
    st.load()
    assert st.roll_of(6) == pytest.approx(20.0)
    assert st.roll_of(5) is None


def test_저장은_임시파일을_거친다(tmp_path):
    """쓰다 죽어도 이전 파일이 남아야 한다."""
    st, _ = _state(tmp_path)
    st.register(6, 1.0)
    st.save()
    assert os.path.exists(st.path)
    assert not os.path.exists(st.path + ".tmp")


# ----------------------------------------------------------- 자세에 반영
def test_손잡이_각은_C_에만_더해진다():
    """툴 Z 둘레 회전이라 손끝 위치도 접근축도 안 바뀐다."""
    peg = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]
    turned = K.roll_about_tool_z(peg, 37.0)
    assert turned[:3] == peg[:3]                      # 위치 그대로
    assert turned[3:5] == peg[3:5]                    # A·B 그대로
    assert turned[5] == pytest.approx(peg[5] + 37.0)
    assert K.tool_z_axis(turned) == pytest.approx(K.tool_z_axis(peg), abs=1e-12)


def test_0도면_그대로():
    peg = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]
    assert K.roll_about_tool_z(peg, 0.0) == pytest.approx(peg)


def test_접근축이_안_바뀌니_경로도_같다():
    """backed_off 가 접근축으로 물러나므로, 손잡이를 돌려도 물러나는 점은 같다."""
    from tunio_robot.teach_points import backed_off
    peg = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]
    a = backed_off(peg, 100.0)
    b = backed_off(K.roll_about_tool_z(peg, 55.0), 100.0)
    assert a[:3] == pytest.approx(b[:3], abs=1e-9)


# ─────────────── adjust_peg 가 실제로 그 각으로 들어가는가 (#31) ───────────────
#
# 이게 이 이슈의 본론이다. 조금 돌리고 다시 더 돌리러 갈 때, 교시 자세 그대로
# 들어가면 손가락이 손잡이와 어긋난 채로 물린다. 여유 파지(21mm)라 페그가
# 그리퍼 쪽으로 끌려오면서 **직전에 돌린 것이 그대로 풀린다.**

class _Peg:
    """teach_points.Pose 흉내. 작업 좌표 페그 하나."""

    kind = "task"
    taught = True
    tcp = "GripperDA_v1"
    sol = 7
    name = "pegs.6"

    def __init__(self, task):
        self.task = list(task)


@pytest.fixture
def spy(monkeypatch):
    """MP 의 저수준 동작을 기록만 하도록 바꾼다. 로봇을 안 부른다."""
    from tunio_robot import motion_primitives as MP
    log = []
    monkeypatch.setattr(MP, "_api", SimpleNamespace(get_current_posj=lambda: [0.0] * 6))
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: None)
    monkeypatch.setattr(MP, "_grip", lambda name: log.append(("grip", name)))
    monkeypatch.setattr(MP, "_release", lambda: log.append(("release",)))
    monkeypatch.setattr(MP, "_movejx", lambda t, m, sol=None: log.append(("movejx", list(t))))
    monkeypatch.setattr(MP, "_movel", lambda t, m: log.append(("movel", list(t))))
    monkeypatch.setattr(MP, "_movej",
                        lambda j, m, relative=False: log.append(("movej", list(j), relative)))
    return MP, log


PEG6 = [342.32, -75.38, 113.49, 86.40, 89.50, 1.11]
TAUGHT_C = PEG6[5]


def _entry_c(log):
    """페그 앞까지 직선으로 들어간 자세의 C (손목 롤)."""
    return next(t[5] for kind, t in ((e[0], e[1]) for e in log if e[0] == "movel"))


def test_등록된_각으로_들어간다(spy):
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 6, -10.0, {}, roll_deg=40.0)
    assert _entry_c(log) == pytest.approx(TAUGHT_C + 40.0)


def test_각을_안_주면_교시_자세_그대로(spy):
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 6, -10.0, {})
    assert _entry_c(log) == pytest.approx(TAUGHT_C)


def test_두_번째_진입이_직전_회전을_되돌리지_않는다(spy, tmp_path):
    """
    이 이슈의 본론. 30도 돌리고 또 30도 돌리러 갈 때를 그대로 재현한다.

    1회차는 교시 자세로 들어가고, 2회차는 **30도 돌아간 자세**로 들어가야 한다.
    2회차도 교시 자세로 들어가면 그게 롤백이다.
    """
    MP, log = spy
    st, _ = _state(tmp_path)
    st.register(6, 0.0)

    MP.adjust_peg(_Peg(PEG6), 6, -30.0, {}, roll_deg=st.roll_of(6))
    st.add_turn(6, -30.0)
    first = _entry_c(log)

    log.clear()
    MP.adjust_peg(_Peg(PEG6), 6, -30.0, {}, roll_deg=st.roll_of(6))
    st.add_turn(6, -30.0)
    second = _entry_c(log)

    assert first == pytest.approx(TAUGHT_C)
    assert second == pytest.approx(TAUGHT_C - 30.0)    # 롤백이면 TAUGHT_C 가 나온다
    assert st.roll_of(6) == pytest.approx(-60.0)


def test_다_돌면_돌아간_자세로_나온다(spy):
    """
    놓고 물러날 때 원래 자세로 되돌리면 손목이 역회전한다.

    그리퍼를 열어도(30mm) 페그가 아직 손가락 사이라 다시 돌려 버릴 수 있다.
    """
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 6, -30.0, {}, roll_deg=40.0)
    retreat = [e for e in log if e[0] == "movel"][-1]
    assert retreat[1][5] == pytest.approx(TAUGHT_C + 40.0 - 30.0)


def test_돌다가_실패하면_현재_자세로_나온다(spy, monkeypatch):
    """부분 회전 후 교시 방향으로 되돌리지 않는다 (#70)."""
    MP, log = spy
    current = list(PEG6)
    current[5] += 25.0                             # 40도에서 -15도만 회전하고 실패
    monkeypatch.setattr(MP, "read_posx", lambda: (current, 7))

    def boom(*a, **k):
        raise MP.MotionError("회전 실패")
    monkeypatch.setattr(MP, "_movej", boom)

    with pytest.raises(MP.MotionError):
        MP.adjust_peg(_Peg(PEG6), 6, -30.0, {}, roll_deg=40.0)
    assert [e for e in log if e[0] == "release"]        # 실패해도 놓는다
    retreat = [e for e in log if e[0] == "movel"][-1]
    assert retreat[1][5] == pytest.approx(TAUGHT_C + 25.0)


def test_들어가기_전에_연다(spy):
    """
    직전 동작이 피크를 물고 있었으면 15mm 인 채로 들어가 손잡이를 들이받는다.

    30mm 는 손잡이에 안 닿는 폭이다(실물 확인). 더 벌리면 옆 페그를 친다.
    """
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 6, -10.0, {}, roll_deg=0.0)
    grips = [e for e in log if e[0] == "grip"]
    assert grips[0] == ("grip", "open")              # 이동 전에 연다
    assert grips[1] == ("grip", "peg")
    # 여는 동작이 첫 이동보다 앞에 있어야 한다
    assert log.index(("grip", "open")) < next(i for i, e in enumerate(log)
                                              if e[0] in ("movejx", "movel"))


def test_나갈_때도_30mm_로_연다(spy):
    """후퇴 중에도 옆 페그를 안 쳐야 한다. _release() 가 open 프리셋을 쓴다."""
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 6, -10.0, {}, roll_deg=0.0)
    assert ("release",) in log


# ───────────── 각 추적은 J6 전용이다 (PR #36 리뷰, 정동휘) ─────────────
#
# peg_joint 는 아직 "실물 확인 후 확정" 이라 YAML 검증이 1~6 을 다 받는다.
# J6 만 툴 Z 둘레 회전이라 관절 회전량이 곧 posx 의 C 변화량이다. 다른 관절이면
# 그 대응이 깨지는데, 예전에는 exit_ 만 조용히 옛 동작으로 떨어졌다 —
# 파지에 성공하고 성공으로 보고하면서 틀리는, #31 그 자체였다.

def test_J6_아니면_각_추적을_거부한다(spy):
    MP, _ = spy
    with pytest.raises(MP.MotionError, match="peg_joint"):
        MP.adjust_peg(_Peg(PEG6), 5, -30.0, {}, roll_deg=40.0)


def test_J6_아니어도_각_없이는_돈다(spy):
    """각 추적을 안 쓰면 다른 관절로 돌리는 것 자체는 막을 이유가 없다."""
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 5, -30.0, {}, roll_deg=0.0)
    assert any(e[0] == "movej" and e[2] for e in log)


def test_J6_아니면_나갈_때_자세를_안_바꾼다(spy):
    """J5 를 돌려도 C 는 그대로다. C 에 더하면 손목이 엉뚱하게 돈다."""
    MP, log = spy
    MP.adjust_peg(_Peg(PEG6), 5, -30.0, {}, roll_deg=0.0)
    retreat = [e for e in log if e[0] == "movel"][-1]
    assert retreat[1][5] == pytest.approx(TAUGHT_C)


def test_거부되면_움직이지_않는다(spy):
    MP, log = spy
    with pytest.raises(MP.MotionError):
        MP.adjust_peg(_Peg(PEG6), 5, -30.0, {}, roll_deg=40.0)
    assert not [e for e in log if e[0] in ("movejx", "movel", "movej")]


@pytest.mark.parametrize("no,expected", [
    (6, (3.0, 6.0)), (5, (3.0, 6.0)), (2, (3.0, 6.0)), (1, (2.0, 4.0)),
])
def test_줄별_속도는_페그_회전에만_적용된다(spy, monkeypatch, no, expected):
    from tunio_robot import teach_points as TP
    MP, _ = spy
    points = TP.load(os.path.join(os.path.dirname(__file__), "..", "config", "teach_points.yaml"))
    # 이 테스트는 줄별 회전 속도값만 본다 — 줄끊어짐 감지(비동기 amovej 경로)는
    # test_string_break.py 몫이라 여기선 꺼서 기존 동기 _movej 경로로 보낸다.
    points.motion["string_break_enabled"] = False
    before = dict(points.motion)
    calls = []
    monkeypatch.setattr(MP, "_movej", lambda j, m, relative=False:
                        calls.append((m["vel"], m["acc"], m["speed_scale"], relative)))
    MP.adjust_peg(points.peg(no), 6, -30, points.motion, roll_deg=20.0)
    assert calls == [(expected[0], expected[1], 1.0, True)]
    unwind = MP.turn_motion(points.motion)
    assert (unwind["vel"], unwind["acc"]) == (2.0, 4.0)
    assert points.motion == before
