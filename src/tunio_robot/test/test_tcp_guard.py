"""
활성 TCP·툴 확인 단위 테스트 (이슈 #23·#26). 로봇 없이 돈다.

지키려는 것: 교시 좌표와 다른 기준으로는 **움직이지 않는다.**
페그 좌표가 GripperDA_v1(툴 Z 208mm) 기준이라, TCP 가 안 켜진 채로 움직이면
208mm 어긋난 곳으로 직선 진입한다. 헤드스톡 쪽이다.
"""
import pytest
from tunio_robot import motion_primitives as MP


class FakeApi:
    """
    두산 API 흉내. set_* 가 성공할지 미리 정한다.

    두산 원본은 실패를 예외가 아니라 -1 로 돌려준다 (DSR_ROBOT2.py 6332행 부근).
    그래서 반환값을 안 보면 조용히 틀린다.
    """

    def __init__(self, tool="", tcp="", allow_set=True):
        self.tool, self.tcp, self.allow_set = tool, tcp, allow_set
        self.calls = []

    def get_tool(self):
        return self.tool

    def get_tcp(self):
        return self.tcp

    def set_tool(self, name):
        self.calls.append(("set_tool", name))
        if not self.allow_set:
            return -1
        self.tool = name
        return 0

    def set_tcp(self, name):
        self.calls.append(("set_tcp", name))
        if not self.allow_set:
            return -1
        self.tcp = name
        return 0


@pytest.fixture
def api(monkeypatch):
    a = FakeApi()
    monkeypatch.setattr(MP, "_api", a)
    monkeypatch.setattr(MP, "_robot_cfg", {"tcp": "GripperDA_v1", "tool": "Tool Weight"})
    return a


def test_안_켜져_있으면_켠다(api):
    MP._ensure_named("tcp", "GripperDA_v1")
    assert api.tcp == "GripperDA_v1"
    assert api.calls == [("set_tcp", "GripperDA_v1")]


def test_이미_켜져_있어도_다시_설정한다(api):
    """
    배포용이라 펜던트 설정에 기대지 않는다.

    설정이 안 된 로봇에 가져다 놓아도 같은 코드가 돌아야 하므로 set 을 생략하지
    않는다. 읽은 값이 맞아 보여도 그건 이 로봇이 우연히 그 상태인 것뿐이다.
    """
    api.tcp = "GripperDA_v1"
    MP._ensure_named("tcp", "GripperDA_v1")
    assert api.calls == [("set_tcp", "GripperDA_v1")]


def test_거부됐는데_이미_맞으면_통과(api):
    """2026-09-20 현장 상태 — set 계열이 전부 -1 인데 펜던트로는 켜져 있다."""
    api.allow_set = False
    api.tcp = "GripperDA_v1"
    MP._ensure_named("tcp", "GripperDA_v1")      # 예외가 안 난다


def test_거부되고_안_맞으면_멈춘다(api):
    """이게 이슈 #26 의 핵심이다. 예전에는 여기서 그냥 움직였다."""
    api.allow_set = False
    api.tcp = ""
    with pytest.raises(MP.MotionError) as e:
        MP._ensure_named("tcp", "GripperDA_v1")
    assert "GripperDA_v1" in str(e.value)


def test_다른_TCP_가_켜져_있어도_멈춘다(api):
    """빈 문자열만 위험한 게 아니다. 발현용 TCP 가 켜져 있어도 페그 좌표는 틀린다."""
    api.allow_set = False
    api.tcp = "PickTCP"
    with pytest.raises(MP.MotionError):
        MP._ensure_named("tcp", "GripperDA_v1")


def test_툴도_같은_규칙이다(api):
    api.allow_set = False
    with pytest.raises(MP.MotionError):
        MP._ensure_named("tool", "Tool Weight")


def test_이름이_비면_아무것도_안_한다(api):
    MP._ensure_named("tcp", None)
    MP._ensure_named("tcp", "")
    assert api.calls == []


def test_ensure_tcp_는_robot_cfg_를_기본값으로(api):
    MP.ensure_tcp()
    assert api.calls == [("set_tcp", "GripperDA_v1")]


def test_ensure_tcp_는_이름을_받을_수_있다(api):
    """발현·피크는 자기 TCP 를 켠다 (이슈 #21, 성현)."""
    MP.ensure_tcp("PickTCP")
    assert api.calls == [("set_tcp", "PickTCP")]


def test_adjust_peg_가_움직이기_전에_TCP_를_켠다(api, monkeypatch):
    """순서가 중요하다. 켜기가 goto 보다 **뒤**면 첫 직선 진입이 틀린 기준으로 나간다."""
    order = []
    monkeypatch.setattr(MP, "_goto_pose", lambda *a: order.append("goto"))
    monkeypatch.setattr(MP, "_grip", lambda *a: order.append("grip"))
    monkeypatch.setattr(MP, "_release", lambda *a: order.append("release"))
    monkeypatch.setattr(MP, "_retreat", lambda *a: order.append("retreat"))
    monkeypatch.setattr(MP, "_movej", lambda *a, **k: order.append("movej"))
    monkeypatch.setattr(MP, "ensure_tcp", lambda *a, **k: order.append("tcp"))

    MP.adjust_peg(object(), 6, -5.0, {})
    assert order[0] == "tcp"
    assert order[:3] == ["tcp", "goto", "grip"]


def test_TCP_가_거부되면_페그로_안_간다(api, monkeypatch):
    """확인이 실패하면 goto 까지 못 간다. 안 움직이는 쪽이 낫다."""
    api.allow_set = False
    moved = []
    monkeypatch.setattr(MP, "_goto_pose", lambda *a: moved.append("goto"))

    with pytest.raises(MP.MotionError):
        MP.adjust_peg(object(), 6, -5.0, {})
    assert moved == []


# ───────────────── 두산 래퍼의 함정 (DSR_ROBOT2.py 6283~6382행) ─────────────────
#
# get_tcp()/get_tool() 을 그대로 믿으면 안 되는 이유가 둘 있다. 실제 설치본을
# 읽고 확인한 것이라, 드라이버가 갱신되면 이 테스트가 먼저 알려준다.

class OddApi:
    """타입을 섞거나 예외를 던지는 래퍼 흉내."""

    def __init__(self, get_returns=None, get_raises=None, set_raises=None):
        self.get_returns, self.get_raises = get_returns, get_raises
        self.set_raises = set_raises
        self.calls = []

    def get_tcp(self):
        if self.get_raises:
            raise self.get_raises
        return self.get_returns

    def set_tcp(self, name):
        self.calls.append(name)
        if self.set_raises:
            raise self.set_raises
        return 0


def test_get_가_숫자를_돌려주면_못_읽은_것으로_본다(monkeypatch):
    """
    응답이 None 이면 -1(int)을 돌려준다. 문자열로 비교하면 조용히 안 맞음이 된다.

    -1 을 이름으로 오해하고 통과시키면 안 된다. 멈춰야 한다.
    """
    monkeypatch.setattr(MP, "_api", OddApi(get_returns=-1))
    assert MP._read_named("tcp") is None
    with pytest.raises(MP.MotionError):
        MP._ensure_named("tcp", "GripperDA_v1")


def test_set_이_예외를_던져도_MotionError_로_바꾼다(monkeypatch):
    """
    서비스가 예외로 실패하면 -1 이 아니라 UnboundLocalError 가 난다.

    except 절이 로그만 찍고 ret 을 안 채운 채 return ret 을 타기 때문이다.
    그대로 새어 나가면 부르는 쪽이 못 알아본다.
    """
    api = OddApi(get_returns="", set_raises=UnboundLocalError("ret"))
    monkeypatch.setattr(MP, "_api", api)
    with pytest.raises(MP.MotionError) as e:
        MP._ensure_named("tcp", "GripperDA_v1")
    assert api.calls == ["GripperDA_v1"]         # 시도는 했다
    assert "GripperDA_v1" in str(e.value)


def test_set_이_예외여도_결과가_맞으면_통과한다(monkeypatch):
    """판정은 set 이 아니라 get 이 한다. 예외가 나도 켜져 있으면 진행한다."""
    monkeypatch.setattr(MP, "_api", OddApi(get_returns="GripperDA_v1",
                                           set_raises=UnboundLocalError("ret")))
    MP._ensure_named("tcp", "GripperDA_v1")


def test_get_이_예외를_던지면_멈춘다(monkeypatch):
    monkeypatch.setattr(MP, "_api", OddApi(get_raises=RuntimeError("서비스 없음")))
    with pytest.raises(MP.MotionError, match="get_tcp"):
        MP._read_named("tcp")
