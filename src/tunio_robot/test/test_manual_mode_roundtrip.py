"""
tool/tcp 설정을 위한 MANUAL<->AUTONOMOUS 왕복 단위 테스트. 로봇 없이 돈다.

두산 컨트롤러는 set_tool/set_tcp/add_tcp 를 **MANUAL 모드에서만** 받아준다
(2026-09-23 `tools/tcp_diag.py --manual` 실기로 확정 — AUTONOMOUS 에서 셋 다 -1,
MANUAL 에서 셋 다 0). `ros2_control`이 브링업마다 로봇을 AUTONOMOUS 로 강제
전환해 두므로, connect() 는 `_ensure_tool_tcp` 안에서 MANUAL 로 잠깐 왕복한 뒤
tool/tcp 를 맞추고 AUTONOMOUS로 복귀한다. MANUAL에서 시작해도 같다.
"""
import pytest
from tunio_robot import motion_primitives as MP

MANUAL, AUTONOMOUS = 0, 1
CFG = {"tool": "Tool Weight", "tcp": "GripperDA_v1"}


class ModeApi:
    """모드 전환 + tool/tcp 설정을 함께 흉내 낸다. set_tool/tcp 는 MANUAL 에서만 먹는다."""

    ROBOT_MODE_MANUAL = MANUAL
    ROBOT_MODE_AUTONOMOUS = AUTONOMOUS

    def __init__(self, start_mode=AUTONOMOUS, tool="", tcp="",
                 allow_mode_switch=True, restore_fails=False):
        self.mode = start_mode
        self.tool, self.tcp = tool, tcp
        self.allow_mode_switch = allow_mode_switch
        self.restore_fails = restore_fails
        self.calls = []

    def get_robot_mode(self):
        return self.mode

    def set_robot_mode(self, want):
        self.calls.append(("set_robot_mode", want))
        if not self.allow_mode_switch:
            return -1
        if self.restore_fails and want == AUTONOMOUS:
            return -1                      # 되돌리기만 실패하게
        self.mode = want
        return 0

    def get_tool(self):
        return self.tool

    def get_tcp(self):
        return self.tcp

    def set_tool(self, name):
        self.calls.append(("set_tool", name))
        if self.mode != MANUAL:
            return -1
        self.tool = name
        return 0

    def set_tcp(self, name):
        self.calls.append(("set_tcp", name))
        if self.mode != MANUAL:
            return -1
        self.tcp = name
        return 0


@pytest.fixture(autouse=True)
def _no_sleep(monkeypatch):
    monkeypatch.setattr(MP.time, "sleep", lambda *_: None)


def test_autonomous에서_manual로_왕복해서_맞춘다(monkeypatch):
    api = ModeApi(start_mode=AUTONOMOUS)
    monkeypatch.setattr(MP, "_api", api)

    MP._ensure_tool_tcp(CFG)

    assert api.tool == "Tool Weight"
    assert api.tcp == "GripperDA_v1"
    assert api.mode == AUTONOMOUS           # 원래 모드로 되돌아왔다
    assert ("set_robot_mode", MANUAL) in api.calls
    assert ("set_robot_mode", AUTONOMOUS) in api.calls


def test_manual에서_시작해도_autonomous로_복귀한다(monkeypatch):
    api = ModeApi(start_mode=MANUAL, tool="Tool Weight", tcp="GripperDA_v1")
    monkeypatch.setattr(MP, "_api", api)

    MP._ensure_tool_tcp(CFG)

    assert api.mode == AUTONOMOUS
    assert [c for c in api.calls if c[0] == "set_robot_mode"] == [
        ("set_robot_mode", MANUAL), ("set_robot_mode", AUTONOMOUS),
    ]


def test_모드_전환과_조회를_못하면_기존_설정_대조로_판정한다(monkeypatch):
    class NoModeApi(ModeApi):
        def get_robot_mode(self):
            return -1                       # 두산 래퍼가 실패 시 주는 값

    api = NoModeApi(start_mode=AUTONOMOUS, tool="Tool Weight", tcp="GripperDA_v1",
                    allow_mode_switch=False)
    monkeypatch.setattr(MP, "_api", api)

    # 왕복은 못 하지만, 이미 값이 맞으므로 set 실패는 get 대조로 통과한다
    MP._ensure_tool_tcp(CFG)
    assert [c for c in api.calls if c[0] == "set_robot_mode"] == [
        ("set_robot_mode", MANUAL),
    ]
    assert api.mode == AUTONOMOUS


def test_manual_전환이_거부되면_왕복_없이_진행한다(monkeypatch):
    """펜던트가 제어권을 쥐고 있는 경우 흉내 — set_robot_mode 자체가 거부된다."""
    api = ModeApi(start_mode=AUTONOMOUS, tool="Tool Weight", tcp="GripperDA_v1",
                  allow_mode_switch=False)
    monkeypatch.setattr(MP, "_api", api)

    # 펜던트가 이미 맞춰 놓은 상태라 set 은 실패해도 get 대조로 통과한다
    MP._ensure_tool_tcp(CFG)
    assert api.mode == AUTONOMOUS


def test_원래_모드로_못_돌아오면_멈춘다(monkeypatch):
    api = ModeApi(start_mode=AUTONOMOUS, restore_fails=True)
    monkeypatch.setattr(MP, "_api", api)

    with pytest.raises(MP.MotionError, match="원래 모드"):
        MP._ensure_tool_tcp(CFG)


def test_tool_tcp가_안_맞아도_모드는_복원한다(monkeypatch):
    """왕복 중 tool/tcp 확인이 실패해도(MotionError), 모드 복원은 반드시 돈다."""
    api = ModeApi(start_mode=AUTONOMOUS, tool="WrongTool")

    def _reject_tool(name):                # MANUAL 이어도 이름이 안 먹히는 상황 흉내
        api.calls.append(("set_tool", name))
        return -1
    api.set_tool = _reject_tool

    monkeypatch.setattr(MP, "_api", api)

    with pytest.raises(MP.MotionError, match="tool"):
        MP._ensure_tool_tcp(CFG)

    assert api.mode == AUTONOMOUS            # 실패해도 모드는 되돌아왔다
