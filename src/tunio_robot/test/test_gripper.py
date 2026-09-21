"""RG2 프리셋 선택 단위 테스트. 그리퍼·로봇 없이 돈다."""
import os

import pytest
from tunio_robot import gripper as G
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "teach_points.yaml")


def _presets():
    with open(CONFIG, "r", encoding="utf-8") as f:
        return G.parse_presets(yaml.safe_load(f)["gripper"]["presets"])


class FakeApi:
    """set_digital_output 호출을 순서대로 기록한다."""

    def __init__(self):
        self.calls = []

    def set_digital_output(self, index, val):
        self.calls.append((index, val))


def test_저장소_프리셋이_읽힌다():
    p = _presets()
    assert set(p) == {"open", "close", "peg"}
    assert p["peg"].width_mm == 21
    assert p["open"].width_mm > p["peg"].width_mm


def test_출력을_전부_내린_뒤_올린다():
    """중간 조합이 다른 프리셋과 맞아떨어지면 엉뚱한 파지가 한 번 실행된다."""
    rg = G.RG2(FakeApi(), _presets(), settle_sec=0.0)
    rg.select("peg")
    calls = rg.api.calls
    assert calls[:3] == [(1, 0), (2, 0), (3, 0)]      # 먼저 전부 0
    assert calls[3:] == [(3, 1)]                      # 그다음 필요한 것만 1


def test_프리셋마다_다른_조합이_나간다():
    rg = G.RG2(FakeApi(), _presets(), settle_sec=0.0)
    seen = {}
    for name in ("open", "close", "peg"):
        rg.api.calls.clear()
        rg.select(name)
        seen[name] = tuple(c for c in rg.api.calls if c[1] == 1)
    assert len(set(seen.values())) == 3               # 셋이 서로 달라야 한다
    assert seen["peg"] == ((3, 1),)


def test_없는_프리셋은_거부한다():
    """웹에 규칙이 없으면 코드가 만들어낼 수 없다."""
    rg = G.RG2(FakeApi(), _presets(), settle_sec=0.0)
    with pytest.raises(G.GripperError, match="프리셋이 없다"):
        rg.select("pick")


def test_고른_프리셋을_기억한다():
    rg = G.RG2(FakeApi(), _presets(), settle_sec=0.0)
    assert rg.current is None
    rg.select("peg")
    assert rg.current == "peg"


def test_출력값이_0이나_1이_아니면_걸린다():
    with pytest.raises(G.GripperError, match="0/1"):
        G.parse_presets({"bad": {"outputs": {1: 2}}})


def test_outputs_가_없으면_걸린다():
    with pytest.raises(G.GripperError, match="outputs"):
        G.parse_presets({"bad": {"width_mm": 21}})


# ─────────────── 열기 폭이 안전한 값인가 (이슈 #31) ───────────────
#
# 숫자를 박아두지 않고 **조건**을 본다. 웹로직에서 폭을 바꾸면 yaml 기록도 같이
# 고치게 되어 있는데(어긋나도 로봇은 그대로 움직인다), 고칠 때 안전 범위를 벗어나면
# 여기서 걸린다.

def _cfg():
    with open(CONFIG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_손잡이를_다_덮지_못하면_사전_정렬이_필수다():
    """
    폭만으로는 답이 없다는 것이 2026-09-21 실물로 확정됐다.

      38.2mm 보다 커야   손잡이가 59도쯤 서 있을 때 안 닿는다
      38.0mm 이하여야    옆 페그를 안 친다 (41·42·45 전부 쳤다)

    창이 겹치지 않는다. 그래서 38 로 두고 **사람이 작업 전에 세로로 선 손잡이를
    눕혀 둔다.** 그 전제가 빠지면 로봇이 들이받는다. 폭을 다시 건드릴 때 이
    전제를 같이 지우는 일이 없도록 여기서 묶어 둔다.
    """
    d = _cfg()
    handle = d["peg_handle"]
    assert handle["max_sweep_mm"] > handle["length_mm"]      # 대각선이 90도보다 넓다
    if d["gripper"]["presets"]["open"]["width_mm"] <= handle["max_sweep_mm"]:
        assert handle.get("requires_prealign") is True
        assert handle.get("contact_force_n")                 # 빠뜨렸을 때 걸러낼 수단


def test_열기_폭이_손잡이_두께보다는_넓다():
    """눕혀 놓은 손잡이조차 못 지나면 사전 정렬을 해도 소용이 없다."""
    d = _cfg()
    assert d["gripper"]["presets"]["open"]["width_mm"] > d["peg_handle"]["thickness_mm"]


def test_열기_폭이_옆_페그를_안_친다():
    """
    손끝이 중심에서 폭/2 만큼 나간다. 옆 페그 간격(실측 38.0~38.6mm)보다 작아야 한다.

    90mm 일 때 반경 45mm 로 실제로 쳤다. 그래서 줄인 것이다.
    """
    d = _cfg()
    peg_spacing_mm = 38.0        # 2026-09-19 교시에서 잰 양쪽 열 간격 중 작은 쪽
    assert d["gripper"]["presets"]["open"]["width_mm"] / 2.0 < peg_spacing_mm


def test_문턱이_실측값_사이에_있다():
    """빈손 < 문턱 < 물림 < 문턱 < 사점 순서가 깨지면 판정이 뒤집힌다."""
    h = _cfg()["peg_handle"]
    t = h["thresholds_mm"]
    assert h["empty_mm"] < t["empty_below"] < h["thickness_mm"]
    assert h["thickness_mm"] < t["dead_above"] < h["length_mm"]
