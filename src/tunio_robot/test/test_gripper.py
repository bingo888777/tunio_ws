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
    # 2026-09-21 — 90 에서 줄였다. 놓을 때 벌어진 손가락이 옆 페그를 쳤다 (#31).
    # 웹로직이 실제 폭을 정하므로 이 값은 기록일 뿐이다. 어긋나도 로봇은 그대로 움직인다
    assert p["open"].width_mm == 30


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
