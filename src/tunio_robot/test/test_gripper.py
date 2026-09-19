"""RG2 명령 계산 단위 테스트. 그리퍼·로봇 없이 돈다."""
import pytest
from tunio_robot import gripper as G


def test_폭은_0_1mm_단위_정수_문자열로_나간다():
    """onrobot_rg_control 의 genCommand 가 숫자 문자열을 rgwd 로 읽는다."""
    cmds, _ = G.plan_commands(6.0, G.INITIAL_FORCE_N, G.INITIAL_FORCE_N)
    assert cmds == ["60"]


def test_힘을_내리면_d_가_그만큼_붙는다():
    """서버가 40N 으로 시작하므로 20N 을 쓰려면 2.5N씩 8번 내려야 한다."""
    cmds, reached = G.plan_commands(6.0, 20.0, 40.0)
    assert cmds == ["d"] * 8 + ["60"]
    assert reached == 20.0


def test_힘을_올리면_i_가_붙는다():
    cmds, reached = G.plan_commands(6.0, 25.0, 20.0)
    assert cmds == ["i", "i", "60"]
    assert reached == 25.0


def test_이미_목표_힘이면_힘_명령은_안_보낸다():
    cmds, reached = G.plan_commands(6.0, 20.0, 20.0)
    assert cmds == ["60"]
    assert reached == 20.0


def test_2_5N_단위라_목표를_정확히_못_맞추면_도달값을_돌려준다():
    """21N 은 2.5N 격자에 없다. 실제 도달값을 알려줘야 다음 계산이 맞는다."""
    cmds, reached = G.plan_commands(6.0, 21.0, 40.0)
    assert reached == 20.0
    assert cmds.count("d") == 8


def test_힘_계산이_누적돼도_어긋나지_않는다():
    """연속 호출 시 직전 도달값을 넣으면 명령이 중복되지 않는다."""
    _, first = G.plan_commands(6.0, 20.0, G.INITIAL_FORCE_N)
    cmds, second = G.plan_commands(8.0, 20.0, first)
    assert cmds == ["80"]          # 힘 명령 없음
    assert second == 20.0


def test_RG2_범위를_벗어난_폭은_거부한다():
    for bad in (-1.0, 110.1):
        with pytest.raises(G.GripperError, match="폭"):
            G.plan_commands(bad, 20.0, 40.0)


def test_RG2_범위를_벗어난_힘은_거부한다():
    """rg6 로 잘못 띄우면 40N 초과가 통과해버린다. 여기서 한 번 더 막는다."""
    for bad in (-1.0, 40.1):
        with pytest.raises(G.GripperError, match="힘"):
            G.plan_commands(6.0, bad, 40.0)


def test_한계값은_RG2_사양과_같다():
    assert G.MAX_FORCE_N == 40.0
    assert G.MAX_WIDTH_MM == 110.0
