"""teach_points 단위 테스트. 로봇·두산 API 없이 돈다."""
import os

import pytest
from tunio_robot import teach_points as tp
import yaml

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config", "teach_points.yaml")


def _data():
    with open(CONFIG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _taught():
    """저장소의 교시점 파일을 읽어 전부 교시된 것처럼 바꾼 사본."""
    d = _data()
    for name in d["frames"]:
        d["frames"][name]["taught"] = True
    for no in d["pegs"]:
        d["pegs"][no]["taught"] = True
    for no in d["pluck"]:
        d["pluck"][no]["taught"] = True
    return d


# ---------------------------------------------------------- 파일 자체
def test_저장소_교시점_파일이_구조검증을_통과한다():
    tp.load(CONFIG)


@pytest.mark.parametrize("key", ["turn_vel", "turn_acc"])
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf"), "3", True])
def test_줄별_회전속도는_유한한_양수만_받는다(key, value):
    data = _data()
    data["pegs"][6][key] = value
    with pytest.raises(tp.TeachPointsError, match=key):
        tp.TeachPoints(data)


def test_교시_전에는_자세_조회를_거부한다():
    """
    좌표가 들어 있어도 taught 가 false 면 거부한다.

    교시가 한 벌로 안 맞는다고 판명나면 값을 지우지 않고 taught 만 내린다.
    그때 동작이 막혀야 한다.
    """
    d = _data()
    d["pegs"][6]["taught"] = False
    with pytest.raises(tp.TeachPointsError, match="교시"):
        tp.TeachPoints(d).peg(6)


def test_교시를_마치면_조회된다():
    points = tp.TeachPoints(_taught())
    assert points.peg(2).taught is True


def test_페그는_작업좌표로_읽힌다():
    """좌표계 계산으로 다른 페그를 내려면 관절각이 아니라 작업좌표여야 한다."""
    peg = tp.TeachPoints(_data()).peg(6)
    assert peg.kind == "task"
    assert len(peg.task) == 6
    assert peg.joints is None


def test_대기_자세는_관절각이다():
    points = tp.TeachPoints(_data())
    standby = points.frame("standby")
    assert standby.kind == "joint"
    assert standby.joints == [0.0, 0.0, 75.0, 0.0, 100.0, 0.0]


def test_j와_x를_둘_다_적으면_걸린다():
    d = _data()
    d["pegs"][2]["j"] = [0, 0, 0, 0, 0, 0]
    with pytest.raises(tp.TeachPointsError, match="하나만"):
        tp.TeachPoints(d)


# ---------------------------------------------------------- 방향 변환
def test_조임과_풀림은_관절_부호가_반대다():
    points = tp.TeachPoints(_taught())
    _, tighten = points.joint_delta(6, tp.TIGHTEN, 30.0)
    _, loosen = points.joint_delta(6, tp.LOOSEN, 30.0)
    assert tighten == -loosen


def test_turn_sign_이_음수면_관절_부호가_뒤집힌다():
    """줄마다 헤드에서의 방향이 달라 같은 조임도 관절 부호가 반대일 수 있다."""
    d = _taught()
    d["pegs"][6]["turn_sign"] = 1
    d["pegs"][1]["turn_sign"] = -1
    points = tp.TeachPoints(d)
    _, six = points.joint_delta(6, tp.TIGHTEN, 30.0)
    _, one = points.joint_delta(1, tp.TIGHTEN, 30.0)
    assert six == 30.0
    assert one == -30.0


def test_저장소_파일은_네_줄_모두_반시계가_조임이다():
    """2026-09-19 사용자 확인. 반시계 = 조임 = 관절 음(-) 회전이라 전부 -1 이다."""
    points = tp.TeachPoints(_taught())
    for string in tp.STRINGS:
        _, delta = points.joint_delta(string, tp.TIGHTEN, 30.0)
        assert delta == -30.0, f"{string}번이 {delta} 다"
        _, back = points.joint_delta(string, tp.LOOSEN, 30.0)
        assert back == 30.0


def test_회전_관절_번호를_YAML에서_가져온다():
    d = _taught()
    d["motion"]["peg_joint"] = 5
    points = tp.TeachPoints(d)
    joint, _ = points.joint_delta(6, tp.TIGHTEN, 10.0)
    assert joint == 5


# ---------------------------------------------------------- goal 검증
def test_대상_아닌_줄은_거부한다():
    """3·4번은 이번 범위가 아니다 (CLAUDE.md 4절)."""
    points = tp.TeachPoints(_taught())
    with pytest.raises(tp.TeachPointsError, match="대상 줄"):
        points.joint_delta(3, tp.TIGHTEN, 10.0)


def test_각도는_양수_크기여야_한다():
    """명세: angle_deg 는 항상 양수. 부호는 direction 이 나른다."""
    points = tp.TeachPoints(_taught())
    for bad in (0.0, -10.0):
        with pytest.raises(tp.TeachPointsError, match="양수"):
            points.joint_delta(6, tp.TIGHTEN, bad)


def test_상한을_넘는_각도는_거부한다():
    """서버 계산이 틀려도 줄이 끊어지지 않게 로봇에서 한 번 더 막는다."""
    points = tp.TeachPoints(_taught())
    limit = points.max_angle_deg
    points.joint_delta(6, tp.TIGHTEN, limit)          # 상한 자체는 통과
    with pytest.raises(tp.TeachPointsError, match="상한"):
        points.joint_delta(6, tp.TIGHTEN, limit + 0.1)


def test_direction_은_1_또는_마이너스1만_받는다():
    points = tp.TeachPoints(_taught())
    for bad in (0, 2, -2):
        with pytest.raises(tp.TeachPointsError, match="direction"):
            points.joint_delta(6, bad, 10.0)


# ---------------------------------------------------------- 잘못된 파일
def test_줄이_빠지면_로드에서_걸린다():
    d = _data()
    del d["pegs"][1]
    with pytest.raises(tp.TeachPointsError, match="빠진 줄"):
        tp.TeachPoints(d)


def test_값이_6개가_아니면_걸린다():
    d = _data()
    d["pegs"][6]["x"] = [0, 0, 0]
    with pytest.raises(tp.TeachPointsError, match="6개"):
        tp.TeachPoints(d)


def test_turn_sign_이_없으면_걸린다():
    d = _data()
    del d["pegs"][6]["turn_sign"]
    with pytest.raises(tp.TeachPointsError, match="turn_sign"):
        tp.TeachPoints(d)


def test_peg_joint_범위를_벗어나면_걸린다():
    d = _data()
    d["motion"]["peg_joint"] = 7
    with pytest.raises(tp.TeachPointsError, match="peg_joint"):
        tp.TeachPoints(d)


# ---------------------------------------------------------- 피크 거치대
def test_피크_거치대도_교시_전에는_거부한다():
    """놓친 피크는 비전이 없어 되찾을 수 없다. 좌표 없이 움직이면 안 된다."""
    d = _data()
    d["pick"]["holder"]["taught"] = False      # 저장소 파일은 교시가 끝났을 수 있다. 명시적으로 내린다
    points = tp.TeachPoints(d)
    with pytest.raises(tp.TeachPointsError, match="교시"):
        points.pick_holder()


def test_피크_거치대_교시가_끝나면_파지_설정까지_읽힌다():
    d = _taught()
    d["pick"]["holder"]["taught"] = True
    d["pick"]["holder"]["tcp"] = d["robot"]["tcp"]
    d["pick"]["holder"]["grip_width_mm"] = 3.0
    d["pick"]["holder"]["grip_force_n"] = 10.0
    points = tp.TeachPoints(d)
    holder = points.pick_holder()
    assert holder.grip_width_mm == 3.0
    assert holder.grip_force_n == 10.0


# ───────────────────── 좌표 기준(TCP) 검사 — 이슈 #21 ─────────────────────
#
# posx 는 읽을 때 켜져 있던 TCP 기준의 숫자다. 페그는 GripperDA_v1 을 켜고,
# 발현·거치대는 끄고(플랜지) 교시해서 한 파일에 208mm 어긋난 두 기준이 섞였다.
# connect() 가 robot.tcp 하나만 켜므로, 다른 기준으로 적힌 좌표는 거부한다.

def test_기준이_안_적혀_있으면_거부한다():
    d = _taught()
    d["pick"]["holder"]["taught"] = True
    d["pick"]["holder"]["tcp"] = None
    points = tp.TeachPoints(d)
    with pytest.raises(tp.TeachPointsError) as e:
        points.pick_holder()
    assert "tcp" in str(e.value)


def test_기준이_다르면_거부한다():
    """플랜지로 읽은 좌표를 GripperDA_v1 로 실행하면 208mm 어긋난다."""
    d = _taught()
    d["pegs"][6]["tcp"] = "다른TCP"
    points = tp.TeachPoints(d)
    with pytest.raises(tp.TeachPointsError) as e:
        points.peg(6)
    assert "다른TCP" in str(e.value)


def test_기준이_같으면_통과한다():
    d = _taught()
    points = tp.TeachPoints(d)
    assert points.peg(6).tcp == d["robot"]["tcp"]


def test_관절각_자세는_기준을_안_본다():
    """관절각은 TCP 와 무관하다. 대기 자세에 tcp 를 적으라고 하면 안 된다."""
    d = _taught()
    d["frames"]["standby"].pop("tcp", None)
    points = tp.TeachPoints(d)
    assert points.frame("standby").joints is not None


def test_pick_절이_없는_파일도_로드는_된다():
    """조회할 때만 걸린다. 예전 형식 YAML 로 노드가 뜨는 것까지는 막지 않는다."""
    d = _data()
    del d["pick"]
    points = tp.TeachPoints(d)
    with pytest.raises(tp.TeachPointsError, match="pick.holder"):
        points.pick_holder()


# ---------------------------------------------------------- 경유지
def test_저장소_경유지가_J6만_현재값을_유지한다():
    """J6 은 페그를 돌리는 관절이라 어디에 있든 상관없다."""
    route = tp.TeachPoints(_data()).route()
    assert route.resolve([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]) == \
        [90.0, 0.0, 0.0, 0.0, 0.0, 6.0]


def test_null_인_관절은_현재값을_유지한다():
    """부딪히지 않는 데 필요한 관절만 적으면 된다는 게 경유지의 요점이다."""
    d = _data()
    d["routes"]["transit"] = {"j": [30.0, None, 60.0, None, None, None], "taught": True}
    got = tp.TeachPoints(d).route().resolve([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert got == [30.0, 2.0, 60.0, 4.0, 5.0, 6.0]


def test_전부_적으면_자세가_완전히_고정된다():
    d = _data()
    d["routes"]["transit"] = {"j": [1, 2, 3, 4, 5, 6], "taught": True}
    assert tp.TeachPoints(d).route().resolve([9.0] * 6) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_미교시_경유지는_None_이라_경유하지_않는다():
    """값을 안 채운 상태로 경유하면 엉뚱한 자세로 간다."""
    d = _data()
    d["routes"]["transit"]["taught"] = False
    assert tp.TeachPoints(d).route() is None


def test_줄_번호로_좌우를_가른다():
    assert tp.side_of(6) == tp.side_of(5) == "left"
    assert tp.side_of(2) == tp.side_of(1) == "right"


def test_경유지_관절값이_6개가_아니면_걸린다():
    d = _data()
    d["routes"]["transit"] = {"j": [1, 2, 3], "taught": True}
    with pytest.raises(tp.TeachPointsError, match="6개"):
        tp.TeachPoints(d)


def test_경유지에_숫자도_null도_아닌_값이_있으면_걸린다():
    d = _data()
    d["routes"]["transit"] = {"j": ["x", None, None, None, None, None],
                              "taught": True}
    with pytest.raises(tp.TeachPointsError, match="null"):
        tp.TeachPoints(d)


# ───────────── 발현 스트로크 방향 (이슈 #32) ─────────────
#
# 6·5번(저음 쪽)과 2·1번(고음 쪽)은 빈 공간이 반대라 훑는 방향이 반대다.
# 2026-09-20 실기 확인. 도구 안에만 있던 것을 YAML 로 올렸다.

def test_스트로크_방향이_YAML에서_온다():
    points = tp.TeachPoints(_taught())
    assert points.pluck(6).stroke_sign == 1
    assert points.pluck(1).stroke_sign == -1


def test_저음과_고음의_방향이_반대다():
    """같은 방향으로 훑으면 한쪽은 옆 줄이나 몸통에 걸린다."""
    points = tp.TeachPoints(_taught())
    assert points.pluck(6).stroke_sign == points.pluck(5).stroke_sign
    assert points.pluck(2).stroke_sign == points.pluck(1).stroke_sign
    assert points.pluck(6).stroke_sign != points.pluck(1).stroke_sign


def test_스트로크_방향은_1_아니면_마이너스1():
    d = _taught()
    d["pluck"][6]["stroke_sign"] = 0
    with pytest.raises(tp.TeachPointsError, match="stroke_sign"):
        tp.TeachPoints(d)


def test_스트로크_방향이_없으면_1로_본다():
    d = _taught()
    del d["pluck"][6]["stroke_sign"]
    assert tp.TeachPoints(d).pluck(6).stroke_sign == 1


# ---------------------------------------------------------- 진입 파라미터 (#51)
# 2026-09-21 6번 실기. 밀기 전에도 툴 무게·자세 몫으로 이만큼 읽힌다
INSERT_BASELINE_N = 1.2


def test_미는_힘이_탐색_거리를_감당한다():
    """
    갈 수 있는 거리보다 멀리 가라고 하면 **항상 타임아웃**이다. 첫 페그에서 멈춘다.

    순응 스프링이 밀린 만큼 되민다. 미는 힘은 상한이 걸려 있으므로, 스프링 반력이
    그 상한에 닿는 지점에서 팔이 선다. 그 거리가 reach 다. 전에 search_mm 25.0 에
    insert_force_n 3.0 이라 9mm 밖에 못 가면서 25mm 를 가라고 하고 있었다.
    """
    d = _data()
    handle, motion = d["peg_handle"], d["motion"]
    k = motion["compliance_stx"][1] / 1000.0        # 삽입축(Y) N/m -> N/mm
    reach = (handle["insert_force_n"] - INSERT_BASELINE_N) / k
    assert reach >= handle["search_mm"], (
        f"{handle['insert_force_n']}N 으로는 {reach:.0f}mm 까지만 간다 — "
        f"search_mm {handle['search_mm']} 를 못 채운다 (#51)")


def test_막힘_문턱이_미는_힘보다_낮다():
    """상한 이상이면 막혀도 영영 안 걸린다. 그대로 20초를 버티다 타임아웃된다."""
    handle = _data()["peg_handle"]
    assert handle["contact_force_n"] < handle["insert_force_n"]


# ------------------------------------------------------------ 거치대 팔 모양 (#58)
def test_거치대_왼쪽_sol7_적용과_오른쪽_기존값_유지():
    """6·5번은 실기한 sol 7, 2·1번은 현재 팔 모양을 유지한다."""
    points = tp.TeachPoints(_data())
    assert points.pick_holder().sol_by_side == {"left": 7, "right": None}


def test_거치대_sol_에_이상한_값은_거부한다():
    """0~7 정수 아니면 안 받는다. 엉뚱한 값을 주면 팔이 통째로 뒤집힌다."""
    d = _data()
    d["pick"]["holder"]["sol_by_side"] = {"left": 9}
    with pytest.raises(tp.TeachPointsError, match="sol_by_side"):
        tp.TeachPoints(d)


def test_거치대_sol_의_키는_좌우뿐이다():
    d = _data()
    d["pick"]["holder"]["sol_by_side"] = {"middle": 2}
    with pytest.raises(tp.TeachPointsError, match="left/right"):
        tp.TeachPoints(d)
