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


def test_교시_전에는_자세_조회를_거부한다():
    """전부 0 인 placeholder 로 움직이면 기타와 부딪힌다."""
    points = tp.TeachPoints(_data())
    with pytest.raises(tp.TeachPointsError, match="교시"):
        points.peg(2)          # 2번은 아직 미교시다


def test_교시를_마치면_조회된다():
    points = tp.TeachPoints(_taught())
    assert points.peg(2).taught is True


def test_페그는_작업좌표로_읽힌다():
    """좌표계 계산으로 다른 페그를 내려면 관절각이 아니라 작업좌표여야 한다."""
    points = tp.TeachPoints(_data())
    peg = points.peg(6)        # 6번은 실제 교시값이 들어 있다
    assert peg.kind == "task"
    assert peg.task == [341.0, -81.5, 113.0, 91.0, 90.0, 179.0]


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
    d["pegs"][1]["turn_sign"] = -1
    points = tp.TeachPoints(d)
    _, six = points.joint_delta(6, tp.TIGHTEN, 30.0)
    _, one = points.joint_delta(1, tp.TIGHTEN, 30.0)
    assert six == 30.0
    assert one == -30.0


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
    points = tp.TeachPoints(_data())
    with pytest.raises(tp.TeachPointsError, match="교시"):
        points.pick_holder()


def test_피크_거치대_교시가_끝나면_파지_설정까지_읽힌다():
    d = _taught()
    d["pick"]["holder"]["taught"] = True
    d["pick"]["holder"]["grip_width_mm"] = 3.0
    d["pick"]["holder"]["grip_force_n"] = 10.0
    points = tp.TeachPoints(d)
    holder = points.pick_holder()
    assert holder.grip_width_mm == 3.0
    assert holder.grip_force_n == 10.0


def test_pick_절이_없는_파일도_로드는_된다():
    """조회할 때만 걸린다. 예전 형식 YAML 로 노드가 뜨는 것까지는 막지 않는다."""
    d = _data()
    del d["pick"]
    points = tp.TeachPoints(d)
    with pytest.raises(tp.TeachPointsError, match="pick.holder"):
        points.pick_holder()
