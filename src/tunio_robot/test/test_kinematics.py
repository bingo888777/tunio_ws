"""정기구학이 실측 자세와 맞는지 본다. 2026-09-19 좌우 전환 검증에서 읽은 값이다."""
from tunio_robot.kinematics import elbow, fk

# (관절각, 로봇이 보고한 손끝 XYZ). transit_check.py 출력에서 그대로 옮겼다
MEASURED = [
    ([125.1, -51.8, -87.9, 46.6, -120.7, 29.1], [338.5, -135.4, 112.9]),
    ([52.5, 51.4, 87.9, 134.2, -120.9, -28.0], [339.2, 119.8, 113.9]),
    ([90.0, 0.0, 0.0, 0.0, 0.0, 29.1], [-6.3, 1.4, 1242.9]),
]


def test_실측_자세와_3mm_이내로_맞는다():
    for joints, expect in MEASURED:
        got = fk(joints)["tcp"]
        err = sum((a - b) ** 2 for a, b in zip(got, expect)) ** 0.5
        assert err < 3.0, f"{joints} 에서 {err:.2f}mm 어긋난다"


def test_손목_뒤집기는_손끝을_안_바꾼다():
    """J4 를 180도 돌리고 J5 부호를 뒤집어도 손끝은 그대로여야 한다."""
    j = [52.5, 51.4, 87.9, 134.2, -120.9, -28.0]
    flipped = [j[0], j[1], j[2], j[3] - 180.0, -j[4], j[5] + 180.0]
    a, b = fk(j)["tcp"], fk(flipped)["tcp"]
    assert max(abs(x - y) for x, y in zip(a, b)) < 0.01


def test_팔꿈치는_J3축_원점이다():
    j = [125.1, -51.8, -87.9, 46.6, -120.7, 29.1]
    assert elbow(j) == fk(j)["origins"][2]


def test_tcp_0이면_플랜지가_나온다():
    j = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    assert fk(j, tcp_mm=0.0)["tcp"] == fk(j)["flange"]
