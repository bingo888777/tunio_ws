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


def test_손목_뒤집기는_케이싱을_옮긴다():
    """축 원점은 그대로인데 L자 케이싱만 반대로 가야 한다. sol 3 vs sol 2 실측."""
    sol3 = [54.93, 54.49, 80.75, 134.83, -124.90, -30.04]
    sol2 = [55.01, 54.53, 80.52, -45.10, 125.08, 149.88]
    a, b = fk(sol3), fk(sol2)
    # 축 원점(어깨팔꿈치·손목중심)은 거의 같다
    for k in (2, 3):
        assert max(abs(x - y) for x, y in
                   zip(a["origins"][k], b["origins"][k])) < 3.0
    # 케이싱은 크게 옮겨간다. 2026-09-19 기준 X 168mm, Z 97mm
    d = [a["wrist_casing"][i] - b["wrist_casing"][i] for i in range(3)]
    assert d[0] > 150.0 and d[2] > 80.0, f"케이싱이 안 옮겨졌다: {d}"


def test_케이싱은_손목중심에서_146mm_안에_있다():
    """link_4 무게중심은 자기 축에서 146mm 떨어져 있다. 그 이상 벌어지면 계산이 틀린 것."""
    j = [125.1, -51.8, -87.9, 46.6, -120.7, 29.1]
    r = fk(j)
    d = sum((a - b) ** 2 for a, b in zip(r["wrist_casing"], r["origins"][3])) ** 0.5
    assert 140.0 < d < 152.0, f"{d:.1f}mm"


def test_팔모양_고르기가_1번에서_sol2를_고른다():
    """2026-09-19 실측 — 8개 중 sol 2 만 남아야 한다. 3 은 손목이 기타 쪽으로 뻗는다."""
    from tunio_robot.arm_pose import pick, problems
    cands = [
        (0, [54.95, 129.53, -80.70, -132.69, 127.88, 33.52]),
        (1, [54.83, 129.57, -80.87, 47.28, -127.72, -146.58]),
        (2, [55.01, 54.53, 80.52, -45.10, 125.08, 149.88]),
        (3, [54.93, 54.49, 80.75, 134.83, -124.90, -30.04]),
        (4, [-123.99, -129.63, 80.76, 46.16, 128.45, 32.90]),
        (5, [-124.11, -129.67, 80.93, -133.88, -128.29, -147.20]),
        (6, [-123.93, -54.44, -80.89, 136.14, 125.47, 150.84]),
        (7, [-124.01, -54.40, -81.12, -43.93, -125.29, -29.08]),
    ]
    best = pick(cands, route_j1=90.0)
    assert best is not None and best[0] == 2
    # 받침을 파고드는 것과 멀리 도는 것은 걸러져야 한다
    assert problems(cands[0][1], 90.0)
    assert problems(cands[6][1], 90.0)
    # 실제로 쓰던 sol 3 자체는 '못 쓸 이유'는 없다. 손목이 더 뻗을 뿐이다
    assert not problems(cands[3][1], 90.0)


def test_gather_가_못_푸는_해를_걸러낸다():
    """역기구학은 닿지 않는 자세에 전부 0 이나 예외를 낸다. 그걸 해로 세면 안 된다."""
    from tunio_robot.arm_pose import gather

    def fake(sol):
        if sol in (0, 1):
            raise RuntimeError("못 푼다")
        if sol == 2:
            return [0.0] * 6
        return [float(sol), 54.5, 80.5, -45.1, 125.1, 149.9]

    got = gather(fake)
    assert [s for s, _ in got] == [3, 4, 5, 6, 7]


def test_choose_가_실측_8개에서_sol2를_고른다():
    """arm_pose.choose 는 gather + pick 이다. 도구 셋이 같은 값을 쓰게 하려고 뺐다."""
    from tunio_robot.arm_pose import choose
    table = {
        0: [54.95, 129.53, -80.70, -132.69, 127.88, 33.52],
        1: [54.83, 129.57, -80.87, 47.28, -127.72, -146.58],
        2: [55.01, 54.53, 80.52, -45.10, 125.08, 149.88],
        3: [54.93, 54.49, 80.75, 134.83, -124.90, -30.04],
        4: [-123.99, -129.63, 80.76, 46.16, 128.45, 32.90],
        5: [-124.11, -129.67, 80.93, -133.88, -128.29, -147.20],
        6: [-123.93, -54.44, -80.89, 136.14, 125.47, 150.84],
        7: [-124.01, -54.40, -81.12, -43.93, -125.29, -29.08],
    }
    got = choose(lambda s: table[s], route_j1=90.0)
    assert got is not None and got[0] == 2


def test_choose_는_쓸_만한_게_없으면_None():
    from tunio_robot.arm_pose import choose
    # 전부 어깨팔꿈치가 베이스 아래로 가는 해만 준다
    bad = [54.95, 129.53, -80.70, -132.69, 127.88, 33.52]
    assert choose(lambda s: bad, route_j1=90.0) is None


def test_회전_속도가_이동_속도와_따로_간다():
    """돌리는 건 줄에 장력을 거는 동작이라 이동보다 느려야 한다."""
    from tunio_robot.motion_primitives import turn_motion
    cfg = {"vel": 5.0, "acc": 10.0, "turn_vel": 2.0, "turn_acc": 4.0,
           "approach_mm": 100.0}
    t = turn_motion(cfg)
    assert t["vel"] == 2.0 and t["acc"] == 4.0
    assert t["approach_mm"] == 100.0          # 나머지는 그대로 따라온다
    assert cfg["vel"] == 5.0                  # 원본을 안 건드린다


def test_turn_vel_이_없으면_이동_속도를_쓴다():
    """예전 형식 YAML 로도 노드가 뜨게 한다. 느려지기만 안 할 뿐이다."""
    from tunio_robot.motion_primitives import turn_motion
    assert turn_motion({"vel": 5.0, "acc": 10.0})["vel"] == 5.0


def test_저장소_파일에_회전_속도가_있다():
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), "..", "config",
                        "teach_points.yaml")
    with open(path, "r", encoding="utf-8") as f:
        motion = yaml.safe_load(f)["motion"]
    assert motion["turn_vel"] < motion["vel"], "회전이 이동보다 빠르면 안 된다"
