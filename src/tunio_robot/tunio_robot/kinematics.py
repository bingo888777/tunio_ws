"""
M0609 정기구학 — 관절각으로 팔꿈치·손목이 어디 있는지 계산한다.

왜 필요한가
  같은 손끝 좌표라도 역기구학 해(팔 자세, sol)가 8가지다. 손끝은 같은데
  팔꿈치가 정반대로 간다. 2026-09-19 검증에서 1번 페그 접근점의 팔꿈치가
  바깥쪽으로 뻗어 부딪힐 뻔했다. 어느 해를 골라야 하는지 알려면 손끝 말고
  **팔꿈치가 어디로 가는지**를 봐야 하는데, 두산 API 는 손끝만 알려준다.

어디서 온 값인가
  dsr_description2/xacro/macro.m0609.white.xacro 의 joint origin 을 그대로 옮겼다.
  실측 자세 3개로 맞춰 보니 오차 1.3~2.8mm 다 (대부분 Z 방향 2mm 쏠림).
  충돌 여유를 보는 용도라 이 정도면 충분하다. 정밀 위치는 로봇에게 물어라.

쓰는 법
  from tunio_robot.kinematics import fk
  r = fk([125.1, -51.8, -87.9, 46.6, -120.7, 29.1])
  r["tcp"]      -> 손끝 [X, Y, Z] mm
  r["elbow"]    -> 팔꿈치(J3 축 원점) [X, Y, Z] mm
  r["wrist"]    -> 손목(J5 축 원점) [X, Y, Z] mm
  r["wrist_casing"] -> J5 쪽 L자 케이싱이 부푼 자리 [X, Y, Z] mm
  r["origins"]  -> 관절 1~6 축 원점 전부
"""
import math

# (부모→자식 고정 변환의 rpy, xyz[m]) — 관절 회전은 이 변환 뒤에 Z축으로 걸린다
_H = math.pi / 2
_CHAIN = (
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.1345)),
    ((0.0, -_H, -_H), (0.0, 0.0062, 0.0)),
    ((0.0, 0.0, _H), (0.411, 0.0, 0.0)),
    ((_H, 0.0, 0.0), (0.0, -0.368, 0.0)),
    ((-_H, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ((_H, 0.0, 0.0), (0.0, -0.121, 0.0)),
)

# teach_points.yaml 의 robot.tcp 오프셋. 툴 Z 로 208mm
DEFAULT_TCP_MM = 208.0

# link_4 의 무게중심(xacro 의 inertial origin, 단위 m). 자기 축에서 146mm 벗어나 있다.
# J5 쪽 L자 케이싱이 부푼 자리라, "손목이 어느 쪽으로 뻗었나"를 이 점으로 본다.
# 손목 뒤집기 갈래는 축 원점이 전부 같아서 이 점으로만 구분된다.
_WRIST_CASING = (0.00004, 0.097079, -0.10981)


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _homog(rot, pos):
    m = [list(rot[i]) + [pos[i]] for i in range(3)]
    m.append([0.0, 0.0, 0.0, 1.0])
    return m


def _rpy(r, p, y):
    """URDF 의 rpy — Z(y)·Y(p)·X(r) 순으로 곱한 회전행렬."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _rotz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def fk(joints_deg, tcp_mm=DEFAULT_TCP_MM):
    """
    관절각(도) 6개로 각 축의 원점과 손끝 위치를 낸다. 단위는 mm.

    tcp_mm — 플랜지에서 툴 Z 방향으로 나간 거리. 0 이면 플랜지 위치가 나온다.
    """
    if len(joints_deg) < 6:
        raise ValueError("관절각 6개가 필요하다")
    m = _homog(((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0.0, 0.0, 0.0))
    origins = []
    link4 = None
    for n, ((rpy_, xyz), q) in enumerate(zip(_CHAIN, joints_deg[:6])):
        m = _matmul(m, _homog(_rpy(*rpy_), xyz))
        m = _matmul(m, _homog(_rotz(math.radians(q)), (0.0, 0.0, 0.0)))
        origins.append([m[i][3] * 1000.0 for i in range(3)])
        if n == 3:
            link4 = [row[:] for row in m]
    flange = [m[i][3] * 1000.0 for i in range(3)]
    tool_z = [m[i][2] for i in range(3)]
    tcp = [flange[i] + tool_z[i] * tcp_mm for i in range(3)]
    casing = [sum(link4[i][k] * _WRIST_CASING[k] for k in range(3)) * 1000.0
              + origins[3][i] for i in range(3)]
    return {
        "origins": origins,
        "wrist_casing": casing,    # J5 쪽 L자 케이싱이 부푼 자리
        "flange": flange,
        "tcp": tcp,
        "tool_z": tool_z,
        "elbow": origins[2],        # J3 축 원점
        "wrist": origins[4],        # J5 축 원점
    }


def wrist_casing(joints_deg):
    """J5 쪽 L자 케이싱이 부푼 자리만 [X, Y, Z] mm 로."""
    return fk(joints_deg)["wrist_casing"]


def elbow(joints_deg):
    """팔꿈치(J3 축 원점) 위치만 [X, Y, Z] mm 로."""
    return fk(joints_deg)["elbow"]
