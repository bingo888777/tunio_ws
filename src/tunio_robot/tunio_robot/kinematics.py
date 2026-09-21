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


def tool_z_axis(posx):
    """
    주어진 posx 자세의 툴 Z축을 Base 기준 단위벡터로.

    두산 posx 의 자세 [A, B, C] 는 intrinsic ZYZ 오일러각이다.
    R = Rz(A)·Ry(B)·Rz(C) 이고 툴 Z축은 그 3열인데, Rz(C) 는 Z축을 안 건드리므로
    C 가 빠진다. 그래서 손목이 얼마나 돌아 있든 툴 Z 방향은 A·B 로만 정해진다.
    """
    a, b = math.radians(posx[3]), math.radians(posx[4])
    return (math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.cos(b))


def shift_tcp(posx, delta_mm):
    """
    같은 물리적 자세를 **다른 TCP 기준의 posx** 로 옮긴다 (이슈 #21).

    posx     — Base 절대 [X, Y, Z, A, B, C]
    delta_mm — 원래 기준점에서 새 기준점까지 툴 Z 방향 거리(mm).
               플랜지 → GripperDA_v1(툴 Z 208mm) 이면 +208.
               GripperDA_v1 → 플랜지면 -208.

    왜 이게 되나
      TCP 오프셋이 [0, 0, 208, 0, 0, 0] 로 **순수 평행이동**이다. 자세를 안 돌리므로
      A·B·C 는 그대로고 위치만 툴 Z 방향으로 밀린다. 그래서 재교시가 필요 없다.

    실측 확인 (2026-09-21)
      발현 접촉점 4줄이 플랜지 기준(태그 tcpless-4strings-ok)과 GripperDA_v1 기준
      (2026-09-19 교시) 양쪽으로 남아 있어 서로 견줄 수 있었다. 변환한 Z 가
      교시값과 5·2·1번에서 1.2~1.4mm 차이였다. 6번은 그 사이에 재교시돼 6.4mm 다.
      X·Y 는 이 자세들이 B=±180 이라 변환이 건드리지 않는데 2.5~5.4mm 차이가
      나는데, 그건 교시 세션이 달라 생긴 드리프트지 변환 오차가 아니다.
    """
    if len(posx) < 6:
        raise ValueError("posx 는 [X, Y, Z, A, B, C] 6개다")
    z = tool_z_axis(posx)
    return [posx[i] + z[i] * delta_mm for i in range(3)] + list(posx[3:6])


def roll_about_tool_z(posx, deg):
    """
    툴 Z 둘레로 손목만 돌린 자세를 만든다. 손끝 위치와 접근축은 안 바뀐다 (이슈 #31).

    intrinsic ZYZ 에서 마지막 Rz(C) 가 **툴 Z 둘레** 회전이다. 툴 Z 자체는
    Rz(A)Ry(B) 로 정해지므로 C 를 건드려도 방향이 안 바뀐다
    (teach_points.approach_axis 주석과 같은 이야기다).

    그래서 페그 손잡이가 돌아가 있으면 C 에 그만큼 더해 주면 된다. 접근 경로는
    그대로 두고 손가락 방향만 맞춘다.
    """
    if len(posx) < 6:
        raise ValueError("posx 는 [X, Y, Z, A, B, C] 6개다")
    out = list(posx)
    out[5] = posx[5] + float(deg)
    return out


def wrist_casing(joints_deg):
    """J5 쪽 L자 케이싱이 부푼 자리만 [X, Y, Z] mm 로."""
    return fk(joints_deg)["wrist_casing"]


def elbow(joints_deg):
    """팔꿈치(J3 축 원점) 위치만 [X, Y, Z] mm 로."""
    return fk(joints_deg)["elbow"]
