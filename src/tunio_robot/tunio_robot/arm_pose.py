"""
팔 모양(sol) 고르기 — 같은 좌표를 갈 수 있는 8가지 중 안전한 것을 뽑는다.

사람이 코를 만질 때 팔꿈치를 위로 들 수도 옆으로 벌릴 수도 있다. 손끝은 같은데
팔 모양만 다르다. 로봇도 같아서 한 좌표에 역기구학 해가 8개다.
안 정해 주면 로봇이 그때그때 고르고, 그래서 매번 달라진다.

무엇으로 거르나 (2026-09-19 실측에서 나온 기준)
  1. 어깨팔꿈치가 베이스 면(Z=0) 아래로 내려가면 받침·책상을 파고든다. 못 쓴다.
  2. 경유지 J1 에서 너무 많이 돌아야 하면 가는 길이 기타를 쓸고 간다.
  3. 남은 것 중 **손목 케이싱이 기타에서 가장 먼 것**을 고른다.
     기타는 베이스에서 멀리(X 약 300~345) 놓여 있으므로, 케이싱 X 가 작을수록
     안쪽으로 물러나 있는 것이다. 1번 페그에서 X 376 은 걸렸고 X 207 은 괜찮았다.

케이싱이 왜 기준인가
  손목 뒤집기 갈래끼리는 **축 원점이 전부 같다.** 축 위치만 봐서는 구분이 안 된다.
  실제로 부푼 곳은 link_4 의 L자 케이싱이고 자기 축에서 146mm 벗어나 있다.
  부딪힐 뻔한 게 그것이다. kinematics.fk 의 wrist_casing 참조.
"""
from tunio_robot.kinematics import fk

# 어깨팔꿈치가 이 높이(mm) 아래로 내려가면 받침을 파고든다
MIN_ELBOW_Z = 0.0
# 경유지 J1 에서 이보다 많이(도) 돌아야 하면 가는 길이 길다
MAX_J1_SWING = 100.0


def problems(joints, route_j1=None):
    """이 팔 모양을 못 쓰는 이유들. 빈 목록이면 쓸 만하다."""
    out = []
    elbow_z = fk(joints)["origins"][2][2]
    if elbow_z < MIN_ELBOW_Z:
        out.append(f"어깨팔꿈치가 베이스 면 아래 {-elbow_z:.0f}mm — 받침을 파고든다")
    if route_j1 is not None and abs(joints[0] - route_j1) > MAX_J1_SWING:
        out.append(f"경유지 J1({route_j1:.0f})에서 {abs(joints[0] - route_j1):.0f}도 "
                   f"돌아야 한다 — 가는 길이 기타를 쓴다")
    return out


def rank(candidates, route_j1=None):
    """
    (sol, 관절각) 목록을 안전한 순서로 돌려준다.

    반환은 (sol, joints, casing, problems) 의 목록. 쓸 만한 것이 앞에 오고,
    그 안에서는 손목 케이싱이 기타에서 먼(=X 가 작은) 순이다.
    걸러진 것도 뒤에 붙여서 돌려준다. 왜 걸렀는지 보여줘야 하기 때문이다.
    """
    rows = []
    for sol, joints in candidates:
        rows.append((sol, list(joints), fk(joints)["wrist_casing"],
                     problems(joints, route_j1)))
    rows.sort(key=lambda r: (bool(r[3]), r[2][0]))
    return rows


def pick(candidates, route_j1=None):
    """제일 나은 팔 모양 하나. 쓸 만한 게 없으면 None."""
    rows = rank(candidates, route_j1)
    if not rows or rows[0][3]:
        return None
    return rows[0]


def gather(ikin_fn):
    """
    해 8가지(sol 0~7)를 물어 (sol, 관절각) 목록을 만든다.

    ikin_fn — sol 번호를 받아 관절각 6개를 내는 함수. 보통
              lambda s: DSR_ROBOT2.ikin(posx(task), s) 를 넘긴다.
    못 푸는 해(닿지 않는 자세)는 뺀다. 전부 0 이 나오는 것도 해가 없는 것이다.
    """
    out = []
    for sol in range(8):
        try:
            j = ikin_fn(sol)
        except Exception:                               # noqa: B902
            continue
        if j is not None and len(j) >= 6 and not all(abs(v) < 1e-9 for v in j):
            out.append((sol, [float(v) for v in j]))
    return out


def choose(ikin_fn, route_j1=None):
    """
    역기구학으로 8가지를 받아 제일 나은 것을 고른다. (sol, 관절각, 케이싱).

    **팔 모양을 안 고르면 로봇이 그때그때 현재 자세를 따라간다.** 경유지가
    특이점 자세라 그 값이 모호해서 갈 때마다 달라지고, 2026-09-19 회전 확인에서
    실제로 경유지→6번 접근점 경로가 순회 때와 달라져 부딪힐 뻔했다.
    """
    best = pick(gather(ikin_fn), route_j1)
    if best is None:
        return None
    sol, joints, casing, _ = best
    return sol, joints, casing
