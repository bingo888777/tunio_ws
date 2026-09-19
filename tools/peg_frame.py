#!/usr/bin/env python3
"""
헤드스톡 좌표계 맞추기 — 교시한 페그 좌표로 나머지 페그를 추정하고 검증한다.

왜 필요한가
  페그를 전부 교시하면 제일 확실하지만, 기타를 옮길 때마다 조금씩 틀어진다.
  몇 개만 교시하고 나머지는 계산으로 내면, 다시 놓을 때도 기준 페그 몇 개만
  다시 잡으면 된다. 두산 API 의 set_user_cart_coord(x1, x2, x3, pos) 가
  이 좌표계를 로봇에 등록하는 함수다.

좌표계 정의
  원점 — 기준 페그(보통 6번)
  X축  — 같은 쪽 이웃 페그로 향하는 방향 (헤드스톡 긴 축, 끝쪽이 +)
  Y축  — 반대편 페그열로 향하는 방향 (X축 성분을 뺀 나머지)
  Z축  — X x Y

쓰는 법
  python3 tools/peg_frame.py                      # 저장소 교시점으로 분석
  python3 tools/peg_frame.py --verify 2 308 60 109.5   # 실측값과 대조

단위: 위치 mm, 자세 intrinsic ZYZ deg (두산 posx 규약).
"""
import argparse
import os
import sys

import numpy as np
import yaml

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")

# 헤드스톡에서의 페그 순서. 같은 쪽끼리 묶는다.
# 6->5->4 와 1->2->3 은 둘 다 너트에서 헤드 끝으로 가는 방향이다.
LEFT_ORDER = [6, 5, 4]
RIGHT_ORDER = [1, 2, 3]


def load_poses(path):
    """
    교시점 YAML 에서 작업 좌표(x)가 있는 페그를 꺼낸다.

    taught 가 false 여도 가져온다. 값이 서로 안 맞는지 보는 게 이 도구의 일이라
    "아직 못 믿는 값"이야말로 봐야 할 대상이다. 대신 교시 시점을 같이 돌려준다.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    out, meta = {}, {}
    for key, item in (data.get("pegs") or {}).items():
        pose = item.get("x")
        if pose and any(v != 0 for v in pose):
            no = int(key)
            out[no] = np.array(pose, dtype=float)
            meta[no] = (item.get("session"), bool(item.get("taught")))
    return out, meta


def side_of(no):
    if no in LEFT_ORDER:
        return "left"
    if no in RIGHT_ORDER:
        return "right"
    raise SystemExit(f"{no}번은 헤드스톡 배치에 없는 줄이다")


def axis_from_pair(poses, order):
    """같은 쪽 페그 두 개로 헤드스톡 긴 축 방향과 칸 간격을 구한다."""
    have = [n for n in order if n in poses]
    if len(have) < 2:
        return None, None, have
    a, b = have[0], have[1]
    step = poses[b][:3] - poses[a][:3]          # 너트 -> 끝 방향 한 칸
    pitch = float(np.linalg.norm(step))
    return step, pitch, have


def build_frame(poses):
    """
    헤드스톡 좌표계를 만든다.

    반환: (원점, 3x3 회전행렬[열이 X,Y,Z축], 한 칸 벡터)
    """
    step, pitch, have = axis_from_pair(poses, LEFT_ORDER)
    if step is None:
        raise SystemExit("같은 쪽 페그가 두 개는 있어야 축을 잡는다 (예: 6번·5번)")

    origin = poses[have[0]][:3]
    ex = step / pitch

    # 반대편 페그로 Y축을 잡는다. X 성분은 빼서 직교시킨다
    other = [n for n in RIGHT_ORDER if n in poses]
    if not other:
        raise SystemExit("반대편 페그가 하나는 있어야 Y축을 잡는다 (예: 1번)")
    v = poses[other[0]][:3] - origin
    ey = v - np.dot(v, ex) * ex
    ey /= np.linalg.norm(ey)
    ez = np.cross(ex, ey)

    return origin, np.column_stack([ex, ey, ez]), step


def to_frame(p, origin, rot):
    return rot.T @ (np.asarray(p, dtype=float)[:3] - origin)


def predict(no, poses, origin, rot, step):
    """
    아직 안 잡은 페그의 위치를 추정한다.

    같은 쪽에 이미 잡은 페그가 있으면 거기서 칸 수만큼 떨어뜨린다.
    자세(A,B,C)는 같은 쪽 페그 것을 그대로 쓴다. 접근 방향이 같기 때문이다.
    """
    order = LEFT_ORDER if side_of(no) == "left" else RIGHT_ORDER
    known = [n for n in order if n in poses]
    if not known:
        return None, None
    base = known[0]
    gap = order.index(no) - order.index(base)
    pos = poses[base][:3] + step * gap
    return pos, poses[base][3:]


def report_alignment(step, pitch):
    """X축 축정렬이 얼마나 틀어졌는지 각도로 보여준다."""
    yaw = np.degrees(np.arctan2(step[1], step[0]))
    yaw = yaw - 180.0 if yaw > 90 else (yaw + 180.0 if yaw < -90 else yaw)
    pitch_deg = np.degrees(np.arctan2(-step[2], np.hypot(step[0], step[1])))
    print("축정렬 상태 (기타를 BASE X축에 맞춰 놓았다는 전제)")
    print(f"  페그 한 칸 간격   {pitch:.2f} mm")
    print(f"  수평 틀어짐(yaw)  {yaw:+.2f} 도   <- XY평면에서 X축과의 각")
    print(f"  상하 기울기       {pitch_deg:+.2f} 도   <- 헤드스톡이 들리거나 숙인 정도")
    if abs(yaw) > 2.0 or abs(pitch_deg) > 2.0:
        print("  → 2도를 넘는다. 매번 이 정도로 틀어진다면 기준 페그를 다시 잡고")
        print("     좌표계를 새로 만드는 편이 낫다. 고정 좌표를 그냥 쓰면 헛잡는다")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=DEFAULT_YAML, help="교시점 파일")
    ap.add_argument("--verify", nargs=4, metavar=("줄번호", "X", "Y", "Z"),
                    help="실측값과 추정값을 대조한다")
    ap.add_argument("--repeat", nargs=4, metavar=("줄번호", "X", "Y", "Z"),
                    help="이미 교시한 페그를 다시 잡아 재현성을 본다")
    args = ap.parse_args()

    poses, meta = load_poses(os.path.abspath(args.yaml))
    if not poses:
        raise SystemExit(f"기록된 작업 좌표가 없다: {args.yaml}")

    print("기록된 페그")
    sessions = set()
    for no in sorted(poses, reverse=True):
        session, taught = meta[no]
        sessions.add(session)
        mark = "" if taught else "  (taught: false — 아직 못 믿는 값)"
        print(f"  {no}번  교시 {session}{mark}")
    if len(sessions) > 1:
        print("\n  ⚠️ 교시 시점이 서로 다르다. 그 사이 기타가 움직였다면")
        print("     아래 분석의 불일치는 페그 배치가 아니라 기타 이동 때문이다.")
    print()
    origin, rot, step = build_frame(poses)
    _, pitch, _ = axis_from_pair(poses, LEFT_ORDER)
    report_alignment(step, pitch)

    print("\n헤드스톡 좌표계에서 본 위치 (원점=기준 페그, X=헤드 끝 방향)")
    for no in sorted(poses, reverse=True):
        f = to_frame(poses[no], origin, rot)
        print(f"  {no}번  X {f[0]:+8.2f}  Y {f[1]:+8.2f}  Z {f[2]:+8.2f} mm")

    print("\n아직 안 잡은 페그 추정")
    for no in (6, 5, 2, 1):
        if no in poses:
            continue
        pos, ori = predict(no, poses, origin, rot, step)
        if pos is None:
            print(f"  {no}번 — 같은 쪽에 잡은 페그가 없어 추정 불가")
            continue
        print(f"  {no}번  [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}, "
              f"{ori[0]:.1f}, {ori[1]:.1f}, {ori[2]:.1f}]")

    if args.repeat:
        no = int(args.repeat[0])
        again = np.array([float(v) for v in args.repeat[1:]])
        if no not in poses:
            raise SystemExit(f"{no}번은 교시된 적이 없다. --verify 를 쓰라")
        err = again - poses[no][:3]
        dist = float(np.linalg.norm(err))
        print(f"\n{'=' * 60}")
        print(f"재현성 — {no}번 페그를 다시 잡았을 때")
        print(f"{'=' * 60}")
        print(f"  처음  [{poses[no][0]:.1f}, {poses[no][1]:.1f}, {poses[no][2]:.1f}]")
        print(f"  다시  [{again[0]:.1f}, {again[1]:.1f}, {again[2]:.1f}]")
        print(f"  차이  X {err[0]:+.1f}  Y {err[1]:+.1f}  Z {err[2]:+.1f} mm"
              f"   (거리 {dist:.1f} mm)")
        print()
        if dist <= 1.5:
            print("  교시가 재현된다. 페그 사이 불일치는 교시 손떨림이 아니라")
            print("  실제 배치 차이다. 좌표를 그대로 믿고 쓰면 된다.")
        elif dist <= 4.0:
            print("  같은 페그를 잡아도 이만큼 달라진다. 페그끼리의 불일치도")
            print("  상당 부분 이 탓이다. 파지 폭 여유가 이 크기를 덮어야 한다.")
        else:
            print("  교시 자체가 안 흔들리게 잡는 게 먼저다. 이 상태로는")
            print("  어떤 좌표를 넣어도 헛잡는다. 접근 자세·조명·받침대 고정을 보라.")

    if args.verify:
        no = int(args.verify[0])
        measured = np.array([float(v) for v in args.verify[1:]])
        pos, _ = predict(no, poses, origin, rot, step)
        if pos is None:
            raise SystemExit(f"{no}번을 추정할 수 없다")
        err = measured - pos
        dist = float(np.linalg.norm(err))
        print(f"\n{'=' * 60}")
        print(f"검증 — {no}번 페그")
        print(f"{'=' * 60}")
        print(f"  추정  [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
        print(f"  실측  [{measured[0]:.1f}, {measured[1]:.1f}, {measured[2]:.1f}]")
        print(f"  오차  X {err[0]:+.1f}  Y {err[1]:+.1f}  Z {err[2]:+.1f} mm"
              f"   (거리 {dist:.1f} mm)")
        print()
        if dist <= 2.0:
            print("  계산으로 내도 되겠다. 페그 배치가 규칙적이고 축이 잘 잡혔다.")
        elif dist <= 5.0:
            print("  접근은 맞지만 그대로 쓰기엔 아슬아슬하다.")
            print("  여유를 준 파지 폭이 이 오차를 덮을 수 있는지 확인하고 쓸 것.")
        else:
            print("  계산으로 내면 헛잡는다. 페그마다 직접 교시하는 편이 낫다.")
            print("  원인 후보 — 페그 간격이 균일하지 않거나, 양쪽 열이 평행하지 않거나,")
            print("  교시 때마다 기타가 틀어졌거나, TCP 가 서로 다르거나.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
