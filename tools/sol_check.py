#!/usr/bin/env python3
"""
팔 자세(sol) 고르기 — 같은 손끝 좌표의 역기구학 해 8가지를 비교한다.

**로봇을 움직이지 않는다.** ikin 서비스로 물어보기만 한다.

왜 필요한가
  손끝 좌표가 같아도 팔 모양은 8가지다. 어깨가 앞/뒤, 팔꿈치가 위/아래,
  손목이 뒤집힘/안 뒤집힘. 2026-09-19 좌우 전환 검증에서 1번 페그 접근점의
  손목 케이싱이 바깥쪽으로 뻗어 부딪힐 뻔했다.
  손목 뒤집기 갈래는 **손끝을 하나도 안 바꾸고** 케이싱만 반대로 돌린다.

  teach_points.yaml 의 pegs.<번호>.sol 이 null 이면 그때그때 현재 자세를
  따라가서 매번 달라진다. 여기서 고른 값을 적어 고정해야 한다.

읽는 법
  손목 갈래는 J5 부호로 갈린다. 어깨·팔꿈치 갈래는 J1·J3 로 갈린다.
  어깨팔꿈치(J3 축)와 손목중심(J4=J5 축) 위치를 같이 찍어 준다.
  손목 뒤집기끼리는 축 원점이 똑같다 — 케이싱 방향만 다르다. 눈으로 봐야 한다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/sol_check.py --peg 1
  python3 tools/sol_check.py --peg 1 --at peg      # 접근점 말고 페그 자체에서
  python3 tools/sol_check.py --peg 1 --approach 60
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")


def fmt(vals, w=7, d=1):
    return "[" + ", ".join(f"{v:{w}.{d}f}" for v in vals) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--at", choices=["approach", "peg"], default="approach",
                    help="접근점에서 볼지 페그 자체에서 볼지")
    ap.add_argument("--approach", type=float, default=None,
                    help="물러나는 거리(mm). 기본은 motion.approach_mm")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    from tunio_robot.kinematics import fk
    points = TP.load(os.path.abspath(args.yaml))
    peg = points.peg(args.peg)
    back = args.approach if args.approach is not None \
        else points.motion.get("approach_mm", 100.0)
    target = list(peg.task) if args.at == "peg" else TP.backed_off(peg.task, back)

    print()
    print("=" * 70)
    print(f"{args.peg}번 페그 {'접근점' if args.at == 'approach' else '파지점'} "
          f"팔 자세 비교")
    print("=" * 70)
    print(f"  손끝 좌표 {fmt(target, 8, 2)}")
    if args.at == "approach":
        print(f"  (페그에서 {back:.0f}mm 뒤)")
    print(f"  지금 적힌 sol: {peg.sol if peg.sol is not None else 'null — 미지정'}")
    print("  로봇을 움직이지 않는다. 물어보기만 한다.\n")

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_sol_check", namespace=args.id)
    DR_init.__dsr__node = node

    rows = []
    try:
        from DR_common2 import posx
        from DSR_ROBOT2 import get_current_posj, get_current_solution_space, ikin

        now_j = [round(v, 2) for v in get_current_posj()]
        now_sol = get_current_solution_space()
        print(f"  지금 자세  sol {now_sol}  {fmt(now_j, 7, 2)}\n")

        for sol in range(8):
            try:
                j = list(ikin(posx(target), sol))
            except Exception as e:                      # noqa: B902
                print(f"  sol {sol}  못 푼다: {e}")
                continue
            if not j or len(j) < 6 or all(abs(v) < 1e-9 for v in j):
                print(f"  sol {sol}  해가 없다 (닿지 않는 자세)")
                continue
            rows.append((sol, [round(v, 2) for v in j]))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not rows:
        print("\n푼 해가 하나도 없다. 드라이버와 좌표를 확인하라")
        return 1

    print()
    for sol, j in rows:
        r = fk(j)
        mark = "  <- 지금 자세" if abs(j[4] - now_j[4]) < 1.0 and \
            abs(j[1] - now_j[1]) < 1.0 and abs(j[0] - now_j[0]) < 1.0 else ""
        wrist_side = "손목 뒤집힘(J5<0)" if j[4] < 0 else "손목 안 뒤집힘(J5>0)"
        print(f"sol {sol}   {fmt(j, 7, 2)}{mark}")
        print(f"        {wrist_side}")
        print(f"        어깨팔꿈치(J3축) {fmt(r['origins'][2])}   "
              f"손목중심 {fmt(r['origins'][3])}")
        print(f"        손끝 {fmt(r['tcp'])}")
        print()

    print("=" * 70)
    print("고르는 법")
    print("=" * 70)
    print("  1. J5 부호가 지금과 반대인 해가 손목 케이싱을 반대편으로 넘긴 것이다.")
    print("     축 원점이 같아도 케이싱은 정반대로 간다. 손끝은 안 바뀐다.")
    print("  2. 어깨팔꿈치가 기타에서 먼 쪽인 해를 고른다.")
    print("  3. 골랐으면 실제로 가보고 확인한다 —")
    print(f"       python3 tools/transit_check.py --from 6 --to {args.peg} "
          f"--dst-sol <번호> --speed 30")
    print("  4. 괜찮으면 teach_points.yaml 의 "
          f"pegs.{args.peg}.sol 에 그 번호를 적는다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
