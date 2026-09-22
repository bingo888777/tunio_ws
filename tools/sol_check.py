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
  손목 뒤집기끼리는 **축 원점이 똑같다.** 그래서 축 위치만 봐서는 구분이 안 된다.
  대신 link_4 의 L자 케이싱이 부푼 자리를 같이 찍는다 (kinematics.py 참조).
  부딪힐 뻔한 게 그 케이싱이다.

  걸러내는 기준 두 가지를 자동으로 붙여 준다 —
    · 어깨팔꿈치가 베이스 면(Z=0) 아래로 내려가면 받침·책상을 파고든다. 못 쓴다.
    · 경유지 J1 에서 많이 돌아야 하는 해는 가는 길에 기타를 쓸고 간다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/sol_check.py --peg 1
  python3 tools/sol_check.py --peg 1 --at peg      # 접근점 말고 페그 자체에서
  python3 tools/sol_check.py --peg 1 --approach 60
  python3 tools/sol_check.py --point pick.holder   # 피크 거치대 (#58)
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")


def _sign(v):
    return 1 if v >= 0 else -1


class _Say:
    """connect() 가 진행을 찍게 하는 최소 출력기. 어디서 멈췄는지 보여야 한다."""

    def info(self, m):
        print(f"    {m}")

    def warn(self, m):
        print(f"    ! {m}")

    def error(self, m):
        print(f"    !! {m}")


def fmt(vals, w=7, d=1):
    return "[" + ", ".join(f"{v:{w}.{d}f}" for v in vals) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, choices=[1, 2, 5, 6])
    ap.add_argument("--point", choices=["pick.holder"],
                    help="페그 말고 다른 교시점에서 본다. 거치대 팔 모양(#58)용")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--at", choices=["approach", "peg"], default="approach",
                    help="접근점에서 볼지 페그 자체에서 볼지")
    ap.add_argument("--approach", type=float, default=None,
                    help="물러나는 거리(mm). 기본은 motion.approach_mm")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    if (args.peg is None) == (args.point is None):
        ap.error("--peg 나 --point 중 하나만 준다")

    from tunio_robot import teach_points as TP
    from tunio_robot.kinematics import fk
    points = TP.load(os.path.abspath(args.yaml))
    back = args.approach if args.approach is not None \
        else points.motion.get("approach_mm", 100.0)

    if args.point == "pick.holder":
        # 거치대는 **위에서 수직으로** 내려간다. 그래서 볼 자리는 페그처럼
        # "뒤로 물러난 점" 이 아니라 "위로 올라간 점" 이다 (pick_up_mm).
        holder = points.pick_holder()
        up = points.motion.get("pick_up_mm", 50.0)
        target = list(holder.task)
        target[2] += up
        title = f"피크 거치대 위 {up:.0f}mm 대기점"
        sub = f"  (거치대 {fmt(holder.task, 8, 2)} 에서 Base +Z {up:.0f}mm)"
        now = holder.sol_by_side
        cur = (f"left={now.get('left')} right={now.get('right')}"
               if any(v is not None for v in now.values()) else "둘 다 null — 미지정")
        cur = f"지금 적힌 sol_by_side: {cur}"
    else:
        peg = points.peg(args.peg)
        target = list(peg.task) if args.at == "peg" else TP.backed_off(peg.task, back)
        title = f"{args.peg}번 페그 {'접근점' if args.at == 'approach' else '파지점'}"
        sub = f"  (페그에서 {back:.0f}mm 뒤)" if args.at == "approach" else ""
        cur = f"지금 적힌 sol: {peg.sol if peg.sol is not None else 'null — 미지정'}"

    print()
    print("=" * 70)
    print(f"{title} 팔 자세 비교")
    print("=" * 70)
    print(f"  손끝 좌표 {fmt(target, 8, 2)}")
    if sub:
        print(sub)
    print(f"  {cur}")
    print("  로봇을 움직이지 않는다. 물어보기만 한다.\n")

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_sol_check", namespace=args.id)
    DR_init.__dsr__node = node

    # 서비스 디스커버리를 기다리고 TCP 를 확인한다. 둘 다 빠져 있었다 —
    #   · 안 기다리면 ikin 첫 호출이 **영원히 멈춘다.** spin_until_future_complete
    #     에 타임아웃이 없다 (#21). 이 도구만 빠져 있었다 (2026-09-21)
    #   · TCP 가 안 맞으면 ikin 이 **다른 기준으로 푼다.** 페그 좌표는
    #     GripperDA_v1(툴 Z 208mm) 기준이라, 안 켜진 채 풀면 208mm 어긋난
    #     자리의 팔 모양이 나온다. 그걸 보고 sol 을 고르면 조용히 틀린다 (#26)
    from tunio_robot import motion_primitives as MP
    robot_cfg = dict(points.robot)
    robot_cfg["id"], robot_cfg["model"] = args.id, args.model
    print("  서비스를 기다린다 (최대 30초) …")
    MP.connect(robot_cfg, logger=_Say())
    print(f"  TCP 확인됨 — tool='{MP._read_named('tool')}' "
          f"tcp='{MP._read_named('tcp')}'\n")

    route = points.route()
    route_j1 = route.joints[0] if route and route.joints[0] is not None else None

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
    ok = []
    for sol, j in rows:
        r = fk(j)
        # 같은 갈래인가. 값 자체는 접근 거리에 따라 몇 도씩 달라지므로
        # 부호(어느 갈래인지)로 본다
        same = (_sign(j[2]) == _sign(now_j[2]) and _sign(j[4]) == _sign(now_j[4])
                and abs(j[0] - now_j[0]) < 90.0)
        bad = []
        if r["origins"][2][2] < 0.0:
            bad.append(f"어깨팔꿈치가 베이스 면 아래 {-r['origins'][2][2]:.0f}mm "
                       f"— 받침을 파고든다")
        if route_j1 is not None and abs(j[0] - route_j1) > 100.0:
            bad.append(f"경유지 J1({route_j1:.0f})에서 {abs(j[0] - route_j1):.0f}도 "
                       f"돌아야 한다 — 가는 길이 길다")
        tag = "  <- 지금 자세" if same else ""
        print(f"sol {sol}   {fmt(j, 7, 2)}{tag}")
        print(f"        J1 {j[0]:+7.2f}도 — {'왼쪽' if j[0] < 0 else '오른쪽'}"
              f"  (6·5번은 왼쪽, 2·1번은 오른쪽)")
        if args.point == "pick.holder":
            # **코드와 같은 함수로 판단한다.** 따로 계산하면 도구는 "경유 안 함" 이라
            # 찍고 실기에서는 경유하는 일이 생긴다 (motion_primitives.j1_flip)
            flip, turn = MP.j1_flip(now_j[0], j[0], points.motion)

            print(f"        지금 J1 {now_j[0]:+.2f} 에서 {turn:.0f}도 돌아야 한다"
                  f"  ->  {'J1=90 경유지를 탄다' if flip else '경유 없이 바로 간다'}")
        print(f"        {'손목 뒤집힘(J5<0)' if j[4] < 0 else '손목 안 뒤집힘(J5>0)'}"
              f"    케이싱 {fmt(r['wrist_casing'])}")
        print(f"        어깨팔꿈치 {fmt(r['origins'][2])}   "
              f"손목중심 {fmt(r['origins'][3])}")
        for b in bad:
            print(f"        X  {b}")
        print()
        if not bad:
            ok.append((sol, j, r, same))

    print("=" * 70)
    print("고르는 법")
    print("=" * 70)
    if not ok:
        print("  쓸 만한 해가 없다. 페그 좌표나 경유지부터 다시 보라.")
        return 1
    print("  걸러지고 남은 해 — " + ", ".join(f"sol {s}" for s, _, _, _ in ok))
    cur = next((r for _, _, r, same in ok if same), None)
    if cur is not None:
        far = max(ok, key=lambda t: abs(t[2]["wrist_casing"][0]
                                        - cur["wrist_casing"][0]))
        if far[3]:
            print("  지금 자세 말고 케이싱이 다른 데로 가는 해가 없다.")
        else:
            d = [far[2]["wrist_casing"][i] - cur["wrist_casing"][i] for i in range(3)]
            print(f"  케이싱을 가장 많이 옮기는 건 sol {far[0]} 이다 "
                  f"(X {d[0]:+.0f}, Y {d[1]:+.0f}, Z {d[2]:+.0f} mm).")
    print("  케이싱이 기타에서 먼 쪽(X 작고 Z 낮은 쪽)인 해를 고른다.")
    if args.point == "pick.holder":
        thr = points.motion.get("j1_flip_deg", 120.0)
        print(f"  경유 판단 문턱은 motion.j1_flip_deg = {thr:g}도 다.")
        print("  이 도구가 찍는 '경유지를 탄다/안 탄다' 는 motion_primitives.j1_flip 이")
        print("  낸 값 그대로다 — 실기에서 코드가 내릴 판단과 같다.")
        print("  ⚠️ 위 판단은 **지금 팔이 서 있는 자세 기준**이다. 실기에서는 직전")
        print("     동작이 남긴 자세에서 출발하므로 줄마다 달라질 수 있다.")
        print()
        print("  거치대는 **J1 부호**로 고른다 — 다음에 갈 페그 쪽과 같아야")
        print("  좌우 전환 경유지를 건너뛴다 (#58).")
        print("    J1 음수 -> sol_by_side.left  (6·5번 쪽)")
        print("    J1 양수 -> sol_by_side.right (2·1번 쪽)")
        print("  적은 뒤 dry_run 으로 순서를 보고, 실기는 천천히 한 번 —")
        print("    ros2 run tunio_robot robot_controller --ros-args -p dry_run:=true")
        print("    teach_points.yaml 의 pick.holder.sol_by_side 에 적는다")
        return 0
    print("  실제로 가보고 확인한 뒤에 적는다 —")
    print(f"    python3 tools/transit_check.py --from 6 --to {args.peg} "
          f"--dst-sol <번호> --speed 30")
    print(f"    teach_points.yaml 의 pegs.{args.peg}.sol 에 그 번호를 적는다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
