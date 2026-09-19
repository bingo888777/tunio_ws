#!/usr/bin/env python3
"""
관절 조그 — ROS 로 조금씩 밀면서 경유지를 찾는다.

드라이버가 붙어 있으면 로봇이 자동 모드로 잡혀서 티치펜던트의 직접교시가
안 먹는다. 둘 중 하나만 된다. 그래서 **손으로 미는 대신 여기서 민다.**
관절 이동(movej 상대)이라 특이점과 무관하고, 움직인 뒤 값을 바로 찍어 준다.

**팔이 움직인다.** 비상정지를 손 닿는 곳에 두고 시작하라.
한 번에 한 관절만, 작은 각도씩 움직이는 것이 기본이다.

명령
  1 -20        1번 관절을 -20도 (관절 번호 공백 각도)
  1-20         붙여 써도 된다
  6 +90        6번 관절을 +90도
  step 5       기본 각도를 5도로 (그 뒤로는 '1+' '1-' 만 쳐도 된다)
  1+           기본 각도만큼 +
  p            지금 자세 찍기 (안 움직인다)
  u            직전 이동 되돌리기
  m            지금 자세를 후보로 적어두기
  go 6         6번 페그 접근점으로 (movejx, 크게 움직인다)
  go route     teach_points.yaml 의 경유지로 (movej)
  go standby   대기 자세로
  q            끝내기. 적어둔 후보와 yaml 조각을 찍는다

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/jog.py
  python3 tools/jog.py --speed 20        # 더 느리게
  python3 tools/jog.py --confirm-over 10 # 10도 넘으면 매번 확인
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")

CMD = re.compile(r"^([1-6])\s*([+-])\s*([0-9.]*)$")


def fmt(vals):
    return "[" + ", ".join(f"{v:7.2f}" for v in vals) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--step", type=float, default=5.0, help="기본 이동 각도(도)")
    ap.add_argument("--speed", type=float, default=30.0,
                    help="설정 속도의 몇 %% 로 움직일지")
    ap.add_argument("--confirm-over", type=float, default=15.0,
                    help="이 각도를 넘으면 움직이기 전에 확인을 받는다")
    ap.add_argument("--approach", type=float, default=None)
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))

    if args.approach is None:
        args.approach = points.motion.get("approach_mm", 100.0)
    motion = dict(points.motion)
    scale = max(0.05, args.speed / 100.0)
    motion["vel"] = points.motion.get("vel", 30) * scale
    motion["acc"] = points.motion.get("acc", 30) * scale
    motion["travel_vel"] = [v * scale for v in points.motion.get("travel_vel", [50.0, 30.0])]
    motion["travel_acc"] = [v * scale for v in points.motion.get("travel_acc", [100.0, 60.0])]

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_jog", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DSR_ROBOT2 import (get_current_posj, get_current_posx,
                            get_current_solution_space)

    def show():
        posj = list(get_current_posj())
        posx = get_current_posx()
        pos = posx[0] if isinstance(posx, (list, tuple)) and len(posx) == 2 else posx
        print(f"  관절 {fmt(posj)} deg")
        print(f"  손끝 {fmt(list(pos)[:6])}  (앞 3개 mm, 뒤 3개 deg)")
        print(f"  팔 자세(sol) {get_current_solution_space()}")
        return posj

    step = args.step
    marks = []
    last = None

    print()
    print("=" * 62)
    print("관절 조그")
    print("=" * 62)
    print("팔이 움직인다. 비상정지를 손 닿는 곳에.")
    print(f"속도 설정값의 {args.speed:.0f}% (관절 {motion['vel']:.1f} deg/s), "
          f"기본 각도 {step:.1f}도")
    print("명령은 'h' 로 다시 본다. 끝내기는 q 또는 Ctrl+C\n")
    show()

    def jog(joint, delta):
        nonlocal last
        if abs(delta) > args.confirm_over:
            now = list(get_current_posj())
            tgt = list(now)
            tgt[joint - 1] += delta
            print(f"  J{joint} {now[joint - 1]:.2f} -> {tgt[joint - 1]:.2f}")
            if input("  큰 각도다. 진행할까? (엔터=진행, n=취소) ").strip().lower().startswith("n"):
                print("  취소했다")
                return
        d = [0.0] * 6
        d[joint - 1] = delta
        MP._movej(d, motion, relative=True)
        last = (joint, -delta)
        show()

    try:
        while True:
            raw = input(f"조그[{step:g}도] ").strip().lower()
            if not raw:
                continue
            if raw in ("q", "quit", "exit"):
                break
            if raw in ("h", "?", "help"):
                print(__doc__.split("명령\n")[1].split("쓰는 법")[0])
                continue
            if raw == "p":
                show()
                continue
            if raw == "u":
                if last is None:
                    print("  되돌릴 게 없다")
                    continue
                j, d = last
                print(f"  되돌린다: J{j} {d:+.2f}")
                jog(j, d)
                last = None
                continue
            if raw == "m":
                posj = [round(v, 2) for v in get_current_posj()]
                marks.append(posj)
                print(f"  후보 {len(marks)} 적어뒀다: {posj}")
                continue
            if raw.startswith("step"):
                try:
                    step = float(raw.split()[1])
                    print(f"  기본 각도 {step:g}도")
                except (IndexError, ValueError):
                    print("  step 5 처럼 쳐라")
                continue
            if raw.startswith("go"):
                what = raw[2:].strip()
                try:
                    if what in ("route", "transit", "경유지"):
                        route = points.route()
                        if route is None:
                            print("  경유지가 미교시다")
                            continue
                        print(f"  경유지로 {route.joints} (null 은 현재값)")
                        if input("  진행할까? (엔터=진행, n=취소) ").strip().startswith("n"):
                            continue
                        MP.goto_route(route, motion)
                    elif what in ("standby", "대기"):
                        pose = points.frame("standby")
                        print(f"  대기 자세로 {pose.joints}")
                        if input("  진행할까? (엔터=진행, n=취소) ").strip().startswith("n"):
                            continue
                        MP._movej(pose.joints, motion)
                    elif what in ("1", "2", "5", "6"):
                        peg = points.peg(int(what))
                        back = TP.backed_off(peg.task, args.approach)
                        print(f"  {what}번 접근점으로 {fmt(back)} "
                              f"({args.approach:.0f}mm 뒤). 크게 움직인다")
                        if input("  진행할까? (엔터=진행, n=취소) ").strip().startswith("n"):
                            continue
                        MP._movejx(back, motion, sol=peg.sol)
                    else:
                        print("  go 6 / go 1 / go route / go standby")
                        continue
                except Exception as e:                  # noqa: B902
                    print(f"  실패: {e}")
                    continue
                last = None
                show()
                continue
            m = CMD.match(raw.replace(" ", ""))
            if not m:
                print("  '1 -20' 또는 '1-' 처럼 쳐라. 'h' 로 도움말")
                continue
            joint = int(m.group(1))
            amount = float(m.group(3)) if m.group(3) else step
            jog(joint, amount if m.group(2) == "+" else -amount)
    except (KeyboardInterrupt, EOFError):
        print("\n끊었다. 팔은 그 자리에 선다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'=' * 62}")
    print("적어둔 후보")
    print("=" * 62)
    if not marks:
        print("  없다 ('m' 으로 적어둔다)")
        return 0
    for i, j in enumerate(marks, 1):
        print(f"  {i}. {j}")
    j = marks[-1]
    print("\n마지막 후보를 경유지로 쓰려면 teach_points.yaml 의 routes.transit 에 —")
    print(f"  전부 고정:  j: {j}")
    print(f"  J1·J2·J3만: j: [{j[0]}, {j[1]}, {j[2]}, null, null, null]")
    print(f"  J1만:       j: [{j[0]}, null, null, null, null, null]")
    print("  taught: true 로 바꾸는 것도 잊지 마라")
    return 0


if __name__ == "__main__":
    sys.exit(main())
