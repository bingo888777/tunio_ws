#!/usr/bin/env python3
"""
좌우 전환 검증 — 로봇 점검 체크리스트 2.5단계.

한쪽 페그에서 반대쪽으로 넘어갈 때 기타나 주변 물체를 안 치는지 본다.
**페그를 잡지 않는다.** 이동만 한다.

왜 따로 보는가
  왼쪽 페그는 B=+90, 오른쪽은 B=-90 이라 손목이 180도 뒤집혀야 한다.
  TCP 가 208mm 라 그 뒤집힘에 손끝이 반경 20cm 짜리 호를 그린다.
  게다가 이동 중에는 그리퍼가 열려 있어 폭도 제일 넓다 (open 프리셋).
  페그를 잡는 것보다 이쪽이 부딪힐 위험이 크다.

어떻게 보는가
  한 구간 가고 멈춘다. 눈으로 보고, 괜찮으면 다음 구간.
  중간에 아니다 싶으면 n 이나 Ctrl+C. 팔은 그 자리에 선다.
  처음 가보는 경로는 --speed 30 쯤으로 낮추고, 비상정지에 손을 올려두고 본다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/transit_check.py --from 6 --to 1
  python3 tools/transit_check.py --from 6 --to 1 --speed 30   # 처음 가보는 경로
  python3 tools/transit_check.py --from 6 --to 1 --dry-run
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")


def fmt(pose):
    return "[" + ", ".join(f"{v:.1f}" for v in pose) + "]"


def confirm(msg):
    if input(f"{msg} (엔터=진행, n=중단) ").strip().lower().startswith("n"):
        raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", required=True,
                    help="출발 — 줄 번호(1/2/5/6) 또는 'holder'(피크 거치대). "
                         "거치대 출발은 피크를 반환한 직후 자세다 (#34)")
    ap.add_argument("--to", dest="dst", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--no-route", action="store_true",
                    help="경유지를 안 거치고 바로 간다. **거치대 직행을 볼 때만.** "
                         "노드는 지금 거치대에서 출발하면 항상 경유지를 탄다 (#34)")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--approach", type=float, default=None,
                    help="페그에서 물러나는 거리(mm). 기본은 motion.approach_mm")
    ap.add_argument("--src-sol", type=int, default=None,
                    help="출발 접근점의 팔 자세. yaml 의 sol 을 덮어쓴다")
    ap.add_argument("--dst-sol", type=int, default=None,
                    help="도착 접근점의 팔 자세. tools/sol_check.py 로 고른다")
    ap.add_argument("--speed", type=float, default=100.0,
                    help="설정 속도의 몇 %% 로 움직일지. "
                         "처음 가보는 경로는 30 쯤으로 낮춰라")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    if str(args.src).lower() == "holder":
        print("출발이 피크 거치대다 — 피크를 반환한 직후 팔이 남는 자리다.")
        print("  노드는 지금 거치대에서 출발하면 **항상 경유지를 탄다**(#34).")
        print("  직행을 보려면 --no-route 를 준다. 처음이면 --speed 30 으로.")
    elif TP.side_of(int(args.src)) == TP.side_of(args.dst):
        print(f"경고 — {args.src}번과 {args.dst}번은 같은 쪽이다. "
              f"실제로는 경유지를 안 거친다")

    back = args.approach if args.approach is not None \
        else points.motion.get("approach_mm", 100.0)
    # 출발이 거치대면 페그가 아니다. 피크를 반환한 직후 팔이 거기 남는다 (#34).
    # 그 자리에서 페그로 바로 가는 경로는 아직 아무도 안 봤다 —
    # 2026-09-19 충돌 검증은 페그↔페그와 경유지뿐이었다.
    from_holder = str(args.src).lower() == "holder"
    if from_holder:
        src_peg = points.pick_holder()
        args.src = "holder"
    else:
        src_peg = points.peg(int(args.src))
        args.src = int(args.src)
    dst_peg = points.peg(args.dst)
    src_sol = args.src_sol if args.src_sol is not None else getattr(src_peg, "sol", None)
    dst_sol = args.dst_sol if args.dst_sol is not None else dst_peg.sol
    src_back = TP.backed_off(src_peg.task, back)
    dst_back = TP.backed_off(dst_peg.task, back)
    route = points.route()
    if route is None:
        raise SystemExit("경유지(routes.transit)가 미교시다. 먼저 채워라")

    # 속도를 낮춘 설정 사본
    motion = dict(points.motion)
    motion.pop("transit_tcp_vel", None)  # 경로 점검의 --speed를 우선
    motion.pop("transit_tcp_acc", None)
    scale = max(0.05, args.speed / 100.0)
    motion["vel"] = points.motion.get("vel", 30) * scale
    motion["acc"] = points.motion.get("acc", 30) * scale
    motion["travel_vel"] = [v * scale for v in points.motion.get("travel_vel", [50.0, 30.0])]
    motion["travel_acc"] = [v * scale for v in points.motion.get("travel_acc", [100.0, 60.0])]

    print()
    print("=" * 62)
    src_label = "거치대" if from_holder else f"{args.src}번({TP.side_of(args.src)})"
    print(f"이동 경로 검증  {src_label} → {args.dst}번({TP.side_of(args.dst)})")
    print("=" * 62)
    print(f"  출발 접근점  {fmt(src_back)}  "
          f"({'거치대' if from_holder else '페그'}에서 {back:.0f}mm 뒤)")
    print(f"  경유지       {route.joints}"
          + ("   ← 건너뛴다 (--no-route)" if args.no_route else ""))
    print(f"  도착 접근점  {fmt(dst_back)}")
    print(f"  팔 자세(sol) 출발 {src_sol if src_sol is not None else '미지정'} / "
          f"도착 {dst_sol if dst_sol is not None else '미지정 — 현재 자세 유지'}")
    print(f"  속도         설정값의 {args.speed:.0f}% "
          f"(관절 {motion['vel']:.1f} deg/s)")
    print("  그리퍼       열어둔다(open 프리셋). 이동 중 가장 넓은 상태라 최악을 본다")

    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_transit_check", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import arm_pose
    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DR_common2 import posx
    from DSR_ROBOT2 import ikin

    def auto_sol(label, task, have):
        if have is not None:
            return have
        got = arm_pose.choose(lambda t: ikin(posx(task), t), route.joints[0])
        if got is None:
            raise SystemExit(f"{label} 은 쓸 만한 팔 모양이 없다. sol_check.py 로 보라")
        print(f"  {label} 팔 모양 자동 선택 {got[0]} "
              f"(손목 케이싱 X {got[2][0]:.0f}mm)")
        return got[0]

    src_sol = auto_sol(f"{args.src}번", src_back, src_sol)
    dst_sol = auto_sol(f"{args.dst}번", dst_back, dst_sol)
    from DSR_ROBOT2 import (get_current_posj, get_current_posx,
                            get_current_solution_space)
    from tunio_robot.kinematics import fk

    def report(label):
        posx = get_current_posx()
        pos = posx[0] if isinstance(posx, (list, tuple)) and len(posx) == 2 else posx
        posj = list(get_current_posj())
        print(f"    {label} 손끝 {fmt(list(pos)[:6])}")
        print(f"    {label} 관절 {fmt(posj)}   sol {get_current_solution_space()}")
        o = fk(posj)["origins"]
        print(f"    {label} 어깨팔꿈치 {fmt(o[2])}  손목중심 {fmt(o[3])}")

    notes = []
    try:
        confirm("\n그리퍼를 연다(open 프리셋). 손이 끼지 않게 치워라.")
        MP._grip("open")

        print("\n[1/3] 출발 접근점으로 간다. 지금 팔이 기타 근처면 주의해서 보라.")
        confirm("  진행할까")
        MP._movejx(src_back, motion, sol=src_sol)
        report("출발점")
        notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        if args.no_route:
            print("\n[2/3] **경유지를 건너뛴다 (--no-route).** 바로 도착점으로 간다.")
            print("      이게 거치대 직행을 보는 경우다. 아무도 본 적 없는 경로다 (#34).")
            print("      조금이라도 이상하면 즉시 비상정지.")
        else:
            print("\n[2/3] 경유지로 올라간다. **여기서 손목이 뒤집힐 수 있다.**")
            print("      손끝이 기타 쪽으로 쓸고 가는지 잘 보라.")
            confirm("  진행할까")
            MP.goto_route(route, motion)
            report("경유지")
            notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        print(f"\n[3/3] {args.dst}번 접근점으로 내려간다.")
        confirm("  진행할까")
        MP._movejx(dst_back, motion, sol=dst_sol)
        report("도착점")
        notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        print("\n돌아가는 길도 같은지 본다.")
        if input("돌아가 볼까? (y=진행) ").strip().lower().startswith("y"):
            confirm("  경유지로")
            MP.goto_route(route, motion)
            report("경유지")
            confirm(f"  {args.src}번 접근점으로")
            MP._movejx(src_back, motion, sol=src_sol)
            report("출발점")
            notes.append(input("  돌아올 때 문제 있었나? (없으면 엔터) ").strip())
    except KeyboardInterrupt:
        print("\n중단했다. 팔은 그 자리에 선다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    said = [n for n in notes if n]
    print(f"\n{'=' * 62}")
    print("결과")
    print("=" * 62)
    if said:
        for n in said:
            print(f"  · {n}")
        print("\n걸리는 데가 있으면 —")
        print("  · 손목 케이싱(J5 쪽)이 걸리면 **팔 자세(sol)** 를 바꾼다.")
        print("    손끝을 안 바꾸고 케이싱만 반대로 넘길 수 있다 —")
        print(f"      python3 tools/sol_check.py --peg {args.dst}")
        print("  · 페그에 너무 붙으면 motion.approach_mm 을 늘린다")
        print("  · 높이가 모자라면 routes.transit.j 의 J2·J3 를 바꾼다")
    elif args.speed < 100.0:
        print("  걸리는 데 없었다. 기본 속도로 다시 한 번 보고 확정하라.")
        print(f"  python3 tools/transit_check.py --from {args.src} "
              f"--to {args.dst}")
    else:
        print("  걸리는 데 없었다. 기본 속도로 본 것이니 이 경로는 확정이다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
