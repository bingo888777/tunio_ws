#!/usr/bin/env python3
"""
좌우 전환 검증 — 로봇 점검 체크리스트 2.5단계.

한쪽 페그에서 반대쪽으로 넘어갈 때 기타나 주변 물체를 안 치는지 본다.
**페그를 잡지 않는다.** 이동만 한다.

왜 따로 보는가
  왼쪽 페그는 B=+90, 오른쪽은 B=-90 이라 손목이 180도 뒤집혀야 한다.
  TCP 가 208mm 라 그 뒤집힘에 손끝이 반경 20cm 짜리 호를 그린다.
  게다가 이동 중에는 그리퍼가 열려 있어(90mm) 폭도 제일 넓다.
  페그를 잡는 것보다 이쪽이 부딪힐 위험이 크다.

어떻게 보는가
  한 구간 가고 멈춘다. 눈으로 보고, 괜찮으면 다음 구간.
  중간에 아니다 싶으면 n 이나 Ctrl+C. 팔은 그 자리에 선다.
  처음에는 --speed 30 쯤으로 느리게 하고, 비상정지에 손을 올려두고 본다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/transit_check.py --from 6 --to 1 --speed 30
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
    ap.add_argument("--from", dest="src", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--to", dest="dst", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--approach", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=30.0,
                    help="설정 속도의 몇 %% 로 움직일지. 처음에는 30 쯤으로")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    if TP.side_of(args.src) == TP.side_of(args.dst):
        print(f"경고 — {args.src}번과 {args.dst}번은 같은 쪽이다. "
              f"실제로는 경유지를 안 거친다")

    src_back = TP.backed_off(points.peg(args.src).task, args.approach)
    dst_back = TP.backed_off(points.peg(args.dst).task, args.approach)
    route = points.route()
    if route is None:
        raise SystemExit("경유지(routes.transit)가 미교시다. 먼저 채워라")

    # 속도를 낮춘 설정 사본
    motion = dict(points.motion)
    scale = max(0.05, args.speed / 100.0)
    motion["vel"] = points.motion.get("vel", 30) * scale
    motion["acc"] = points.motion.get("acc", 30) * scale
    motion["travel_vel"] = [v * scale for v in points.motion.get("travel_vel", [50.0, 30.0])]
    motion["travel_acc"] = [v * scale for v in points.motion.get("travel_acc", [100.0, 60.0])]

    print()
    print("=" * 62)
    print(f"좌우 전환 검증  {args.src}번({TP.side_of(args.src)}) "
          f"→ {args.dst}번({TP.side_of(args.dst)})")
    print("=" * 62)
    print(f"  출발 접근점  {fmt(src_back)}")
    print(f"  경유지       {route.joints}")
    print(f"  도착 접근점  {fmt(dst_back)}")
    print(f"  속도         설정값의 {args.speed:.0f}% "
          f"(관절 {motion['vel']:.1f} deg/s)")
    print("  그리퍼       열어둔다(90mm). 이동 중 가장 넓은 상태라 최악을 본다")

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

    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DSR_ROBOT2 import get_current_posj, get_current_posx

    def report(label):
        posx = get_current_posx()
        pos = posx[0] if isinstance(posx, (list, tuple)) and len(posx) == 2 else posx
        print(f"    {label} 손끝 {fmt(list(pos)[:6])}")
        print(f"    {label} 관절 {fmt(get_current_posj())}")

    notes = []
    try:
        confirm("\n그리퍼를 연다(90mm). 손이 끼지 않게 치워라.")
        MP._grip("open")

        print("\n[1/3] 출발 접근점으로 간다. 지금 팔이 기타 근처면 주의해서 보라.")
        confirm("  진행할까")
        MP._movejx(src_back, motion, sol=points.peg(args.src).sol)
        report("출발점")
        notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        print("\n[2/3] 경유지로 올라간다. **여기서 손목이 뒤집힐 수 있다.**")
        print("      손끝이 기타 쪽으로 쓸고 가는지 잘 보라.")
        confirm("  진행할까")
        MP.goto_route(route, motion)
        report("경유지")
        notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        print(f"\n[3/3] {args.dst}번 접근점으로 내려간다.")
        confirm("  진행할까")
        MP._movejx(dst_back, motion, sol=points.peg(args.dst).sol)
        report("도착점")
        notes.append(input("  부딪힐 뻔한 데 있었나? (없으면 엔터) ").strip())

        print("\n돌아가는 길도 같은지 본다.")
        if input("돌아가 볼까? (y=진행) ").strip().lower().startswith("y"):
            confirm("  경유지로")
            MP.goto_route(route, motion)
            report("경유지")
            confirm(f"  {args.src}번 접근점으로")
            MP._movejx(src_back, motion, sol=points.peg(args.src).sol)
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
        print("\n걸리는 데가 있으면 경유지를 고친다 —")
        print("  · 손끝이 기타를 쓸면 J5 를 묶어서 안전한 데서 뒤집게 한다")
        print("  · 높이가 모자라면 J2·J3 를 바꾼다")
        print("  routes.transit.j 에서 null 을 값으로 바꾸면 된다")
    else:
        print("  걸리는 데 없었다. 속도를 올려 다시 한 번 보고 확정하라.")
        print(f"  python3 tools/transit_check.py --from {args.src} "
              f"--to {args.dst} --speed 100")
    return 0


if __name__ == "__main__":
    sys.exit(main())
