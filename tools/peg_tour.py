#!/usr/bin/env python3
"""
페그 순회 — 여러 페그를 차례로 잡아 본다. 조율 한 바퀴의 이동 부분 전체다.

6번 잡고 → 5번 잡고 → 1번 잡고 처럼 한 번에 돌린다. 좌우 전환도 그 안에 들어 있다.
**페그를 돌리지는 않는다.** 잡았다 놓기만 한다. 회전은 3단계에서 따로 본다.

팔 모양(sol)을 알아서 고른다
  같은 좌표를 갈 수 있는 팔 모양이 8가지다. 안 정해 주면 로봇이 그때그때 고르고
  매번 달라진다. yaml 의 pegs.<번호>.sol 이 있으면 그걸 쓰고, 없으면 ikin 으로
  8가지를 다 받아 arm_pose 규칙으로 고른다 (받침 안 파고들고, 경유지에서 멀리
  안 돌고, 손목 케이싱이 기타에서 먼 것). 고른 값은 끝날 때 찍어 준다.

한 페그마다
  (쪽이 바뀌면) 경유지 → 접근점 → 페그 → 파지 → 눈으로 확인 → 놓기 → 물러나기
  페그마다 한 번 확인을 받는다. 그 안에서는 안 멈춘다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/peg_tour.py                         # 6 5 1 차례로
  python3 tools/peg_tour.py --pegs 6 5 2 1          # 네 줄 전부
  python3 tools/peg_tour.py --no-grip               # 잡지 않고 이동만
  python3 tools/peg_tour.py --dry-run               # 좌표만 계산
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")


def fmt(vals, d=1):
    return "[" + ", ".join(f"{v:7.{d}f}" for v in vals) + "]"


def confirm(msg):
    if input(f"{msg} (엔터=진행, n=중단) ").strip().lower().startswith("n"):
        raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pegs", type=int, nargs="+", default=[6, 5, 1],
                    choices=[1, 2, 5, 6], help="돌 순서")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--approach", type=float, default=None)
    ap.add_argument("--speed", type=float, default=100.0,
                    help="설정 속도의 몇 %% 로. 처음 가보는 경로는 30 쯤으로 낮춰라")
    ap.add_argument("--preset", default="peg")
    ap.add_argument("--no-grip", action="store_true", help="잡지 않고 이동만")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import runlog
    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    back_mm = args.approach if args.approach is not None \
        else points.motion.get("approach_mm", 100.0)
    route = points.route()
    if route is None:
        raise SystemExit("경유지(routes.transit)가 미교시다. 먼저 채워라")
    route_j1 = route.joints[0]

    motion = dict(points.motion)
    scale = max(0.05, args.speed / 100.0)
    for key, default in (("vel", 30), ("acc", 30)):
        motion[key] = points.motion.get(key, default) * scale
    for key, default in (("travel_vel", [50.0, 30.0]), ("travel_acc", [100.0, 60.0])):
        motion[key] = [v * scale for v in points.motion.get(key, default)]

    plan = []
    side = None
    for n in args.pegs:
        peg = points.peg(n)
        if peg.kind != "task":
            raise SystemExit(f"{n}번이 작업 좌표가 아니다")
        cross = TP.side_of(n) != side
        side = TP.side_of(n)
        plan.append((n, peg, TP.backed_off(peg.task, back_mm), cross))

    print()
    print("=" * 70)
    print("페그 순회  " + " → ".join(str(n) for n, _, _, _ in plan))
    print("=" * 70)
    print(f"  접근 거리  페그에서 {back_mm:.0f}mm 뒤")
    print(f"  속도       설정값의 {args.speed:.0f}% (관절 {motion['vel']:.1f} deg/s)")
    print(f"  파지       {'안 한다 (이동만)' if args.no_grip else args.preset}")
    print(f"  경유지     {route.joints}")
    for n, peg, bk, cross in plan:
        mark = "  [경유지 거침]" if cross else ""
        print(f"  {n}번({TP.side_of(n)})  접근 {fmt(bk, 1)}  "
              f"sol {peg.sol if peg.sol is not None else '자동'}{mark}")
    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_peg_tour", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import arm_pose
    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DR_common2 import posx
    from DSR_ROBOT2 import (get_current_posj, get_current_solution_space, ikin)

    def choose_sol(n, task):
        """적힌 값이 있으면 그걸 쓰고, 없으면 8가지를 받아 고른다."""
        peg = points.peg(n)
        if peg.sol is not None:
            return peg.sol, f"yaml 에 적힌 값 {peg.sol}"
        got = arm_pose.choose(lambda t: ikin(posx(list(task)), t), route_j1)
        if got is None:
            raise RuntimeError(f"{n}번은 쓸 만한 팔 모양이 없다. sol_check.py 로 보라")
        sol, _, casing = got
        return sol, f"자동 선택 {sol} (손목 케이싱 X {casing[0]:.0f}mm)"

    notes = []
    chosen = {}
    log = {"tool": "peg_tour", "pegs": list(args.pegs),
           "gripped": not args.no_grip, "approach_mm": back_mm,
           "speed_pct": args.speed, "visits": []}
    try:
        confirm("\n그리퍼를 연다(open 프리셋). 손이 끼지 않게 치워라.")
        MP._grip("open")
        print("\n팔이 기타에 닿아 있거나 페그를 물고 있으면 먼저 빼내라.")
        confirm("경유지로 올라간다. 팔이 크게 움직인다.")
        MP.goto_route(route, motion)

        for n, peg, bk, cross in plan:
            print(f"\n{'-' * 70}")
            print(f"{n}번 페그 ({TP.side_of(n)} 쪽)")
            sol, why = choose_sol(n, bk)
            chosen[n] = sol
            print(f"  팔 모양  {why}")
            if cross:
                print("  경유지를 거쳐 반대쪽으로 넘어간다")
            confirm("  진행할까")

            if cross:
                MP.goto_route(route, motion)
            MP._movejx(bk, motion, sol=sol)             # 접근점 (관절 보간)
            MP._movel(list(peg.task), motion)           # 페그 (직선)

            posj = [round(v, 2) for v in get_current_posj()]
            print(f"  관절 {fmt(posj, 2)}   sol {get_current_solution_space()}")
            if not args.no_grip:
                MP._grip(args.preset)
                print(f"  '{args.preset}' 로 물었다. 눈으로 보라 — "
                      f"제대로 들어갔나, 헐거운가, 빗나갔나")
            said = input("  어떤가? (문제없으면 엔터) ").strip()
            log["visits"].append({"peg": n, "side": TP.side_of(n), "sol": sol,
                                  "sol_source": why, "crossed": cross,
                                  "approach": list(bk), "peg_task": list(peg.task),
                                  "joints_at_peg": posj,
                                  "solution_space": get_current_solution_space(),
                                  "note": said})
            if said:
                notes.append(f"{n}번: {said}")

            if not args.no_grip:
                MP._release()
            MP._movel(bk, motion)                       # 들어온 직선 그대로

        print(f"\n{'-' * 70}")
        confirm("다 돌았다. 경유지 거쳐 대기 자세로 돌아갈까")
        MP.goto_route(route, motion)
        MP._movej(points.frame("standby").joints, motion)
        print("대기 자세로 돌아왔다.")
    except KeyboardInterrupt:
        print("\n중단했다. 그리퍼를 열고 멈춘다. 팔은 그 자리에 선다.")
        try:
            MP._release()
        except Exception as e:                          # noqa: B902
            print(f"  여는 데 실패: {e}")
    except Exception as e:                              # noqa: B902
        print(f"\n실패: {e}")
        try:
            MP._release()
        except Exception:                               # noqa: B902
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'=' * 70}")
    print("결과")
    print("=" * 70)
    if notes:
        for t in notes:
            print(f"  · {t}")
    else:
        print("  적어둔 문제 없음")
    if chosen:
        log["chosen_sol"] = chosen
        print("\n고른 팔 모양 — teach_points.yaml 의 pegs.<번호>.sol 에 적어라.")
        print("  null 로 두면 매번 달라진다.")
        for n in sorted(chosen):
            print(f"    pegs.{n}.sol: {chosen[n]}")
    path = runlog.save("tour_" + "-".join(str(n) for n in args.pegs), log, LOG_DIR)
    if path:
        print(f"\n기록 저장: {os.path.relpath(path)}")
        print("  (이제 화면을 놓쳐도 sol 값이 파일에 남는다)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
