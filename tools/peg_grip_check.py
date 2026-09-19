#!/usr/bin/env python3
"""
페그 파지 확인 — 로봇 점검 체크리스트 2단계.

21mm 프리셋으로 페그가 실제로 물리는지, 깊이는 얼마가 맞는지 본다.
경유지를 거쳐 반대쪽으로 넘어가는 것도 이걸로 확인된다 (6번 다음 1번을 돌려보면 된다).

**팔이 움직인다.** 단계마다 확인을 받고 한 번에 한 동작만 한다.
비상정지를 손 닿는 곳에 두고 시작하라.

경로
  1. 경유지로     movej  — 관절 이동이라 특이점과 무관하다
  2. 접근점으로   movejx — 관절 보간. 경유지가 특이점 자세라 직선으로 못 나간다
  3. 페그로       movel  — 직선 60mm. 페그에 똑바로 들어가야 한다
  나올 때는 3 → 2 → 1 역순. 들어온 직선 그대로 나온다.

팔 자세(sol)
  movejx 는 역기구학 해를 골라야 한다. 엉뚱한 값이면 팔이 통째로 뒤집힌다.
  teach_points.yaml 의 pegs.<번호>.sol 을 쓰고, 없으면 --sol 로 준다.
  둘 다 없으면 지금 자세를 유지하려 하는데, 경유지가 특이점이라 그 값이
  모호할 수 있다. 처음 한 번은 --jog 로 사람이 옮겨서 sol 을 받아 적어라.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/peg_grip_check.py --peg 6
  python3 tools/peg_grip_check.py --peg 1 --sol 2      # 팔 자세를 직접 지정
  python3 tools/peg_grip_check.py --peg 6 --depth -76  # 깊이를 바꿔 시험
  python3 tools/peg_grip_check.py --peg 6 --jog        # 접근점까지 사람이 조그
  python3 tools/peg_grip_check.py --peg 6 --dry-run    # 좌표만 계산
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")


def fmt(pose):
    return "[" + ", ".join(f"{v:.2f}" for v in pose) + "]"


def confirm(msg):
    if input(f"{msg} (엔터=진행, n=중단) ").strip().lower().startswith("n"):
        raise KeyboardInterrupt


def depth_axis(TP, task):
    """접근축 성분이 가장 큰 축. 그게 '얼마나 깊이 물렸나' 축이다."""
    axis = TP.approach_axis(task)
    return max(range(3), key=lambda i: abs(axis[i]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--approach", type=float, default=60.0,
                    help="접근축을 따라 뒤로 물러나는 거리(mm)")
    ap.add_argument("--depth", type=float, default=None,
                    help="깊이를 바꿔 시험한다. 접근축 좌표를 이 값으로 덮어쓴다")
    ap.add_argument("--sol", type=int, default=None, help="팔 자세(역기구학 해)")
    ap.add_argument("--preset", default="peg", help="쓸 그리퍼 프리셋 이름")
    ap.add_argument("--jog", action="store_true",
                    help="경유지를 안 쓰고 접근점까지 사람이 조그로 옮긴다")
    ap.add_argument("--dry-run", action="store_true", help="좌표만 계산하고 안 움직인다")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    peg = points.peg(args.peg)
    if peg.kind != "task":
        raise SystemExit(f"{args.peg}번이 작업 좌표가 아니다. 이 스크립트는 작업 좌표용이다")

    target = list(peg.task)
    if args.depth is not None:
        k = depth_axis(TP, target)
        print(f"깊이 축은 {'XYZ'[k]} 다. {target[k]:.2f} → {args.depth:.2f} 로 바꾼다")
        target[k] = args.depth
    back = TP.backed_off(target, args.approach)
    sol = args.sol if args.sol is not None else peg.sol
    route = points.route()

    print()
    print("=" * 62)
    print(f"{args.peg}번 페그 파지 확인  ({TP.side_of(args.peg)} 쪽)")
    print("=" * 62)
    if not args.jog:
        if route is None:
            raise SystemExit(
                "경유지(routes.transit)가 미교시라 자동으로 갈 수 없다. "
                "--jog 로 사람이 옮기거나 경유지를 먼저 채워라")
        print(f"  1. 경유지     {route.joints}  (null 은 현재값 유지)")
    print(f"  2. 접근점     {fmt(back)}   ({args.approach:.0f}mm 뒤)")
    print(f"  3. 페그       {fmt(target)}")
    print(f"  팔 자세(sol)  {sol if sol is not None else '미지정 — 현재 자세 유지'}")
    print(f"  프리셋        {args.preset} "
          f"({points.gripper['presets'][args.preset].get('width_mm')}mm)")

    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_grip_check", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DSR_ROBOT2 import get_current_posj, get_current_solution_space

    held = None
    try:
        if args.jog:
            print("\n" + "-" * 62)
            print("**사람이 조그로** 접근점 근처까지 팔을 옮겨라.")
            print(f"  목표: {fmt(back)}")
            print("-" * 62)
            confirm("\n조그 끝났나? 접근점으로 정밀 이동한다.")
            MP._movel(back, points.motion)
        else:
            print("\n지금 팔이 기타에 닿아 있거나 페그를 물고 있으면 먼저 빼내라.")
            confirm("\n경유지로 올라간다. 팔이 크게 움직인다.")
            MP.goto_route(route, points.motion)

            confirm("접근점으로 간다. 직선이 아니라 관절 보간이라 경로가 휜다.")
            MP._movejx(back, points.motion, sol=sol)

        confirm("페그 앞까지 직선으로 들어간다. 그리퍼는 아직 안 닫는다.")
        MP._movel(target, points.motion)

        # 이 자세의 관절값과 팔 자세를 기록해 둔다
        posj = [round(v, 2) for v in get_current_posj()]
        now_sol = get_current_solution_space()
        print(f"\n  지금 관절값  {posj}")
        print(f"  팔 자세(sol) {now_sol}   <- pegs.{args.peg}.sol 에 적어라")

        confirm(f"\n'{args.preset}' 프리셋으로 닫는다.")
        MP._grip(args.preset)

        print("\n눈으로 확인하라 —")
        print("  · 페그가 손가락 사이에 제대로 들어왔나")
        print("  · 헐거워서 흔들리나, 아니면 꽉 물렸나")
        print("  · 빗나가서 헛물었나")
        held = input("\n어떤가? (보이는 대로 적어라) ").strip()

        confirm("\n놓고 물러난다.")
        MP._release()
        MP._movel(back, points.motion)
        if not args.jog:
            confirm("경유지로 돌아간다.")
            MP.goto_route(route, points.motion)
        print("\n끝났다.")
    except KeyboardInterrupt:
        print("\n중단했다. 그리퍼를 열고 멈춘다. 팔은 그 자리에 선다.")
        try:
            MP._release()
        except Exception as e:                      # noqa: B902
            print(f"  여는 데 실패: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if held:
        k = depth_axis(TP, target)
        print(f"\n{'=' * 62}")
        print(f"결과 — {args.peg}번, 깊이({'XYZ'[k]}) {target[k]:.2f}")
        print(f"{'=' * 62}")
        print(f"  {held}")
        print()
        print("헐거우면 웹에서 폭을 1mm 줄이고, 빗나가면 1mm 늘려서 다시.")
        print("깊이가 안 맞으면 --depth 로 값을 바꿔가며 다시 해보라.")
        print("확정되면 teach_points.yaml 에 적는다 —")
        print("  폭: gripper.presets.peg.width_mm")
        print("  깊이: 같은 쪽 페그들의 좌표를 같은 값으로")
        print(f"  팔 자세: pegs.{args.peg}.sol = {now_sol}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
