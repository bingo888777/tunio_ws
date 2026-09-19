#!/usr/bin/env python3
"""
페그 파지 확인 — 로봇 점검 체크리스트 2단계.

21mm 프리셋으로 페그가 실제로 물리는지, 깊이는 얼마가 맞는지 본다.

**팔이 움직인다.** 1단계와 다르다. 단계마다 확인을 받고 한 번에 한 동작만 한다.
비상정지를 손 닿는 곳에 두고 시작하라.

경로
  물러난 지점(접근축 위에서 뒤로) → 직선으로 페그 앞 → 파지 → 해제 → 같은 직선으로 후퇴
  물러난 지점까지 가는 것은 **사람이 조그로** 한다. 그 구간은 경로가 길어서
  자동으로 보내면 기타를 쓸고 갈 수 있다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/peg_grip_check.py --peg 6
  python3 tools/peg_grip_check.py --peg 6 --depth -77.0   # 깊이를 바꿔 시험
  python3 tools/peg_grip_check.py --peg 6 --dry-run       # 좌표만 계산해 보기
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
    ans = input(f"{msg} (엔터=진행, n=중단) ").strip().lower()
    if ans.startswith("n"):
        raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--approach", type=float, default=60.0,
                    help="접근축을 따라 뒤로 물러나는 거리(mm)")
    ap.add_argument("--depth", type=float, default=None,
                    help="깊이를 바꿔 시험한다. 접근축 좌표를 이 값으로 덮어쓴다")
    ap.add_argument("--preset", default="peg", help="쓸 그리퍼 프리셋 이름")
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
        # 접근축 성분이 가장 큰 축을 깊이 축으로 본다 (왼쪽은 Y, 오른쪽도 Y)
        axis = TP.approach_axis(target)
        k = max(range(3), key=lambda i: abs(axis[i]))
        print(f"깊이 축은 {'XYZ'[k]} 다. {target[k]:.2f} → {args.depth:.2f} 로 바꾼다")
        target[k] = args.depth
    back = TP.backed_off(target, args.approach)

    print()
    print("=" * 62)
    print(f"{args.peg}번 페그 파지 확인")
    print("=" * 62)
    print(f"  물러난 지점  {fmt(back)}")
    print(f"  페그 앞      {fmt(target)}")
    print(f"  접근 거리    {args.approach:.0f} mm")
    print(f"  프리셋       {args.preset} "
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

    print("\n" + "-" * 62)
    print("먼저 **사람이 조그로** 물러난 지점 근처까지 팔을 옮겨라.")
    print("그 구간은 경로가 길어서 자동으로 보내면 기타를 쓸고 갈 수 있다.")
    print(f"  목표: {fmt(back)}")
    print("-" * 62)

    held = None
    try:
        confirm("\n조그 끝났나? 이제 물러난 지점으로 정밀 이동한다.")
        MP._movel(back, points.motion)

        confirm("페그 앞까지 직선으로 들어간다. 그리퍼는 아직 안 닫는다.")
        MP._movel(target, points.motion)

        confirm(f"'{args.preset}' 프리셋으로 닫는다.")
        MP._grip(args.preset)

        print("\n눈으로 확인하라 —")
        print("  · 페그가 손가락 사이에 제대로 들어왔나")
        print("  · 헐거워서 흔들리나, 아니면 꽉 물렸나")
        print("  · 빗나가서 헛물었나")
        held = input("\n어떤가? (보이는 대로 적어라) ").strip()

        confirm("\n놓고 물러난다.")
        MP._release()
        MP._movel(back, points.motion)
        print("\n물러났다.")
    except KeyboardInterrupt:
        print("\n중단했다. 그리퍼를 열고 멈춘다.")
        try:
            MP._release()
        except Exception as e:                      # noqa: B902
            print(f"  여는 데 실패: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if held:
        print(f"\n{'=' * 62}")
        print(f"결과 — {args.peg}번, 깊이 {target[max(range(3), key=lambda i: abs(TP.approach_axis(target)[i]))]:.2f}")
        print(f"{'=' * 62}")
        print(f"  {held}")
        print()
        print("헐거우면 웹에서 폭을 1mm 줄이고, 빗나가면 1mm 늘려서 다시.")
        print("깊이가 안 맞으면 --depth 로 값을 바꿔가며 다시 해보라.")
        print("확정되면 teach_points.yaml 에 적는다 —")
        print("  폭: gripper.presets.peg.width_mm")
        print("  깊이: 같은 쪽 페그 네 개의 좌표를 같은 값으로")
    return 0


if __name__ == "__main__":
    sys.exit(main())
