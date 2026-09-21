#!/usr/bin/env python3
"""
그리퍼 배선 확인 — 로봇 점검 체크리스트 1단계.

로봇의 디지털 출력 번호가 Compute Box(WebLogic)의 입력 번호와 같은지 본다.
코드는 같다고 가정하고 짜여 있다. 확인한 적은 없다.

**팔은 움직이지 않는다.** 디지털 출력만 켜고 끈다. 그리퍼만 여닫힌다.
그래도 손가락이 물리면 다치니 그리퍼 주변을 비우고 시작한다.

이 스크립트는 두산 연결의 첫 시험도 겸한다. 여기서 막히면 팔을 움직이기 전에
연결 문제부터 잡는 게 낫다.

준비
  1. 티치펜던트에서 로봇을 **자동 모드**로 (수동 대기 상태면 ROS 명령이 안 먹는다)
  2. WebLogic 프로그램이 돌고 있어야 한다 (웹 화면에 "프로그램이 작동 중입니다")
  3. 드라이버 실행 —
     ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
         mode:=real host:=<로봇IP> port:=12345 model:=m0609 name:=dsr01

실행
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/io_check.py

  python3 tools/io_check.py --outputs 1 2 3 4   # 다른 번호까지 훑어볼 때
"""
import argparse
import sys
import time

# WebLogic 에 설정된 규칙 (2026-09-19 화면 기준).
# 출력 하나만 켰을 때 그리퍼가 이렇게 되어야 한다.
EXPECTED = {
    1: "15mm 로 끝까지 닫힘",
    2: "열기 — 지금 웹로직 설정은 38mm (teach_points.yaml gripper.presets.open)",
    3: "21mm — 페그 잡는 폭. 15mm 보다 눈에 띄게 덜 닫힘",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--outputs", type=int, nargs="+", default=[1, 2, 3],
                    help="확인할 디지털 출력 번호")
    ap.add_argument("--settle", type=float, default=1.5,
                    help="출력을 건 뒤 기다리는 시간(초)")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model

    rclpy.init()
    # 네임스페이스가 있어야 한다. DSR_ROBOT2 의 서비스 이름이 상대 경로라
    # /dsr01/dsr_controller2/... 로 풀려야 명령이 로봇에 닿는다
    node = rclpy.create_node("tunio_io_check", namespace=args.id)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import set_digital_output
    except ImportError as e:
        print(f"두산 API import 실패: {e}")
        print("드라이버가 떠 있는지, 워크스페이스를 source 했는지 확인하라")
        return 1

    print()
    print("=" * 62)
    print("그리퍼 배선 확인")
    print("=" * 62)
    print("팔은 안 움직인다. 그리퍼만 여닫힌다.")
    print("손가락이 물리지 않게 그리퍼 주변을 비우고 시작하라.")
    input("\n준비되면 엔터 (중단은 Ctrl+C) ")

    def clear():
        for i in args.outputs:
            set_digital_output(i, 0)

    results = {}
    try:
        print("\n먼저 모든 출력을 0 으로 내린다 (어느 규칙과도 안 맞아 그대로 멈춘다)")
        clear()
        time.sleep(args.settle)

        for idx in args.outputs:
            print(f"\n{'-' * 62}")
            print(f"출력 {idx} 번만 켠다")
            expect = EXPECTED.get(idx)
            if expect:
                print(f"  기대: {expect}")
            else:
                print("  기대: 설정된 규칙 없음 — 아무 변화 없어야 정상")
            clear()
            set_digital_output(idx, 1)
            time.sleep(args.settle)
            answer = input("  실제로 어떻게 됐나? (그대로 보이는 대로 적어라) ").strip()
            results[idx] = answer
    except KeyboardInterrupt:
        print("\n중단했다")
    finally:
        print("\n출력을 전부 내린다")
        try:
            clear()
        except Exception as e:                      # noqa: B902
            print(f"  내리기 실패: {e}")
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'=' * 62}")
    print("결과")
    print("=" * 62)
    for idx in args.outputs:
        got = results.get(idx, "(확인 안 함)")
        print(f"  출력 {idx} → {got}")
        if idx in EXPECTED:
            print(f"     기대했던 것: {EXPECTED[idx]}")
    print()
    print("기대와 다르면 로봇 출력 번호와 WebLogic 입력 번호가 어긋난 것이다.")
    print("teach_points.yaml 의 gripper.presets.<이름>.outputs 를 실제 번호로 고친다.")
    print("아무 출력에도 반응이 없으면 배선이나 WebLogic 실행 상태부터 보라.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
