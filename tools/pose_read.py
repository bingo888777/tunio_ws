#!/usr/bin/env python3
"""
지금 자세 읽기 — 조그로 옮겨 놓고 값을 받아 적을 때 쓴다.

**로봇을 움직이지 않는다.** 읽기만 한다.

경유지를 계산으로 잡기는 어렵다. 조그로 직접 가보고 "이 정도면 안 걸리겠다"
싶은 자리에서 이 도구로 값을 읽는 편이 확실하다.

조그와 같이 쓸 수 있나
  ⚠️ **드라이버를 켜면 직접교시가 안 된다** (2026-09-19 확인). 드라이버가
  붙으면 로봇이 자동 모드로 잡혀 펜던트의 손으로 밀기가 막힌다. 둘 중
  하나만 된다. ROS 로 밀면서 값을 읽으려면 tools/jog.py 를 쓴다.

  드라이버를 끄고 펜던트로 교시하는 쪽이면 값은 펜던트 화면에서 읽으면 된다.
  드라이버가 떠 있는데 서비스 호출만 막히는 경우라면 --topic 으로
  joint_states 를 구독해서 읽는다. 대신 손끝 좌표와 sol 은 안 나오고
  관절값만 나온다. 경유지를 잡는 데는 관절값이면 충분하다.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/pose_read.py              # 엔터 칠 때마다 한 번씩 읽는다
  python3 tools/pose_read.py --watch      # 0.5초마다 계속 찍는다 (조그하며 보기)
  python3 tools/pose_read.py --watch --topic   # 수동 모드에서 서비스가 막힐 때
"""
import argparse
import sys
import time


def fmt(vals, unit=""):
    return "[" + ", ".join(f"{v:7.2f}" for v in vals) + f"]{unit}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--watch", action="store_true", help="계속 찍는다")
    ap.add_argument("--topic", action="store_true",
                    help="두산 서비스 대신 joint_states 토픽을 구독한다. "
                         "수동 모드에서 서비스가 막힐 때 쓴다")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_pose_read", namespace=args.id)
    DR_init.__dsr__node = node

    if args.topic:
        from sensor_msgs.msg import JointState
        import math as _m
        latest = {}

        def on_state(msg):
            latest["j"] = [_m.degrees(v) for v in msg.position[:6]]

        node.create_subscription(JointState, "joint_states", on_state, 10)

        def once():
            for _ in range(200):                   # 첫 메시지를 기다린다
                rclpy.spin_once(node, timeout_sec=0.05)
                if "j" in latest:
                    break
            if "j" not in latest:
                print("  joint_states 가 안 온다. 드라이버가 떠 있는지 보라")
                return None
            print(f"  관절 {fmt(latest['j'])} deg")
            print("  (토픽 모드라 손끝 좌표와 sol 은 안 나온다)")
            return latest["j"]

        return loop(args, node, once)

    try:
        from DSR_ROBOT2 import (get_current_posj, get_current_posx,
                                get_current_solution_space)
    except ImportError as e:
        print(f"두산 API import 실패: {e}")
        print("수동 모드라면 --topic 을 붙여 보라")
        return 1

    def once():
        posj = list(get_current_posj())
        posx = get_current_posx()
        pos = posx[0] if isinstance(posx, (list, tuple)) and len(posx) == 2 else posx
        sol = get_current_solution_space()
        print(f"  관절 {fmt(posj)} deg")
        print(f"  손끝 {fmt(list(pos)[:6])}  (앞 3개 mm, 뒤 3개 deg)")
        print(f"  팔 자세(sol) {sol}")
        return posj

    return loop(args, node, once)


def loop(args, node, once):
    """엔터마다 또는 주기마다 읽어서 찍는다."""
    import rclpy
    print("\n로봇을 움직이지 않는다. 읽기만 한다.")
    try:
        if args.watch:
            print("조그하면서 보라. Ctrl+C 로 끝낸다.\n")
            while True:
                once()
                print()
                time.sleep(args.period)
        else:
            print("엔터를 칠 때마다 한 번씩 읽는다. Ctrl+C 로 끝낸다.\n")
            last = None
            while True:
                input("엔터 ")
                last = once()
                print()
    except (KeyboardInterrupt, EOFError):
        print("\n끝냈다.")
        if not args.watch and last:
            j = [round(v, 2) for v in last]
            print("\n경유지로 쓰려면 teach_points.yaml 의 routes.transit 에 —")
            print(f"  전부 고정:  j: {j}")
            print(f"  J1·J2·J3만: j: [{j[0]}, {j[1]}, {j[2]}, null, null, null]")
            print(f"  J1만:       j: [{j[0]}, null, null, null, null, null]")
            print("  taught: true 로 바꾸는 것도 잊지 마라")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
