#!/usr/bin/env python3
"""피크 반환→높은 이동→페그 앞 하강만 검사. 기본은 계획, --execute도 단계별 확인."""
import argparse
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "tunio_robot"))


def confirm(message):
    if input(f"\n{message}\n진행은 y, 나머지는 중단: ").strip().lower() != "y":
        raise RuntimeError("사용자가 중단함 — 자동 복귀하지 않는다")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--string", type=int, choices=[1, 2, 5, 6], default=2)
    parser.add_argument("--yaml", default=str(ROOT / "src/tunio_robot/config/teach_points.yaml"))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--returned", action="store_true",
                        help="이미 피크를 반환하고 빈손으로 상승한 상태. 집기/반환 생략")
    parser.add_argument("--roll-deg", type=float, default=0.0,
                        help="접근 자세의 손잡이 각. 기본은 교시각 0도, 등록값 변경 없음")
    args = parser.parse_args()
    from tunio_robot import motion_primitives as MP
    from tunio_robot import teach_points as TP
    from tunio_robot import runlog as RL
    points = TP.load(args.yaml)
    motion = dict(points.motion)
    # 시험도 운영 코드와 같은 경로를 쓰되 이동 속도만 낮춘다.
    motion["speed_scale"] = min(float(motion.get("speed_scale", 1.0)), 0.2)
    import math
    if not math.isfinite(args.roll_deg):
        parser.error("roll-deg는 유한한 숫자여야 한다")
    pose = MP._rolled(points.peg(args.string), args.roll_deg)
    floor, margin = MP._peg_transfer_floor(motion)
    print(f"{args.string}번 페그 앞 경로 시험: roll={args.roll_deg}도, speed_scale=0.2 이하")
    print(f"피크 반환/수직 상승(Z≥{floor + 2 * margin:g}) → 높은 접근점 → 수직 하강")
    print(f"이동 중 TCP/J3~J6 모델 하한 {floor:g}mm + 임시 여유 {margin:g}mm")
    print("페그를 잡거나 돌리지 않으며, 접근점에서 끝난다. 자동 홈 복귀 없음.")
    print("기본 교시각 0도 시험. 실제 조절의 누적 roll이 다르면 경로도 달라질 수 있다.")
    if not args.execute:
        print("계획만 출력했다. 실기 실행은 --execute (각 단계에서 y 필요).")
        return 0

    import rclpy
    rclpy.init()
    probe = rclpy.create_node("pick_to_peg_check")
    log = {"tool": "pick_to_peg_check", "string": args.string, "roll_deg": args.roll_deg,
           "success": False, "stages": []}
    try:
        # 실행 중인 통합 제어기와 직접 API 도구가 동시에 팔을 움직이지 않게 한다.
        until = time.monotonic() + 2.0
        while time.monotonic() < until:
            rclpy.spin_once(probe, timeout_sec=0.1)
        names = {n for n, _ in probe.get_node_names_and_namespaces()}
        busy = names & {"robot_controller", "task_manager", "single_string_test"}
        if busy:
            raise RuntimeError(f"먼저 통합 노드를 종료하라: {sorted(busy)}")
        confirm("두산 브링업만 유지하고 다른 로봇 조작 도구를 종료했는지 확인. 실기 연결")
        MP.connect(points.robot, points.gripper, logger=probe.get_logger())
        standby = points.frame("standby")
        side = TP.side_of(args.string)
        holder = points.pick_holder()
        sol = holder.sol_by_side.get(side)
        if not args.returned:
            confirm("빈 그리퍼·거치대에 피크가 있는 상태에서 피크 집기 → 발현 대기 자세")
            MP.pick_up(holder, motion, standby=points.frame("pluck_standby"), via=standby,
                       sol=sol, route=points.route(), logger=probe.get_logger())
            log["stages"].append("picked")
            confirm("피크 반환 → 그리퍼 열기 → 수직 상승")
            MP.pick_return(holder, motion, via=standby, sol=sol,
                           route=points.route(), logger=probe.get_logger())
            log["stages"].append("returned")
        else:
            confirm("피크는 거치대, 그리퍼는 빈손·열림·상승 완료 상태인지 확인")
        MP._pick_to_peg_approach(pose, side, points.route(), motion,
                                 logger=probe.get_logger(), confirm=confirm)
        log["stages"].append("at_peg_approach")
        log["success"] = True
        print("페그 앞 접근점 도착. 파지/회전/자동 복귀 없이 종료한다.")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        log["error"] = str(exc) or "KeyboardInterrupt"
        print(f"중단: {log['error']}. 자동 회수 동작 없음.", file=sys.stderr)
        return 1
    finally:
        path = RL.save("pick_to_peg_check", log, os.path.expanduser("~/.tunio/robot_data"))
        print(f"기록: {path or '저장 실패'}")
        if MP._node is not None:
            MP._node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
