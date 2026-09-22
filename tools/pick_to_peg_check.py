#!/usr/bin/env python3
"""피크 반환→높은 이동→페그 앞 하강만 검사. 기본은 계획, --execute도 단계별 확인."""
import argparse
import math
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
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--check-only", action="store_true",
                      help="실기 좌표/IK/경로 검사만 수행. 이동·그리퍼 명령 없음")
    parser.add_argument("--returned", action="store_true",
                        help="이미 피크를 반환한 빈손 상태. 집기/반환 생략, 부족한 높이는 수직 상승")
    parser.add_argument("--roll-deg", type=float, default=0.0,
                        help="접근 자세의 손잡이 각. 기본은 교시각 0도, 등록값 변경 없음")
    parser.add_argument("--speed-scale", type=float, default=None,
                        help="시험 이동 배율. 생략하면 YAML의 공통 이동 배율 사용")
    args = parser.parse_args()
    from tunio_robot import motion_primitives as MP
    from tunio_robot import teach_points as TP
    from tunio_robot import runlog as RL
    points = TP.load(args.yaml)
    motion = dict(points.motion)
    if args.speed_scale is not None:
        if not math.isfinite(args.speed_scale) or args.speed_scale <= 0:
            parser.error("speed-scale은 유한한 양수여야 한다")
        motion["speed_scale"] = args.speed_scale
    if not math.isfinite(args.roll_deg):
        parser.error("roll-deg는 유한한 숫자여야 한다")
    pose = MP._rolled(points.peg(args.string), args.roll_deg)
    floor, margin = MP._peg_transfer_floor(motion)
    print(f"{args.string}번 페그 앞 경로 시험: roll={args.roll_deg}도, "
          f"speed_scale={motion.get('speed_scale', 1.0):g}")
    if args.check_only:
        print("현재 좌표·목표 IK·높이 검사만 수행. 상승/이동/그리퍼 명령은 보내지 않는다.")
    else:
        print(f"피크 반환/수직 상승(Z≥{floor + 2 * margin:g}) → 높은 접근점 → 수직 하강 → 제자리 자세 변경")
    print(f"이동 중 TCP/J3~J6 모델 하한 {floor:g}mm + 임시 여유 {margin:g}mm")
    print("페그를 잡거나 돌리지 않으며, 접근점에서 끝난다. 자동 홈 복귀 없음.")
    print("기본 교시각 0도 시험. 실제 조절의 누적 roll이 다르면 경로도 달라질 수 있다.")
    if not (args.execute or args.check_only):
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
        if args.check_only:
            log["diagnostic"] = MP._pick_to_peg_approach(
                pose, TP.side_of(args.string), points.route(), motion,
                logger=probe.get_logger(), execute=False)
            log["success"] = True
            print("현재 자세 기준 모델 검사 통과. 이동/그리퍼 동작 없음. 실기 무충돌 보증 아님.")
            return 0
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
            confirm("피크는 거치대, 그리퍼는 빈손·열림 상태인지 확인")
            current, _ = MP.read_posx()
            lift = list(current)
            if len(lift) != 6 or not all(math.isfinite(v) for v in lift):
                raise RuntimeError("현재 위치를 읽지 못해 상승 중단")
            if lift[2] < floor + 2 * margin:
                confirm(f"현재 Z={lift[2]:.2f} → {floor + 2 * margin:.2f}mm 수직 상승")
                lift[2] = floor + 2 * margin
                MP._movel_base(lift, 10.0, 20.0)
                MP._check_pick_return_position(lift, motion, "시험 시작 상승", probe.get_logger())
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
