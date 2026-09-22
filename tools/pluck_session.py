#!/usr/bin/env python3
"""
pluck_session — 오디오 측정을 위해 로봇이 줄을 반복해서 튕긴다 (이슈 #32).

누가 쓰나
  **정정인(마이크·분석)** 이 로봇 앞에서 돌린다. 이 도구는 튕기기만 하고 소리는
  안 듣는다. 측정은 tunio_audio 쪽에서 따로 한다.

  여기서 나와야 할 것 (#32 · #6 · #7) —
    · 로봇 발현 + 로봇 구동 상태에서 4줄 돌출 실측 (특히 6번 E2)
    · 돌출 문턱 20dB 유지할지 22dB 로 올릴지
    · 타현 세기 목표 — "4줄 돌출 >= 20dB 를 만족하는 세팅"
    · 5·6번 y축 미세 조정

시작 전에
  1. 펜던트에서 TCP `GripperDA_v1` · 툴 `Tool Weight` 를 켠다.
     (connect() 가 켜려고도 하지만 거부될 수 있다 — #23)
  2. 거치대에 피크를 놓아 둔다. 이 도구가 집고 마지막에 돌려놓는다.
  3. 브링업 터미널과 같은 ROS_DOMAIN_ID 인지 확인한다 (#21). 시작할 때 찍어 준다.

쓰는 법
  python3 tools/pluck_session.py --strings 6 5 2 1 --repeat 10
  python3 tools/pluck_session.py --string 6 --repeat 5 --gap 4.0
  python3 tools/pluck_session.py --string 5 --repeat 3 --dy -1.5   # y축 미세 조정
  python3 tools/pluck_session.py --strings 6 5 2 1 --step          # 한 번씩 확인하며
  python3 tools/pluck_session.py --strings 6 --dry-run             # 안 움직이고 계획만

y축 미세 조정 (--dy)
  5·6번은 y축 조정이 남아 있다 (#24 커밋). --dy 로 접촉점을 Base Y 방향으로
  밀어 보고, 소리가 제일 좋은 값을 찾으면 그 값을 알려 달라. teach_points.yaml
  의 pluck.<줄>.x 두 번째 숫자에 더하면 된다. 이 도구는 YAML 을 안 고친다.

기록
  튕길 때마다 시각을 남긴다 (docs/robot_data/). 녹음과 맞춰 볼 수 있게
  벽시계와 시작 이후 경과 초를 같이 적는다.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..",
                            "src", "tunio_robot", "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")

# pluck_string 이 motion.get(키, 기본값) 으로 읽는 값. 성현이 실기로 맞춘 것이라
# 그대로 쓴다. 확정되면 teach_points.yaml 의 motion 절로 옮긴다
PLUCK_MOTION = {
    "pluck_up_mm": 20.0,
    "pluck_stroke_tool": [-8.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pluck_retract_tool": [0.0, 0.0, -8.0, 0.0, 0.0, 0.0],
    "pluck_vel_approach": 100, "pluck_acc_approach": 100,
    "pluck_vel_fine": 20, "pluck_acc_fine": 50,
    "pluck_vel_stroke": 100, "pluck_acc_stroke": 200,
    "pick_up_mm": 50.0,
    "pick_vel_approach": 100, "pick_acc_approach": 100,
    "pick_vel_fine": 30, "pick_acc_fine": 50,
}


def say_env():
    """브링업과 같은 도메인인지 한눈에 (#21 재발 방지)."""
    keys = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "ROS_SUPER_CLIENT", "ROS_LOCALHOST_ONLY")
    print("ROS 환경:", "  ".join(f"{k}={os.environ.get(k, '(없음)')}" for k in keys))
    print("  ↑ 브링업 터미널과 같아야 한다. 다르면 서비스가 하나도 안 보인다")


def confirm(msg):
    print(msg)
    if input("    >>> 엔터 = 진행 / n = 중단: ").strip().lower() == "n":
        raise SystemExit("사용자가 중단했다.")


def load_pluck(points, no, dy):
    """
    발현 자세를 꺼낸다. **taught:false 여도 꺼낸다.**

    taught 를 올리려면 이 측정이 먼저 끝나야 하는데(#32), 조회를 거부하면
    측정을 못 한다. 그래서 여기서는 통과시키고 화면에 크게 알린다.
    """
    pose = points._plucks.get(no)
    if pose is None:
        raise SystemExit(f"teach_points.yaml 에 pluck.{no} 가 없다")
    points._check_tcp(pose)                 # 좌표 기준은 그대로 따진다 (#21)
    if dy:
        pose.task = list(pose.task)
        pose.task[1] += dy                  # Base Y 로 밀어 본다
    return pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--string", type=int, choices=[1, 2, 5, 6])
    ap.add_argument("--strings", type=int, nargs="+", choices=[1, 2, 5, 6])
    ap.add_argument("--repeat", type=int, default=10, help="줄마다 몇 번 튕길지")
    ap.add_argument("--gap", type=float, default=4.0,
                    help="튕기고 다음까지 기다리는 초. 소리가 잦아들 시간 (기본 4.0)")
    ap.add_argument("--dy", type=float, default=0.0,
                    help="접촉점을 Base Y 로 이만큼 민다(mm). 5·6번 미세 조정용")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--no-pick", action="store_true",
                    help="피크를 이미 쥐고 있다. 집기·반환을 건너뛴다")
    ap.add_argument("--step", action="store_true", help="한 번 튕길 때마다 확인받는다")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    strings = args.strings or ([args.string] if args.string else [6, 5, 2, 1])

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    motion = dict(points.motion)
    motion.update(PLUCK_MOTION)

    say_env()
    print("\n" + "=" * 66)
    print(f"발현 세션 — {strings}번, 줄당 {args.repeat}회")
    print("=" * 66)
    print(f"  기준 TCP    {points.robot.get('tcp')}  (펜던트에서 켜 두었는지 확인)")
    print(f"  간격        {args.gap:.1f}초")
    if args.dy:
        print(f"  y 미세 조정  {args.dy:+.2f}mm  ← 이 값은 YAML 에 안 들어간다")
    untaught = [n for n in strings if not points._plucks[n].taught]
    if untaught:
        print(f"  ⚠️ {untaught}번은 taught:false 다. 측정으로 확정하려고 일부러 통과시킨다 (#32)")
    if not args.no_pick:
        print("  피크        거치대에서 집고, 끝나면 돌려놓는다")

    poses = {n: load_pluck(points, n, args.dy) for n in strings}
    for n in strings:
        p = poses[n]
        print(f"  {n}번  {[round(v, 2) for v in p.task[:3]]}  "
              f"스트로크 {p.stroke_sign:+d}")

    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy
    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_pluck_session", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    standby = points.frame("standby")

    log = {"tool": "pluck_session", "strings": strings, "repeat": args.repeat,
           "gap_sec": args.gap, "dy_mm": args.dy,
           "started_at": time.strftime("%Y-%m-%d %H:%M:%S"), "plucks": []}
    t0 = time.monotonic()

    try:
        confirm("\n대기 자세로 간다.")
        MP._movej(standby.joints, motion)

        if not args.no_pick:
            print("대기 자세에서 그리퍼를 연다.")
            MP._release()
            confirm("피크를 집으러 간다. 거치대에 피크가 있는지 확인하라.")
            MP.pick_up(points.pick_holder(), motion, standby=standby)

        for n in strings:
            pose = poses[n]
            confirm(f"\n[{n}번 줄] {args.repeat}회 튕긴다. "
                    f"{args.gap:.1f}초 간격. 녹음을 시작하라.")
            for i in range(1, args.repeat + 1):
                if args.step:
                    confirm(f"  {n}번 {i}/{args.repeat}")
                at = time.monotonic() - t0
                MP.pluck_string(pose, motion, standby=standby)
                log["plucks"].append({
                    "string": n, "index": i,
                    "at_sec": round(at, 2),
                    "wall": time.strftime("%H:%M:%S"),
                })
                print(f"  {n}번 {i}/{args.repeat}  (+{at:.1f}초, {time.strftime('%H:%M:%S')})")
                if i < args.repeat:
                    time.sleep(args.gap)
            print(f"  [{n}번 줄] 끝. 녹음을 멈춰도 된다")

        if not args.no_pick:
            confirm("\n피크를 돌려놓는다.")
            MP.pick_return(points.pick_holder(), motion)
        confirm("대기 자세로 돌아간다.")
        MP._movej(standby.joints, motion)
    finally:
        os.makedirs(LOG_DIR, exist_ok=True)
        log["ended_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        path = os.path.join(LOG_DIR,
                            time.strftime("%Y-%m-%d_%H%M%S") + "_pluck_session.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(log, f, ensure_ascii=False, indent=2)
        print(f"\n  기록 저장: {path}  (발현 {len(log['plucks'])}회)")
        print("  녹음과 맞춰 보려면 at_sec(시작 이후 초) 또는 wall(벽시계)을 쓰면 된다")

    if args.dy:
        print(f"\n  y {args.dy:+.2f}mm 로 소리가 좋았다면 알려 달라. "
              "teach_points.yaml 의 pluck.<줄>.x 두 번째 값에 더하면 된다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
