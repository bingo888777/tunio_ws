#!/usr/bin/env python3
"""
wipe_check — 안경닦이로 넥 닦기를 단계별로 확인한다 (2026-09-23).

이 도구가 확인하는 것
  1. 거치대에서 안경닦이 집기
  2. 픽업 자리에서 Z 만 먼저 닦기 높이로 올리고, 그다음 수평으로 시작점 이동
     (대각선 이동으로 기타를 쓸고 가지 않게 — motion_primitives.wipe_strings)
  3. 순응제어 켜고, 힘 제어(Base -Z)로 접촉 정착 대기 → 종료점까지 훑기
     (motion_primitives._wipe_stroke)
  4. 순응 끄고 종료점 명목 높이로 복귀 (= 접촉 해제)
  5. 반환 상자에 반환 (픽업 자리와 다른 곳)

각 단계 사이에 확인을 받는다. wipe_strings() 를 통째로 부르지 않고
단계를 쪼개 부르는 이유 — 처음 실기라 어느 단계에서 잘못되는지 바로 보여야 한다.
문제가 생기면 여기서 멈추고 안경닦이는 손으로 정리할 것.

시작 전에 필요한 것 (teach_points.yaml 의 wipe 절)
  wipe.pickup      안경닦이를 집는 자리
  wipe.start       너트, 3·4번 줄 사이 접촉 자세
  wipe.end         적당한 종료점 (브릿지까지는 안 감)
  wipe.return_box  안경닦이를 놓는 상자 구역 (pickup 과 다른 자리 —
                   그리퍼가 한 점만 잡아서 펼쳐진 채로 물리므로 접어서
                   되돌려 놓을 수 없다)
  넷 다 taught: true 여야 한다. 없으면 여기서 바로 멈춘다.

힘 제어
  힘 방향은 자세에서 추론하지 않고 **Base -Z 로 고정**했다(자세 기울기와
  무관하게 넥을 누르는 방향은 항상 아래이므로). 시작점보다 `wipe_press_depth_mm`
  만큼 더 깊은 목표로 **비동기 이동**을 걸고(끝나길 안 기다림), 그 상태에서
  0.2초마다 힘을 읽는다 — 목표힘 근방(±`wipe_arrive_tolerance_n`)에 연속
  `wipe_arrive_confirm_polls`회 들어오면 곧바로 훑기로 넘어간다. 도달 못 해도
  `wipe_settle_sec`(기본 20초, 최대 대기)이 지나면 그냥 넘어간다.

  안전 상한 `wipe_max_force_n`(기본 목표힘+10)을 넘으면 `MoveStop` 서비스콜로
  즉시 멈추고 실패 처리한다 — "컨트롤러가 알아서 멈춰 줄 것"이라는 가정에만
  기대지 않는다.

  ⚠️ **1차 구현.** `amovel`+`set_desired_force` 조합으로 실제로 멈춰 세워지는지
  자체가 가설이다(2026-09-23). 세거나(줄이 밀림) 약하면(안 닦임) --press-n
  으로 바로 바꿔 보고, 맞으면 yaml 의 motion 절 wipe_press_force_n 에 적어 넣는다.

안전
  · 순응제어가 걸린 채로 움직인다 — 걸리면 물러나지 밀지 않는다
  · 단계마다 확인을 받는다. 비상정지에 손을 올려두고 볼 것
  · 실패해도 안경닦이 반환은 finally 로 항상 시도한다(놓친 채 끝나지 않게)

쓰는 법
  python3 tools/wipe_check.py                        # 기본값으로 확인
  python3 tools/wipe_check.py --press-n 1.5           # 누르는 힘을 낮춰서
  python3 tools/wipe_check.py --press-depth-mm 5      # 접촉 시도 깊이를 더 작게(안전)
  python3 tools/wipe_check.py --max-force 8           # 안전 상한을 낮게(첫 실기 추천)
  python3 tools/wipe_check.py --settle-sec 10         # 도달 못 해도 넘어가는 최대 대기를 줄여서
  python3 tools/wipe_check.py --speed 20              # 접근·훑기 속도를 20%로
  python3 tools/wipe_check.py --dry-run               # 안 움직이고 계획·좌표만 찍는다
  python3 tools/wipe_check.py --yes                   # 확인 없이 쭉 돈다
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..",
                            "src", "tunio_robot", "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")


class _Say:
    @staticmethod
    def info(msg):
        print(f"    · {msg}")

    @staticmethod
    def warn(msg):
        print(f"    ⚠ {msg}")


def confirm(msg):
    print(msg)
    if input("    >>> 엔터 = 진행 / n = 중단: ").strip().lower() == "n":
        raise SystemExit("사용자가 중단했다.")


def fmt(vals):
    return "[" + ", ".join(f"{v:7.2f}" for v in vals) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--press-n", type=float, default=None,
                    help="누르는 목표힘(N). yaml 의 wipe_press_force_n 을 덮어쓴다")
    ap.add_argument("--settle-sec", type=float, default=None,
                    help="목표힘 도달을 못 해도 넘어가는 최대 대기 시간(초). "
                         "yaml 의 wipe_settle_sec 을 덮어쓴다")
    ap.add_argument("--arrive-tolerance-n", type=float, default=None,
                    help="목표힘 도달로 볼 오차 범위(N). "
                         "yaml 의 wipe_arrive_tolerance_n 을 덮어쓴다")
    ap.add_argument("--press-depth-mm", type=float, default=None,
                    help="시작점보다 얼마나 더 깊이 비동기 이동을 걸지(mm). "
                         "yaml 의 wipe_press_depth_mm 을 덮어쓴다. 처음엔 작게")
    ap.add_argument("--max-force", type=float, default=None,
                    help="이 힘(N)을 넘으면 MoveStop 으로 즉시 멈추고 실패 처리한다. "
                         "yaml 의 wipe_max_force_n 을 덮어쓴다(기본은 목표힘+10)")
    ap.add_argument("--speed", type=float, default=None,
                    help="설정 속도의 %%. 처음 실기면 낮춰서 볼 것")
    ap.add_argument("--yes", action="store_true", help="확인 없이 쭉 돈다")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))

    motion = dict(points.motion)
    if args.press_n is not None:
        print(f"  ** wipe_press_force_n 덮어씀: "
              f"{motion.get('wipe_press_force_n', 2.0)} -> {args.press_n}")
        motion["wipe_press_force_n"] = args.press_n
    if args.settle_sec is not None:
        print(f"  ** wipe_settle_sec 덮어씀: "
              f"{motion.get('wipe_settle_sec', 20.0)} -> {args.settle_sec}")
        motion["wipe_settle_sec"] = args.settle_sec
    if args.press_depth_mm is not None:
        print(f"  ** wipe_press_depth_mm 덮어씀: "
              f"{motion.get('wipe_press_depth_mm', 10.0)} -> {args.press_depth_mm}")
        motion["wipe_press_depth_mm"] = args.press_depth_mm
    if args.max_force is not None:
        print(f"  ** wipe_max_force_n 덮어씀: "
              f"{motion.get('wipe_max_force_n', '(목표힘+10)')} -> {args.max_force}")
        motion["wipe_max_force_n"] = args.max_force
    if args.arrive_tolerance_n is not None:
        print(f"  ** wipe_arrive_tolerance_n 덮어씀: "
              f"{motion.get('wipe_arrive_tolerance_n', 1.0)} -> {args.arrive_tolerance_n}")
        motion["wipe_arrive_tolerance_n"] = args.arrive_tolerance_n
    if args.speed:
        motion.pop("transit_tcp_vel", None)
        motion.pop("transit_tcp_acc", None)
        scale = max(0.05, args.speed / 100.0)
        motion["vel"] = points.motion.get("vel", 30) * scale
        motion["acc"] = points.motion.get("acc", 30) * scale
        motion["travel_vel"] = [v * scale for v in points.motion.get("travel_vel", [50.0, 30.0])]
        motion["travel_acc"] = [v * scale for v in points.motion.get("travel_acc", [100.0, 60.0])]

    print("\n" + "=" * 66)
    print("넥 닦기 단계별 확인 — wipe_check")
    print("=" * 66)

    try:
        wipe = points.wipe()
    except TP.TeachPointsError as e:
        print(f"\n  ✗ {e}")
        print("\n  teach_points.yaml 의 wipe 절이 아직 안 채워졌거나 taught: false 다.")
        print("  이 파일의 머리말(--help)에 좌표 찾는 절차가 적혀 있다.")
        return 1

    press_n = float(motion.get("wipe_press_force_n", 2.0))
    settle_sec = float(motion.get("wipe_settle_sec", 20.0))
    press_depth_mm = float(motion.get("wipe_press_depth_mm", 10.0))
    max_force_n = float(motion.get("wipe_max_force_n", press_n + 10.0))
    poll_sec = float(motion.get("wipe_poll_sec", 0.2))
    arrive_tol_n = float(motion.get("wipe_arrive_tolerance_n", 1.0))
    arrive_confirm_polls = int(motion.get("wipe_arrive_confirm_polls", 3))
    travel_z = float(motion.get("wipe_travel_z_mm", wipe.start.task[2]))
    print(f"  픽업 자리        {fmt(wipe.pickup.task)}")
    print(f"  시작점(너트)     {fmt(wipe.start.task)}")
    print(f"  종료점           {fmt(wipe.end.task)}")
    print(f"  반환 상자        {fmt(wipe.return_box.task)}")
    print(f"  경유 높이(Z)     {travel_z:.1f}mm (픽업 자리에서 먼저 여기까지 올린다)")
    print(f"  누르는 목표힘    {press_n:.1f}N (Base -Z 고정)")
    print(f"  접촉 방식        시작점보다 {press_depth_mm:.0f}mm 더 깊이 비동기 이동 "
          f"(끝나길 안 기다림) — 힘/순응이 먼저 멈춰 세우는 걸 기대")
    print(f"  안전 상한        {max_force_n:.1f}N 넘으면 MoveStop 으로 즉시 멈추고 실패 처리 "
          f"({poll_sec:.1f}초마다 확인)")
    print(f"  도달 판정        목표 ±{arrive_tol_n:.1f}N 이내가 연속 {arrive_confirm_polls}회면 "
          "곧바로 훑기로 넘어간다")
    print(f"  최대 대기        {settle_sec:.0f}초 (도달 못 해도 이 시간이 지나면 그냥 넘어간다)")
    print(f"  훑기 속도        {motion.get('wipe_vel', 30.0):.0f}mm/s "
          f"(가속 {motion.get('wipe_acc', 50.0):.0f})")

    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy
    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_wipe_check", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    print("\n  서비스를 기다린다 (최대 30초) …")
    MP.connect(points.robot, points.gripper, logger=_Say())
    print(f"  TCP 확인됨 — tool='{MP._read_named('tool')}' tcp='{MP._read_named('tcp')}'\n")

    log = {"tool": "wipe_check", "press_n": press_n, "settle_sec": settle_sec, "steps": []}
    ok = True
    try:
        confirm("1) 거치대에서 안경닦이를 집는다.")
        MP.pick_up(wipe.pickup, motion, logger=_Say())
        log["steps"].append("pick_up")

        try:
            v_app, a_app = MP._transit_speed(motion)
            mid = list(wipe.pickup.task)
            mid[2] = travel_z

            confirm(f"\n2a) 픽업 자리에서 Z 만 먼저 {travel_z:.1f}mm 로 올린다 "
                    f"(대각선으로 안 가고 높이부터) — {fmt(mid)}")
            MP._movel_base(mid, v_app, a_app)
            log["steps"].append("lift_to_travel_z")

            confirm(f"\n2b) 그다음 수평으로 시작점까지 이동한다 — {fmt(wipe.start.task)}")
            MP._movel_base(wipe.start.task, v_app, a_app)
            log["steps"].append("approach_start")

            confirm(f"\n3) 순응제어를 켜고, {press_n:.1f}N(Base -Z)으로 "
                    f"{settle_sec:.0f}초 접촉 정착 대기 후 종료점까지 훑는다.\n"
                    "    ⚠️ 힘 제어 첫 실행이면 기타에서 떨어진 자리에서 한 번 보라.")
            ret_on = MP.compliance(motion, on=True)
            print(f"    · 순응제어 켜기: task_compliance_ctrl 반환={ret_on!r} (0 정상 / -1 거부)")
            try:
                MP._wipe_stroke(wipe.end.task, motion, logger=_Say())
                log["steps"].append("wipe_stroke")
            finally:
                ret_off = MP.compliance(motion, on=False)
                print(f"    · 순응제어 끄기: release_compliance_ctrl 반환={ret_off!r}")

            confirm(f"\n4) 종료점 명목 높이로 복귀한다(접촉 해제) — {fmt(wipe.end.task)}")
            MP._movel_base(wipe.end.task, v_app, a_app)
            log["steps"].append("retreat")
        finally:
            confirm("\n5) 안경닦이를 반환 상자에 놓는다.")
            MP.pick_return(wipe.return_box, motion, logger=_Say())
            log["steps"].append("pick_return")
    except MP.MotionError as e:
        print(f"\n  ✗ 실패 — {e}")
        log["error"] = str(e)
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    log["ok"] = ok
    try:
        from tunio_robot import runlog
        path = runlog.save("wipe_check", log, LOG_DIR)
        if path:
            print(f"\n  기록 {path}")
    except Exception:                                 # noqa: B902 — 기록 실패로 결과를 덮지 않는다
        pass

    print("\n" + "=" * 66)
    print("전부 통과. 안경닦이가 넥을 훑고 반환됐다" if ok else "실패한 단계가 있다. 위 메시지를 보라")
    print("=" * 66)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
