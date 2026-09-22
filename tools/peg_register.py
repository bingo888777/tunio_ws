#!/usr/bin/env python3
"""
peg_register — Phase 0 등록부터 재진입까지 **전체 흐름**을 확인한다 (이슈 #31).

이 도구가 확인하는 것
  1. 손잡이가 어느 각에 있든 물어서 각을 확정한다 (Phase 0, register_peg)
     · 서 있으면 못 들어간다. 진입 중 접촉을 감지해 멈추고 어느 줄인지 알린다
  2. 그 각을 기억한다 (peg_state, 파일로 남는다)
  3. 조절하러 다시 들어갈 때 **그 각으로 돌아서** 들어간다
     · 이게 안 되면 물리면서 페그가 교시 각으로 되돌려진다 (#31 의 본론)

  --turn 을 주면 3번까지 본다. 안 주면 1~2번만 하고 끝낸다.

안전
  · Phase 0 는 꽉 물지만(15mm) **돌리지 않는다.** 돌릴 때는 21mm 로 푼다
  · 진입은 **힘 제어**다. 미는 힘에 상한을 걸어 그보다 세게는 못 민다.
    기타가 몇 N 에서 밀리는지 몰라도 안전하다 (2026-09-21 변경)

시작 전에
  **세로로 선 손잡이가 있으면 손으로 눕혀 두세요.** 열기 폭 38mm 로는 서 있는
  손잡이(38.2mm 필요)를 못 지납니다. 폭을 키우면 옆 페그를 칩니다.
  조율 전이라 음이 틀어져도 상관없습니다. 빠뜨리면 로봇이 멈추고 알려줍니다.
  · 기본 회전 방향은 푸는 쪽이다. 조이는 쪽은 줄이 끊어질 수 있다
  · 단계마다 확인을 받는다

쓰는 법
  python3 tools/peg_register.py --peg 6                      # 등록만
  python3 tools/peg_register.py --peg 6 --turn 30 --cycles 2 # 등록 + 조절 2회
  python3 tools/peg_register.py --pegs 6 5 2 1               # 네 줄 등록
  python3 tools/peg_register.py --peg 6 --dry-run            # 안 움직이고 계획만
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


class _Say:
    """register_peg 이 진행 상황을 찍게 한다. 어디서 멈췄는지 보이려는 것."""

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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, choices=[1, 2, 5, 6])
    ap.add_argument("--pegs", type=int, nargs="+", choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--turn", type=float, default=None,
                    help="등록 뒤 이 각도만큼 조절해 본다(도). 없으면 등록만 한다")
    ap.add_argument("--cycles", type=int, default=1, help="--turn 을 몇 번 반복할지")
    ap.add_argument("--direction", choices=["loosen", "tighten"], default="loosen")
    ap.add_argument("--host", default="192.168.1.1")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65)
    ap.add_argument("--width-reg", type=int, default=9)
    ap.add_argument("--state", default=None, help="peg_state 파일 경로")
    ap.add_argument("--fresh", action="store_true", help="기존 등록을 무시하고 새로 등록")
    ap.add_argument("--no-force", action="store_true",
                    help="힘 제어 없이 그냥 들어간다. 사람이 손잡이를 눕혀 놓고 "
                         "지켜보는 측정 세션용 (#13, #8)")
    ap.add_argument("--assume-grip", action="store_true",
                    help="폭을 안 읽고 물렸다고 본다. Modbus 가 안 붙는 PC 용. "
                         "빈손·사점을 못 가리니 눈으로 확인할 것")
    # 힘 제어 값을 YAML 안 고치고 바로 바꿔 본다. 실측 전 임시값이라 쓸어 봐야 한다
    ap.add_argument("--force-n", type=float, default=None,
                    help="미는 힘 상한(N). YAML 의 insert_force_n 을 덮어쓴다. "
                         "순응 스프링을 이기고 갈 거리 = (이 값 - 기준선) / 강성")
    ap.add_argument("--contact-n", type=float, default=None,
                    help="막힘 판정 문턱(N). YAML 의 contact_force_n 을 덮어쓴다. "
                         "기준선과 스프링 몫을 뺀 **접촉력** 기준이다")
    ap.add_argument("--search-mm", type=float, default=None,
                    help="힘 제어로 더듬을 거리(mm). YAML 의 search_mm 을 덮어쓴다. "
                         "나머지 구간은 위치 제어로 빠르게 간다")
    ap.add_argument("--no-route", action="store_true",
                    help="경유지를 건너뛴다. **이미 그 페그 근처에 있을 때만.** "
                         "힘 제어 값을 여러 번 고쳐 가며 볼 때 빠르다. "
                         "다른 쪽 페그로 넘어갈 때 쓰면 팔이 기타를 쓸고 간다")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    pegs = args.pegs or ([args.peg] if args.peg else None)
    if not pegs:
        raise SystemExit("--peg 또는 --pegs 로 대상 줄을 지정하라")

    from tunio_robot import peg_state as PS
    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    handle = dict(points.peg_handle())
    for flag, key in (("force_n", "insert_force_n"), ("contact_n", "contact_force_n"),
                      ("search_mm", "search_mm")):
        v = getattr(args, flag)
        if v is not None:
            print(f"  ** {key} 덮어씀: {handle.get(key)} -> {v}")
            handle[key] = v
    motion = dict(points.motion)
    if args.speed:
        motion.pop("transit_tcp_vel", None)
        motion.pop("transit_tcp_acc", None)
        motion["vel"] = args.speed
    state = PS.PegState(path=args.state or PS.DEFAULT_PATH).load()
    if args.fresh:
        state.forget()

    print("\n" + "=" * 66)
    print(f"페그 등록 · 재진입 확인 — {pegs}번")
    print("=" * 66)
    print(f"  잴 때 프리셋   {handle['probe_preset']} "
          f"({points.gripper['presets'][handle['probe_preset']].get('width_mm')}mm) — 꽉 문다")
    print(f"  작업 프리셋    peg ({points.gripper['presets']['peg'].get('width_mm')}mm)")
    print(f"  열기           open ({points.gripper['presets']['open'].get('width_mm')}mm)")
    print(f"  문턱           빈손 < {handle['thresholds_mm']['empty_below']} "
          f"< 물림 < {handle['thresholds_mm']['dead_above']} < 사점  (mm)")
    if args.no_force:
        print("  진입           힘 제어 없이 그냥 들어간다 (--no-force)")
        print("                 ⚠️ 손잡이가 눕혀져 있는지 **사람이** 확인할 것")
    else:
        print(f"  진입           접근점 → 페그 앞 {handle['search_mm']:.0f}mm 는 위치 제어,\n"
              f"                 거기서부터 힘 제어 — 미는 힘 {handle['insert_force_n']:.1f}N 상한, "
              f"{handle['contact_force_n']:.1f}N 에 막힘 판정")
    if args.assume_grip:
        print("  폭 판정        안 한다 (--assume-grip). 물렸는지 눈으로 확인할 것")
    if handle.get("requires_prealign"):
        print("  ⚠️ 세로로 선 손잡이는 미리 손으로 눕혀 둘 것 (열기 38mm 로는 못 지난다)")
    if args.turn:
        print(f"  조절 확인      {args.direction} {args.turn:.0f}도 × {args.cycles}회")
    print(f"  상태 파일      {state.path}")
    known = state.registered()
    print(f"  이미 등록된 줄 {known if known else '없음'}")

    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy
    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_peg_register", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import motion_primitives as MP
    from tunio_robot.grip_width import make_reader
    MP.connect(points.robot, points.gripper)
    read_width = make_reader(args.host, args.port, args.unit, args.width_reg)

    log = {"tool": "peg_register", "pegs": pegs, "direction": args.direction,
           "turn_deg": args.turn, "cycles": args.cycles, "results": {}}
    failed = []
    # 경유지는 **반대쪽으로 넘어갈 때만** 쓴다. 같은 쪽 페그끼리는 37mm 밖에
    # 안 떨어져 있어 바로 간다 (robot_controller._cross_to 와 같은 규칙).
    # 첫 페그는 팔이 어디 있는지 모르므로 무조건 경유한다.
    last_side = None

    for no in pegs:
        peg = points.peg(no)
        print("\n" + "=" * 66)
        print(f"[{no}번 페그]  {TP.side_of(no)} 쪽")
        print("=" * 66)
        route = points.route()
        side = TP.side_of(no)
        if args.no_route:
            print("  경유지를 건너뛴다 (--no-route). 지금 자리에서 바로 들어간다.")
            print("  ⚠️ 그 페그 근처에 있을 때만 맞다. 반대쪽이면 기타를 쓸고 간다")
        elif side == last_side:
            print(f"  경유지를 안 거친다 — 직전 페그와 같은 {side} 쪽이다 "
                  f"(37mm 옆). 반대쪽으로 넘어갈 때만 경유한다")
        elif route is not None:
            confirm(f"경유지로 올라간다 ({last_side or '어디 있는지 모름'} -> {side}). "
                    f"팔이 크게 움직인다.")
            MP.goto_route(route, motion)
        last_side = side

        rec = {"attempts": []}
        confirm(f"\n1) 등록 — 순응 켜고 {handle['insert_force_n']:.1f}N 상한으로 밀어 넣은 뒤\n"
                "    꽉 물어 각을 확정한다. 손잡이가 서 있으면 막혀서 멈춘다.\n"
                "    ⚠️ 힘 제어 첫 실행이면 기타에서 떨어진 자리에서 한 번 보라.")
        try:
            roll = MP.register_peg(peg, motion, handle, read_width, logger=_Say(),
                                   use_force=not args.no_force,
                                   assume_grip=args.assume_grip)
        except MP.MotionError as e:
            print(f"    ✗ 등록 실패 — {e}")
            rec["error"] = str(e)
            log["results"][no] = rec
            failed.append(no)
            continue
        state.register(no, roll)
        state.save()
        rec["registered_roll_deg"] = state.roll_of(no)
        print(f"    ✓ 손잡이 각 {state.roll_of(no):+.1f}도 로 등록했다")

        if not args.turn:
            log["results"][no] = rec
            continue

        for c in range(1, args.cycles + 1):
            before = state.roll_of(no)
            joint, delta = points.joint_delta(
                no, TP.LOOSEN if args.direction == "loosen" else TP.TIGHTEN, args.turn)
            confirm(f"\n2-{c}) 조절 — 손잡이 각 {before:+.1f}도 로 들어가 "
                    f"J{joint} 를 {delta:+.1f}도 돌린다.\n"
                    f"    들어갈 때 손목이 {before:+.1f}도 돌아간 채인지 눈으로 보라. "
                    "세로로 정렬해 들어가면 그게 되돌림이다.")
            with state.turning(no, delta if joint == 6 else None):
                MP.adjust_peg(peg, joint, delta, motion, roll_deg=before)
            after = state.roll_of(no)
            print(f"    ✓ 손잡이 각 {before:+.1f} -> {after}도")
            rec["attempts"].append({"cycle": c, "before": before,
                                    "joint_delta": delta, "after": after})
        log["results"][no] = rec

    route = points.route()
    if route is not None:
        if args.no_route:
            print("\n끝났다. 경유지를 안 거친다 (--no-route). 팔은 접근점에 선다.")
        else:
            confirm("\n끝났다. 경유지로 돌아간다.")
        MP.goto_route(route, motion)

    print("\n" + "=" * 66)
    print("결과")
    print("=" * 66)
    for no in pegs:
        r = log["results"].get(no, {})
        if "error" in r:
            print(f"  {no}번  ✗ {r['error'][:60]}")
        else:
            print(f"  {no}번  ✓ 등록 {r.get('registered_roll_deg'):+.1f}도"
                  + (f", 조절 {len(r['attempts'])}회" if r.get("attempts") else ""))
    if failed:
        print(f"\n  ⚠ 실패한 줄: {failed}. 4줄 전부 실패하면 조율을 시작하지 않는다 "
              "(CLAUDE.md 5절)")

    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime("%Y-%m-%d_%H%M%S") + "_peg_register.json")
    log["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(log, f, ensure_ascii=False, indent=2)
    print(f"\n  기록 저장: {path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
