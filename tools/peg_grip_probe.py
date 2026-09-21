#!/usr/bin/env python3
"""
peg_grip_probe — 페그를 "제대로 물었는지" 가르는 폭 문턱값을 실측한다 (이슈 #31).

무엇을 재나
  그리퍼를 살짝 조여 보고 **어디서 멈추는지**를 Modbus 로 읽는다.
  손잡이가 납작한 막대라 멈추는 위치가 세 가지로 갈린다.

    물림   손잡이가 눕혀져 있다 → 두께에서 걸려 멈춘다   (좁다)
    사점   손잡이가 세워져 있다 → 길이에서 걸려 멈춘다   (넓다)
    빈손   아무것도 없다        → 끝까지 닫힌다          (제일 좁다)

  이 셋의 숫자를 재서 그 사이를 문턱으로 잡는다.

왜 조여서 재나 — peg 프리셋(21mm)으로는 못 가린다
  21mm 는 손잡이 두께(19~20mm 추정)보다 **넓다.** 그래서 손잡이가 있든 없든
  그냥 21mm 까지 가서 멈춘다. 닿지도 않는다. 읽은 숫자가 똑같아 구분이 안 된다.
  느슨하게 무는 건 돌릴 때 옆힘으로 안전모드가 걸리는 걸 피하려는 것이라
  (사용자 확인, 21mm 로 180도까지 안전모드 없음) 그 설정은 그대로 둔다.
  **재는 순간에만** 더 조이고, 돌릴 때는 다시 21mm 로 푼다.

안전
  · 조인 채로 돌리지 않는다. 이 도구는 **회전을 아예 안 한다.**
  · 단계마다 확인을 받는다. 엔터가 아니면 멈춘다.
  · 로봇은 페그 접근점과 파지점 사이만 오간다.

쓰는 법
  python3 tools/peg_grip_probe.py --peg 6
  python3 tools/peg_grip_probe.py --peg 6 --probe-preset close
  python3 tools/peg_grip_probe.py --peg 6 --dry-run     # 로봇 없이 흐름만
  python3 tools/peg_grip_probe.py --peg 6 --simulate    # 모의값으로 결과 미리보기
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_status import BASE, COUNT, read_registers   # noqa: E402

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..",
                            "src", "tunio_robot", "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")


def confirm(msg):
    print(msg)
    ans = input("    >>> 엔터 = 진행 / n = 중단: ").strip().lower()
    if ans == "n":
        raise SystemExit("사용자가 중단했다.")


# --simulate 가 쓰는 모의값. 손잡이 두께 19.5 / 길이 26 / close 15 를 가정한 것뿐이다.
# **실측이 아니다.** 결과 화면이 어떻게 나오는지 보여주는 용도다
_FAKE = {"물림": 19.5, "사점": 26.2, "빈손": 15.0}


def read_width(args, label=None):
    """지금 손가락 사이 폭(mm). 못 읽으면 None."""
    if args.simulate:
        return _FAKE.get(label), 1 if label == "물림" else 0
    try:
        regs = read_registers(args.host, args.port, args.unit, BASE, COUNT)
    except (OSError, IOError) as e:
        print(f"    ⚠ Modbus 를 못 읽었다 ({e})")
        return None, None
    width = regs[args.width_reg] / 10.0
    grip = regs[11] if len(regs) > 11 else None
    return width, grip


def measure(args, MP, key, label, question, close_it=True):
    """한 경우를 잰다. 사람에게 상태를 만들어 달라고 한 뒤 조여 보고 읽는다."""
    print(f"\n{'=' * 66}\n[{label}]\n{'=' * 66}")
    confirm(question)
    if MP is not None:
        if close_it:
            MP._grip(args.probe_preset)     # 살짝 조인다. 여기서만
        time.sleep(args.settle)
    width, grip = read_width(args, key)
    if width is None:
        print("    폭을 못 읽어 이 항목은 건너뛴다.")
    else:
        print(f"    폭 {width:.1f}mm   파지신호 {'있음' if grip else '없음'}")
    if MP is not None:
        MP._grip("open")                    # 바로 다시 연다 (30mm)
        time.sleep(args.settle)
    return width


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--probe-preset", default="close",
                    help="잴 때 쓸 프리셋. 손잡이 두께보다 좁아야 걸린다 (기본 close=15mm)")
    ap.add_argument("--host", default="192.168.1.1")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65)
    ap.add_argument("--width-reg", type=int, default=9,
                    help="폭이 들어 있는 레지스터 번호. --scan 으로 확인한 값")
    ap.add_argument("--settle", type=float, default=0.8,
                    help="닫으라고 한 뒤 멈추기를 기다리는 시간(초)")
    ap.add_argument("--speed", type=float, default=None)
    ap.add_argument("--dry-run", action="store_true", help="로봇 없이 흐름만")
    ap.add_argument("--simulate", action="store_true",
                    help="모의 폭으로 결과 화면만 미리 본다. 실측이 아니다")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(args.yaml)
    peg = points.peg(args.peg)
    motion = dict(points.motion)
    if args.speed:
        motion["vel"] = args.speed

    print(f"\n{'=' * 66}")
    print(f"{args.peg}번 페그 — 물림 / 사점 / 빈손 문턱 실측")
    print(f"{'=' * 66}")
    print(f"  잴 때 프리셋  {args.probe_preset} "
          f"({points.gripper['presets'][args.probe_preset].get('width_mm')}mm)")
    print(f"  작업 프리셋   peg ({points.gripper['presets']['peg'].get('width_mm')}mm) "
          "— 이 도구는 여기로 돌리지 않는다")
    print("  회전은 하지 않는다. 조인 채로 돌리면 옆힘으로 안전모드가 걸린다")

    MP = None
    if not (args.dry_run or args.simulate):
        import rclpy
        import DR_init
        DR_init.__dsr__id = args.id
        DR_init.__dsr__model = args.model
        rclpy.init()
        node = rclpy.create_node("tunio_grip_probe", namespace=args.id)
        DR_init.__dsr__node = node
        from tunio_robot import motion_primitives as MP_
        MP = MP_
        MP.connect(points.robot, points.gripper)

        back = TP.backed_off(peg.task, motion.get("approach_mm", 100.0))
        confirm("\n그리퍼를 연다(30mm). 손이 끼지 않게 치워라.")
        MP._grip("open")
        route = points.route()
        if route is not None:
            confirm("경유지로 올라간다. 팔이 크게 움직인다.")
            MP.goto_route(route, motion)
        confirm("접근점으로 간다.")
        MP._movejx(back, motion, sol=peg.sol)
        confirm("페그 앞까지 직선으로 들어간다. 아직 안 닫는다.")
        MP._movel(peg.task, motion)

    got = {}
    got["물림"] = measure(
        args, MP, "물림", "1/3  물림 — 제대로 잡은 상태",
        "손잡이를 손으로 **눕혀** 주세요 (그리퍼 손가락과 나란하게).\n"
        "    그 상태에서 조여 보고 폭을 읽습니다.")
    got["사점"] = measure(
        args, MP, "사점", "2/3  사점 — 손잡이가 세로로 선 상태",
        "이번에는 손잡이를 손으로 **90도 돌려 세워** 주세요.\n"
        "    손가락이 양 끝에 걸려 덜 닫힐 겁니다.")

    if MP is not None:
        confirm("\n3/3 은 허공에서 잽니다. 페그에서 물러납니다.")
        MP._movel(TP.backed_off(peg.task, motion.get("approach_mm", 100.0)), motion)
    got["빈손"] = measure(
        args, MP, "빈손", "3/3  빈손 — 아무것도 없는 상태",
        "지금 페그에서 떨어져 있습니다. 이대로 허공에서 조여 봅니다.")

    if MP is not None:
        confirm("\n끝났다. 경유지로 돌아간다.")
        MP._grip("open")
        route = points.route()
        if route is not None:
            MP.goto_route(route, motion)

    report(args, got)


def report(args, got):
    print(f"\n{'=' * 66}\n결과 — {args.peg}번 페그\n{'=' * 66}")
    for k in ("물림", "사점", "빈손"):
        v = got.get(k)
        print(f"  {k}   {'못 읽음' if v is None else f'{v:6.1f}mm'}")

    vals = {k: v for k, v in got.items() if v is not None}
    if len(vals) < 3:
        print("\n세 개가 다 있어야 문턱을 낼 수 있다. 못 읽은 항목을 다시 재라.")
        print("Modbus 가 안 읽히면 먼저 python3 tools/gripper_status.py --scan")
        return

    hold, dead, empty = got["물림"], got["사점"], got["빈손"]
    if not (empty < hold < dead):
        print("\n⚠ 예상한 순서(빈손 < 물림 < 사점)가 아니다. 셋 중 하나가 잘못 잡혔다.")
        print("   손잡이를 눕힌/세운 상태가 바뀌었거나, 폭 레지스터 번호가 틀렸을 수 있다.")
        print("   python3 tools/gripper_status.py --watch 로 여닫으며 어느 칸이 변하는지 보라.")
        return

    lo = (empty + hold) / 2.0        # 이보다 좁으면 빈손
    hi = (hold + dead) / 2.0         # 이보다 넓으면 사점
    print(f"\n  문턱 — 폭이 {lo:.1f}mm 보다 좁으면 **빈손**")
    print(f"         {lo:.1f} ~ {hi:.1f}mm 면 **물림**")
    print(f"         {hi:.1f}mm 보다 넓으면 **사점** (손목을 조금 돌리고 재시도)")
    print(f"\n  여유 — 빈손·물림 사이 {hold - empty:.1f}mm, 물림·사점 사이 {dead - hold:.1f}mm")
    if min(hold - empty, dead - hold) < 2.0:
        print("  ⚠ 여유가 2mm 미만이다. 판정이 흔들릴 수 있으니 몇 번 더 재 보라.")

    out = {
        "tool": "peg_grip_probe", "peg": args.peg,
        "probe_preset": args.probe_preset, "width_reg": args.width_reg,
        "measured_mm": got,
        "thresholds_mm": {"empty_below": round(lo, 1), "dead_above": round(hi, 1)},
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime("%Y-%m-%d_%H%M%S")
                        + f"_grip_probe_peg{args.peg}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n  기록 저장: {path}")
    print("  이 값을 teach_points.yaml 의 pegs.<번호> 에 옮겨 적으면 Phase 0 가 쓴다.")


if __name__ == "__main__":
    main()
