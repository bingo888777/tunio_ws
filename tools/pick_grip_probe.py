#!/usr/bin/env python3
"""
pick_grip_probe — RG2 가 **피크를 물었는지 폭으로 가릴 수 있는지** 본다.

무엇을 보나
  그리퍼를 close 로 조여 보고 멈춘 폭을 Modbus 로 읽는다. 빈손과 피크 회차를
  번갈아 여러 번 재서, 두 무리가 **겹치지 않고 갈리는지**를 본다.

    빈손      골무끼리 맞닿아 끝까지 닫힌다        (6번 페그 실측 13.0mm)
    피크      피크 두께만큼 덜 닫힌다             (이 도구가 재려는 값)

  갈리면 문턱을 제안하고, 겹치면 "폭으로는 못 가른다" 고 말한다.
  **차이가 작을 수 있다.** 피크는 페그 손잡이(19.6mm)와 달리 두께가 1mm 안팎이고
  골무 고무가 눌려 그만큼을 먹는다. 그래서 한 번 재서 끝내지 않고 반복한다.

왜 번갈아 재나
  한쪽을 몰아서 재면 그 사이에 바뀐 것(골무가 눌린 정도, 온도, 자세)이 통째로
  두 무리의 차이로 둔갑한다. 2026-09-21 회전 토크 회차가 정확히 그래서 못 쓰게
  됐다 — 파지와 허공이 다른 날이라 비교가 성립하지 않았다(CLAUDE.md 3.6).
  번갈아 재면 그 몫이 양쪽에 똑같이 섞여 상쇄된다.

무엇을 안 하나
  · **팔은 안 움직인다.** 사람이 손으로 피크를 손가락 사이에 끼웠다 뺀다
  · 파지 감지 플래그(레지스터 11)로 판정하지 않는다. **빈손도 '있음' 으로 읽힌다**
    (2026-09-21 실측). 같이 찍기는 하지만 참고일 뿐이다
  · Modbus 로 쓰지 않는다. 기능 코드 3(읽기)만 보낸다

준비
  1. 티치펜던트를 자동 모드로, WebLogic 프로그램이 돌고 있어야 한다
  2. 두산 드라이버 실행 (tools/io_check.py 머리말 참조)
  3. Modbus 가 읽히는지 먼저 확인 — python3 tools/gripper_status.py

쓰는 법
  python3 tools/pick_grip_probe.py                  # 빈손·피크 5회씩 번갈아
  python3 tools/pick_grip_probe.py --trials 10
  python3 tools/pick_grip_probe.py --preset peg     # 다른 프리셋으로 조여 본다
  python3 tools/pick_grip_probe.py --no-robot       # 사람이 직접 여닫는다(폭만 읽음)
  python3 tools/pick_grip_probe.py --simulate       # 모의값으로 결과 화면만 본다
"""
import argparse
import json
import os
import random
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from gripper_status import BASE, COUNT, read_registers   # noqa: E402

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..",
                            "src", "tunio_robot", "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")

EMPTY, PICK = "빈손", "피크"

# --simulate 가 쓰는 모의값. **실측이 아니다.** 빈손 13.0(peg_handle.empty_mm)에
# 피크 두께 1mm 를 얹고 잡음 0.1mm 를 준 것뿐이다. 화면 확인용
_FAKE = {EMPTY: 13.0, PICK: 14.0}


def confirm(msg):
    print(msg)
    if input("    >>> 엔터 = 진행 / n = 중단: ").strip().lower() == "n":
        raise SystemExit("사용자가 중단했다.")


def read_width(args, label):
    """지금 손가락 사이 폭(mm)과 파지 플래그. 못 읽으면 (None, None)."""
    if args.simulate:
        return round(_FAKE[label] + random.gauss(0, 0.1), 1), 1
    try:
        regs = read_registers(args.host, args.port, args.unit, BASE, COUNT)
    except (OSError, IOError) as e:
        print(f"    ⚠ Modbus 를 못 읽었다 ({e})")
        return None, None
    if args.width_reg >= len(regs):
        print(f"    ⚠ 레지스터 {args.width_reg} 번이 응답에 없다 (총 {len(regs)}개)")
        return None, None
    grip = regs[11] if len(regs) > 11 else None
    return regs[args.width_reg] / 10.0, grip


def one_trial(args, rg2, label, n, total):
    """한 회차 — 열고, 사람이 상태를 만들고, 조이고, 읽는다."""
    print(f"\n{'-' * 62}")
    print(f"[{label}] {n}/{total}")
    if rg2 is not None:
        rg2.select(args.open_preset)
        time.sleep(args.settle)
    ask = ("피크를 손가락 사이에 **끼워** 주세요 (작업 때와 같은 자리·방향으로)."
           if label == PICK else
           "피크를 **빼** 주세요. 손가락 사이를 비웁니다.")
    if rg2 is None and not args.simulate:
        ask += f"\n    그리퍼도 사람이 '{args.open_preset}' → '{args.preset}' 으로 여닫으세요."
    confirm(f"    {ask}")
    if rg2 is not None:
        rg2.select(args.preset)
        time.sleep(args.settle)
    width, grip = read_width(args, label)
    if width is None:
        print("    폭을 못 읽어 이 회차는 버린다.")
    else:
        flag = "있음" if grip else ("없음" if grip is not None else "?")
        print(f"    폭 {width:5.1f}mm   (파지플래그 {flag} — 판정에 안 쓴다)")
    return width


def describe(vals):
    """평균·최소·최대·표준편차. 한 개뿐이면 표준편차는 없다."""
    if not vals:
        return None
    return {
        "n": len(vals), "mean": statistics.fmean(vals),
        "min": min(vals), "max": max(vals),
        "sd": statistics.stdev(vals) if len(vals) > 1 else None,
    }


def report(args, got):
    print(f"\n{'=' * 62}\n결과 — 피크 파지를 폭으로 가릴 수 있나\n{'=' * 62}")
    stats = {}
    for label in (EMPTY, PICK):
        vals = got[label]
        s = describe(vals)
        stats[label] = s
        if s is None:
            print(f"  {label}   측정값 없음")
            continue
        sd = f"{s['sd']:.2f}" if s["sd"] is not None else "—"
        print(f"  {label}   평균 {s['mean']:5.2f}mm   "
              f"범위 {s['min']:.1f}~{s['max']:.1f}   표준편차 {sd}   ({s['n']}회)")
        print(f"         값: " + ", ".join(f"{v:.1f}" for v in vals))

    verdict, threshold, gap = "판정 불가", None, None
    if stats[EMPTY] and stats[PICK]:
        gap = stats[PICK]["min"] - stats[EMPTY]["max"]
        print(f"\n  두 무리 사이 간격  {gap:+.2f}mm  "
              "(피크 최솟값 − 빈손 최댓값. 양수라야 겹치지 않는다)")
        if gap > 0:
            threshold = (stats[EMPTY]["max"] + stats[PICK]["min"]) / 2.0
            verdict = "갈린다"
            print(f"  → **겹치지 않는다.** 문턱 {threshold:.2f}mm — "
                  "이보다 넓으면 물렸고, 좁으면 빈손이다")
            if gap < 0.3:
                print("  ⚠ 간격이 0.3mm 미만이다. 폭 분해능이 0.1mm 라 회차 몇 번으로는")
                print("    운일 수 있다. --trials 를 늘려 다시 재라")
        else:
            verdict = "겹친다"
            print("  → **겹친다. 지금 조건에서는 폭으로 피크를 못 가린다.**")
            print("    볼 것 — 피크가 매번 같은 자리에 끼워졌나, 골무가 눌리는 정도가")
            print("    회차마다 다르지 않나, 더 조이는 프리셋(--preset)이 있나")

    out = {
        "tool": "pick_grip_probe",
        "preset": args.preset, "open_preset": args.open_preset,
        "width_reg": args.width_reg, "settle_sec": args.settle,
        "simulate": args.simulate,
        "trials_mm": {k: got[k] for k in (EMPTY, PICK)},
        "stats_mm": stats,
        "gap_mm": gap, "threshold_mm": threshold, "verdict": verdict,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    if args.simulate:
        print("\n  --simulate 라 저장하지 않는다. 실측이 아니다.")
        return
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR,
                        time.strftime("%Y-%m-%d_%H%M%S") + "_pick_grip_probe.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n  기록 저장: {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=5,
                    help="각 상태를 몇 번씩 잴지 (기본 5). 번갈아 잰다")
    ap.add_argument("--preset", default="close",
                    help="조일 때 쓸 프리셋. 피크 파지는 close 다 (pick.holder.grip_preset)")
    ap.add_argument("--open-preset", default="open", help="회차 사이에 벌릴 프리셋")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--host", default="192.168.1.1")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65)
    ap.add_argument("--width-reg", type=int, default=9,
                    help="폭이 들어 있는 레지스터 번호 (258+9, 2026-09-21 실물 확인)")
    ap.add_argument("--settle", type=float, default=0.8,
                    help="여닫으라고 한 뒤 멈추기를 기다리는 시간(초)")
    ap.add_argument("--no-robot", action="store_true",
                    help="로봇에 안 붙는다. 사람이 직접 여닫고 이 도구는 폭만 읽는다")
    ap.add_argument("--simulate", action="store_true",
                    help="모의 폭으로 결과 화면만 본다. 실측이 아니다")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    print(f"\n{'=' * 62}")
    print("피크 파지 — 폭으로 가릴 수 있나")
    print(f"{'=' * 62}")
    print(f"  조이는 프리셋  {args.preset}        벌리는 프리셋  {args.open_preset}")
    print(f"  회차           {EMPTY}·{PICK} {args.trials}회씩, 번갈아")
    print(f"  폭 레지스터    258+{args.width_reg} (0.1mm 단위)")
    print("  팔은 움직이지 않는다. 그리퍼만 여닫힌다 — 손가락이 물리지 않게 주의.")
    if args.simulate:
        print("  ⚠ --simulate 다. 지어낸 값이라 아무 근거도 되지 않는다.")

    rg2 = None
    node = None
    rclpy = None
    if not (args.no_robot or args.simulate):
        from tunio_robot import teach_points as TP
        from tunio_robot.gripper import RG2, parse_presets
        points = TP.load(os.path.abspath(args.yaml))
        presets = parse_presets(points.gripper.get("presets"))
        for name in (args.preset, args.open_preset):
            if name not in presets:
                raise SystemExit(f"'{name}' 프리셋이 없다. 있는 것: {sorted(presets)}")

        import rclpy as _rclpy
        rclpy = _rclpy
        import DR_init
        DR_init.__dsr__id = args.id
        DR_init.__dsr__model = args.model
        rclpy.init()
        # 네임스페이스가 있어야 DSR_ROBOT2 의 상대 서비스 이름이 로봇에 닿는다
        node = rclpy.create_node("tunio_pick_grip_probe", namespace=args.id)
        DR_init.__dsr__node = node
        try:
            import DSR_ROBOT2
        except ImportError as e:
            raise SystemExit(f"두산 API import 실패: {e}. 드라이버와 source 를 확인하라")
        # motion_primitives.connect 를 안 부른다 — 팔을 안 움직이므로 TCP·툴 검증이
        # 필요 없고, 거기서 막혀 그리퍼 시험을 못 하는 일을 만들지 않는다
        rg2 = RG2(DSR_ROBOT2, presets, settle_sec=points.gripper.get("settle_sec", 0.3))

    got = {EMPTY: [], PICK: []}
    try:
        for n in range(1, args.trials + 1):
            for label in (EMPTY, PICK):
                width = one_trial(args, rg2, label, n, args.trials)
                if width is not None:
                    got[label].append(width)
    except (KeyboardInterrupt, SystemExit) as e:
        print(f"\n중단했다 ({e or 'Ctrl+C'}). 여기까지 잰 것으로 정리한다.")
    finally:
        if rg2 is not None:
            try:
                rg2.select(args.open_preset)    # 열어 두고 끝낸다
            except Exception as e:              # noqa: B902
                print(f"  ⚠ 마지막에 그리퍼를 못 열었다: {e}")
        if node is not None:
            node.destroy_node()
        if rclpy is not None:
            rclpy.shutdown()

    report(args, got)
    return 0


if __name__ == "__main__":
    sys.exit(main())
