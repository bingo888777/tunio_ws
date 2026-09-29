#!/usr/bin/env python3
"""
grip_cycle — 그리퍼를 여닫으며 **닫힌 폭**을 읽는다. 팔은 안 움직인다.

무엇을 하나
  열기 → (엔터) → 닫기 + 폭 측정 → (엔터) → 다시 열기.  Ctrl+C 로 끝낸다.
  끝낼 때 회차별 폭과 흩어짐을 요약한다.

무엇에 쓰나
  손가락 사이에 낀 것의 **두께**를 재는 것이다. 빈손으로 닫으면 13.0mm 에서
  멈추고, 뭔가 끼어 있으면 그만큼 더 벌어진 채 멈춘다.
    · 피크를 물린 폭이 빈손과 갈리는지 (A안의 유일한 미지수)
    · 페그 손잡이 물림 19.6 / 사점 32.8 / 빈손 13.0 의 재현성
  peg_grip_probe.py 와 달리 **로봇을 페그로 보내지 않는다.** 그리퍼만 쓴다.

경로가 둘이다. 헷갈리지 말 것 (CLAUDE.md 3.6)
  제어 — 로봇의 디지털 출력 → Compute Box 의 WebLogic 프리셋.
         폭·힘은 웹에서 정하고 코드는 프리셋을 고를 뿐이다.
  읽기 — Compute Box 의 Modbus 홀딩 레지스터 258+9 를 **읽기만** 한다.
         기능 코드 3 만 보내므로 구조적으로 아무것도 못 쓴다.

파지 감지 플래그는 안 쓴다
  레지스터 11 은 **빈손일 때도 '있음' 으로 읽힌다** (2026-09-21 실측).
  이 도구는 폭(258+9)만 본다.

TCP·툴
  connect() 가 set_tool("Tool Weight") / set_tcp("GripperDA_v1") 를 켜고 get 으로
  대조한다. 못 맞추면 여기서 멈춘다 (#23, #26). 그리퍼만 쓰는 도구라 TCP 가 꼭
  필요하진 않지만, 다른 도구와 같은 입구를 쓰는 편이 상태가 섞이지 않는다.
  여기서 막히면 `python3 tools/tcp_diag.py` 로 원인을 가른다.

안전
  · 팔은 전혀 안 움직인다. 디지털 출력만 켜고 끈다
  · 손가락이 물리면 다친다. **그리퍼 주변을 비우고 시작한다**
  · 끝날 때(Ctrl+C 포함) 반드시 열고 나간다

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/grip_cycle.py                      # close(15mm) 로 조인다
  python3 tools/grip_cycle.py --preset peg         # peg(21mm) 로
  python3 tools/grip_cycle.py --label "피크 2장"   # 기록에 남길 이름
  python3 tools/grip_cycle.py --save               # docs/robot_data/ 에 JSON
  python3 tools/grip_cycle.py --dry-run            # 로봇 없이 흐름만
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")
DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..",
                            "src", "tunio_robot", "config", "teach_points.yaml")


def confirm(msg):
    """엔터가 아니면 멈춘다. peg_grip_probe 와 같은 규칙이다."""
    ans = input(f"{msg}\n  계속하려면 엔터, 그만두려면 q + 엔터 > ").strip().lower()
    if ans == "q":
        raise KeyboardInterrupt
    return ans


def settled_width(read, settle, samples=3, gap=0.15):
    """
    닫으라고 한 뒤 기다렸다가 **여러 번 읽어** 아직 움직이는지 본다.

    busy 플래그(레지스터 10)로 판정하지 않는다 — 그 칸의 배치가 아직 추정이라
    (CLAUDE.md 3.6) 믿을 수 없다. 대신 값이 흔들리는지를 직접 본다.

    반환 (대표값, 흩어짐, 읽은 값들). 대표값은 마지막 읽기다.
    """
    time.sleep(settle)
    vals = []
    for i in range(samples):
        if i:
            time.sleep(gap)
        vals.append(read())
    spread = max(vals) - min(vals)
    return vals[-1], spread, vals


def summarize(rows, close_mm, empty_ref):
    print(f"\n{'=' * 66}")
    print("요약")
    print(f"{'=' * 66}")
    if not rows:
        print("  잰 회차가 없다.")
        return
    widths = [r["width_mm"] for r in rows]
    lo, hi = min(widths), max(widths)
    avg = sum(widths) / len(widths)
    print(f"  회차 {len(rows)}개")
    for r in rows:
        mark = "  ⚠ 아직 움직이는 중" if r["spread_mm"] > 0.2 else ""
        print(f"    {r['n']:2d}회  {r['width_mm']:6.1f} mm"
              f"   (흩어짐 {r['spread_mm']:.1f}){mark}")
    print()
    print(f"  최소 {lo:.1f} / 평균 {avg:.1f} / 최대 {hi:.1f} mm   폭 {hi - lo:.1f}mm")
    if close_mm is not None:
        print(f"  프리셋 기록값 {close_mm}mm — 실제로 여기서 멈춘 것과 다르면 웹 값을 확인하라")
    if empty_ref is not None:
        d = avg - empty_ref
        print(f"  빈손 기준선 {empty_ref:.1f}mm 대비 {d:+.1f}mm")
        if abs(d) < 0.3:
            print("    → 빈손과 구분이 안 된다. 손가락 사이에 아무것도 없거나,")
            print("       고무 골무가 얇은 것을 먹어버린 것이다")
        else:
            print("    → 갈린다. 이 차이가 곧 물린 것의 두께다")
    print()
    print("  ⚠ 파지 감지 플래그(레지스터 11)는 이 도구가 보지 않는다.")
    print("     빈손일 때도 '있음' 으로 읽혀 못 쓴다 (2026-09-21 실측).")


def main():
    ap = argparse.ArgumentParser(
        description="그리퍼를 여닫으며 닫힌 폭을 읽는다. 팔은 안 움직인다.")
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--preset", default="close",
                    help="조일 때 쓸 프리셋 (기본 close). peg 는 21mm 라 "
                         "손잡이보다 넓어 두께를 못 잰다")
    ap.add_argument("--open-preset", default="open")
    ap.add_argument("--settle", type=float, default=1.0,
                    help="닫으라고 한 뒤 멈추기를 기다리는 시간(초)")
    ap.add_argument("--samples", type=int, default=3,
                    help="한 회차에 몇 번 읽어 흔들림을 볼지")
    ap.add_argument("--label", default="",
                    help="이 회차가 뭘 물린 건지. 기록에 남는다 (예: '피크 2장')")
    ap.add_argument("--empty-ref", type=float, default=13.0,
                    help="빈손 기준선(mm). 요약에서 대조용. 0 이면 안 쓴다")
    ap.add_argument("--host", default="192.168.1.1", help="Compute Box 주소")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65, help="퀵 체인저 주소")
    ap.add_argument("--width-reg", type=int, default=9,
                    help="폭 레지스터. gripper_status.py --scan 으로 확인한 값")
    ap.add_argument("--save", action="store_true", help="docs/robot_data/ 에 남긴다")
    ap.add_argument("--dry-run", action="store_true", help="로봇 없이 흐름만")
    args = ap.parse_args()

    from tunio_robot import teach_points as TP
    points = TP.load(args.yaml)

    presets = points.gripper.get("presets", {})
    if args.preset not in presets:
        print(f"프리셋 '{args.preset}' 이 교시점 파일에 없다. "
              f"있는 것: {', '.join(sorted(presets))}")
        return 1
    close_mm = presets[args.preset].get("width_mm")
    open_mm = presets.get(args.open_preset, {}).get("width_mm")

    print()
    print("=" * 66)
    print("그리퍼 여닫기 — 닫힌 폭 측정")
    print("=" * 66)
    print(f"  조일 때   {args.preset} ({close_mm}mm 기록)")
    print(f"  열 때     {args.open_preset} ({open_mm}mm 기록)")
    if args.label:
        print(f"  물린 것   {args.label}")
    print(f"  읽기      {args.host}:{args.port} 유닛 {args.unit} "
          f"레지스터 {258 + args.width_reg}")
    print()
    print("  팔은 움직이지 않는다. 그리퍼만 여닫힌다.")
    print("  ⚠ 손가락이 물리면 다친다. 그리퍼 주변을 비우고 시작하라.")

    MP = read = node = None
    if not args.dry_run:
        import rclpy
        import DR_init
        DR_init.__dsr__id = points.robot["id"]
        DR_init.__dsr__model = points.robot["model"]
        rclpy.init()
        # 네임스페이스가 있어야 DSR_ROBOT2 의 상대 경로 서비스가 로봇에 닿는다
        node = rclpy.create_node("tunio_grip_cycle", namespace=points.robot["id"])
        DR_init.__dsr__node = node

        from tunio_robot import motion_primitives as MP_
        from tunio_robot.grip_width import make_reader, WidthError
        MP = MP_
        try:
            # 여기서 set_tool / set_tcp 를 켜고 get 으로 대조한다 (#23, #26)
            MP.connect(points.robot, points.gripper)
        except MP.MotionError as e:
            print(f"\n연결 실패: {e}")
            print("  TCP·툴 설정이 원인이면 진단 도구가 따로 있다 —")
            print("    python3 tools/tcp_diag.py")
            return 1
        read = make_reader(args.host, args.port, args.unit, args.width_reg)
        try:
            now = read()
        except WidthError as e:
            print(f"\n폭을 못 읽는다: {e}")
            print("  · Compute Box 가 이 PC 에서 보이나? ping 으로 확인하라")
            print("  · 주소를 모르면  python3 tools/gripper_status.py --scan")
            return 1
        print(f"\n  지금 폭 {now:.1f} mm")

    rows = []
    n = 0
    try:
        confirm("\n그리퍼를 연다.")
        if MP:
            MP._grip(args.open_preset)
            time.sleep(args.settle)

        while True:
            n += 1
            confirm(f"\n[{n}회] 조인다 ({args.preset}).")
            if MP:
                MP._grip(args.preset)
                w, spread, vals = settled_width(read, args.settle, args.samples)
                reads = " ".join(f"{v:.1f}" for v in vals)
                print(f"  폭 {w:.1f} mm   (읽은 값 {reads}, 흩어짐 {spread:.1f})")
                if spread > 0.2:
                    print("  ⚠ 아직 움직이는 중일 수 있다. --settle 을 늘려 보라")
                rows.append({"n": n, "width_mm": w, "spread_mm": spread,
                             "reads": vals, "label": args.label})
            else:
                print("  [dry-run] 여기서 조이고 폭을 읽는다")

            confirm(f"[{n}회] 다시 연다.")
            if MP:
                MP._grip(args.open_preset)
                time.sleep(args.settle)
    except KeyboardInterrupt:
        print("\n\n끝낸다.")
    except Exception as e:                          # noqa: BLE001
        print(f"\n\n중단 — {e!r}")
    finally:
        # 물린 채로 두고 나가지 않는다
        if MP:
            try:
                MP._grip(args.open_preset)
            except Exception as e:                  # noqa: BLE001
                print(f"  ⚠ 마지막 열기 실패: {e!r}. 그리퍼를 직접 확인하라")
        if node is not None:
            import rclpy
            node.destroy_node()
            rclpy.shutdown()

    summarize(rows, close_mm, args.empty_ref or None)

    if args.save and rows:
        from tunio_robot import runlog
        path = runlog.save("grip_cycle", {
            "tool": "grip_cycle",
            "label": args.label,
            "preset": args.preset,
            "preset_width_mm": close_mm,
            "open_preset": args.open_preset,
            "host": args.host, "unit": args.unit,
            "width_register": args.width_reg,
            "settle_sec": args.settle,
            "empty_ref_mm": args.empty_ref,
            "rows": rows,
        }, LOG_DIR)
        if path:
            print(f"  기록 저장: {os.path.relpath(path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
