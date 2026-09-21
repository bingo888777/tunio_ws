#!/usr/bin/env python3
"""
loop_check — 조율 루프 한 바퀴를 Action 으로 돌려 본다. 정묵·성현 통합 확인용.

무엇을 보나
  발현(성현)과 페그 조절(정묵)이 **한 루프에서 번갈아 돌 때** 팔이 꼬이거나
  경로가 기타를 쓰는지. 지금까지 실동작은 각자 도구로만 했다 —
  pluck_dummy_test(발현), peg_register(페그). 둘이 붙은 적이 없다.

  사이에 피크 왕복이 낀다. 발현 전에 집고, 페그 잡기 전에 돌려놓는다.
  그 왕복 경로가 이 도구가 보려는 것이다.

    play_string   피크 집기 -> 6번 줄 튕기기
    adjust_peg    피크 반환 -> 경유지 -> 페그 물고 돌리기
    play_string   피크 다시 집기 -> 튕기기

준비
  · robot_controller 가 **dry_run:=false** 로 떠 있어야 한다
      ros2 run tunio_robot robot_controller --ros-args
          -p dry_run:=false -p peg_register_force:=false
  · 거치대에 피크가 있어야 한다
  · Phase 0 등록이 끝나 있어야 한다 (tools/peg_register.py)
  · 펜던트에서 GripperDA_v1 · Tool Weight 켜 둘 것

안전
  · 기본 3도만 **푸는 쪽**으로 돌린다. 조이는 쪽은 줄이 끊어질 수 있다
  · 단계마다 확인을 받는다. 비상정지에 손을 올려두고 볼 것

쓰는 법
  python3 tools/loop_check.py                    # 6번 줄, 3도 풀기
  python3 tools/loop_check.py --string 1 --angle 2
  python3 tools/loop_check.py --skip-pluck       # 페그만 (발현 미검증일 때)
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tunio_interfaces.action import AdjustPeg, PlayString, StrumAll


def confirm(msg):
    print(msg)
    if input("    >>> 엔터 = 진행 / n = 중단: ").strip().lower() == "n":
        raise KeyboardInterrupt


def send(node, cli, goal, label, timeout=120.0):
    """Goal 을 보내고 끝날 때까지 기다린다. 성공 여부를 돌려준다."""
    print(f"\n── {label}")
    if not cli.wait_for_server(timeout_sec=10.0):
        print("    ✗ Action 서버가 없다. robot_controller 가 떠 있나?")
        return False
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("    ✗ goal 이 거부됐다 (다른 동작이 도는 중일 수 있다)")
        return False
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=timeout)
    if rf.result() is None:
        print(f"    ✗ {timeout:.0f}초 안에 안 끝났다")
        return False
    res = rf.result().result
    ok = bool(getattr(res, "success", False))
    msg = getattr(res, "message", "") or ""
    print(f"    {'✓' if ok else '✗'} success={ok}  {msg}")
    return ok


HELP = """
  명령                          하는 일
  ----------------------------------------------------------------
  play <줄>                     그 줄을 튕긴다 (피크가 없으면 집어 온다)
  peg <줄> <loosen|tighten> <도>  페그를 돌린다 (피크를 쥐고 있으면 먼저 반환)
  strum                         전체 화음 (아직 미구현 — 실패가 정상)
  ?                             이 도움말
  q                             나가기

  줄 번호는 6 5 2 1. 예 —
    play 6
    peg 6 loosen 3
    play 6
    peg 1 tighten 2
"""


def repl(node, play, adj, strum):
    """
    ROS Action 을 손으로 쏴 본다. 서버가 보내는 것과 같은 goal 이다.

    순서를 마음대로 섞을 수 있어서 노드의 판단을 확인하기 좋다 —
    피크를 언제 집고 놓는지, 경유지를 언제 타는지, Phase 0 를 언제 도는지는
    전부 robot_controller 안에 있고 **서버는 지시하지 않는다**(CLAUDE.md 2절).
    발현 두 번을 연달아 시키면 집기가 한 번만 나와야 맞다.
    """
    print(HELP)
    while True:
        try:
            line = input("loop> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line:
            continue
        w = line.split()
        cmd = w[0].lower()
        try:
            if cmd in ("q", "quit", "exit"):
                return 0
            if cmd in ("?", "h", "help"):
                print(HELP)
            elif cmd == "play":
                g = PlayString.Goal()
                g.string_number = int(w[1])
                send(node, play, g, f"play_string {w[1]}")
            elif cmd == "peg":
                g = AdjustPeg.Goal()
                g.string_number = int(w[1])
                g.direction = -1 if w[2].startswith("l") else 1
                g.angle_deg = float(w[3])
                send(node, adj, g, f"adjust_peg {w[1]} {w[2]} {w[3]}도")
            elif cmd == "strum":
                send(node, strum, StrumAll.Goal(), "strum_all")
            else:
                print(f"  모르는 명령: {cmd}   ? 로 도움말")
        except (IndexError, ValueError):
            print("  인자가 모자라거나 숫자가 아니다.  ? 로 도움말")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--string", type=int, default=6, choices=[1, 2, 5, 6])
    ap.add_argument("--angle", type=float, default=3.0, help="페그 회전량(도)")
    ap.add_argument("--direction", choices=["loosen", "tighten"], default="loosen",
                    help="기본은 푸는 쪽. 조이는 쪽은 줄이 끊어질 수 있다")
    ap.add_argument("--skip-pluck", action="store_true",
                    help="발현을 건너뛴다. 페그 조절만 본다")
    ap.add_argument("--yes", action="store_true", help="확인 없이 쭉 돈다")
    ap.add_argument("-i", "--interactive", action="store_true",
                    help="정해진 순서 대신 **손으로 명령을 쳐서** 돌린다. "
                         "서버가 보내는 것과 같은 goal 이라 순서를 마음대로 섞어 볼 수 있다")
    args = ap.parse_args()

    no = args.string
    direction = -1 if args.direction == "loosen" else 1

    print()
    print("=" * 66)
    print("조율 루프 확인 — 손으로 명령" if args.interactive
          else f"조율 루프 확인 — {no}번 줄")
    print("=" * 66)
    if not args.interactive:
        print(f"  발현      {'건너뜀' if args.skip_pluck else f'{no}번 줄 (피크 집기 포함)'}")
        print(f"  페그      {args.direction} {args.angle:.1f}도 (direction={direction:+d})")
    print("  ⚠️ robot_controller 가 dry_run:=false 로 떠 있어야 한다")
    print("  ⚠️ 거치대에 피크가 있어야 하고 Phase 0 등록이 끝나 있어야 한다")

    rclpy.init()
    node = Node("tunio_loop_check")
    play = ActionClient(node, PlayString, "/robot/play_string")
    adj = ActionClient(node, AdjustPeg, "/robot/adjust_peg")
    strum = ActionClient(node, StrumAll, "/robot/strum_all")

    if args.interactive:
        try:
            return repl(node, play, adj, strum)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    ok = True
    try:
        if not args.skip_pluck:
            if not args.yes:
                confirm("\n[1/3] 피크를 집고 줄을 튕긴다. 거치대 왕복 경로를 본다.")
            g = PlayString.Goal()
            g.string_number = no
            ok = send(node, play, g, f"[1/3] {no}번 발현") and ok

        if not args.yes:
            confirm(f"\n[2/3] 피크를 반환하고 {no}번 페그를 {args.angle:.0f}도 돌린다.\n"
                    "      **페그 -> 거치대 경로가 이번에 처음 도는 구간이다.**")
        a = AdjustPeg.Goal()
        a.string_number = no
        a.direction = direction
        a.angle_deg = float(args.angle)
        ok = send(node, adj, a, f"[2/3] {no}번 페그 {args.direction} {args.angle:.0f}도") and ok

        if not args.skip_pluck:
            if not args.yes:
                confirm("\n[3/3] 피크를 다시 집고 튕긴다. 여기까지 되면 루프가 돈다.")
            g2 = PlayString.Goal()
            g2.string_number = no
            ok = send(node, play, g2, f"[3/3] {no}번 재발현") and ok
    except KeyboardInterrupt:
        print("\n중단했다. 팔은 그 자리에 선다. 피크를 쥐고 있을 수 있으니 확인할 것")
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n" + "=" * 66)
    if ok:
        print("  전부 통과. 발현 ↔ 페그 왕복이 실기로 돈다")
        print("  다음 — 반대쪽 줄로 바꿔 좌우 전환까지 보라:")
        print(f"    python3 tools/loop_check.py --string {1 if no in (5, 6) else 6}")
    else:
        print("  실패한 단계가 있다. 위 메시지를 그대로 보고하라")
        print("  피크를 쥔 채 멈췄으면 그리퍼를 확인하고 노드를 다시 띄울 것")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
