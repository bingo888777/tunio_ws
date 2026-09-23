#!/usr/bin/env python3
"""
페그 회전 확인 — 로봇 점검 체크리스트 3단계. 토크 기록(6단계)도 같이 얻는다.

페그를 물고 **조금씩 돌리면서 관절 토크를 읽는다.** 세 가지를 한 번에 본다.

  1. 회전 방향 — 반시계가 조임이라는 전제(turn_sign)가 맞는지
  2. 백래시 — 헛돎을 지나 저항이 걸리기 시작하는 각도
  3. 파지 확인 임계값 — 제대로 물었을 때와 헛잡았을 때의 토크 차이

⚠️ **첫 시험은 반드시 푸는 방향으로 한다.** 그게 기본값이다.
   조이는 쪽으로 방향이 틀리면 줄이 끊어진다. 푸는 쪽은 틀려도 음이 올라갈 뿐이다.
   음이 내려가면 turn_sign 이 맞는 것이고, 올라가면 네 줄 전부 부호를 뒤집는다.
   여기에는 마이크가 없다. 귀로 판정하거나 정정인 쪽 도구를 같이 돌려라.

왜 조금씩 나눠 도는가
  두산 API 의 movej 는 끝날 때까지 안 돌아와서 도는 중에 토크를 못 읽는다.
  그래서 작은 각도씩 끊어 돌고 멈춘 자리에서 읽는다. 그러면 각도별 토크가
  줄줄이 남아, 헛돎이 끝나고 저항이 붙는 지점이 눈에 보인다.

헛잡은 것과 비교하려면
  --no-grip 으로 한 번 더 돌려라. 그리퍼를 안 닫고 허공에서 도는 토크가 바닥값이다.
  그 둘이 갈리는 지점이 파지 확인 임계값이다 (CLAUDE.md 3.6).

줄 끊어짐 감지(2026-09-22, 예외처리 B안)를 보려면
  --string cut 으로 한 번 더 돌려라. 페그는 정상으로 물되(그리퍼는 잡는다),
  줄이 없거나 장력이 없는 상태(끊어놓은 폐줄 등)로 돈다. 기본값은
  --string intact(줄 정상)다. 파지 여부(--no-grip)와는 다른 축이라 둘을
  섞지 말 것 — 줄 유무 비교는 **둘 다 문 상태**에서만 뜻이 있다.
  같은 페그·방향·각도·속도·팔 모양·파지 프리셋으로 intact 한 번, cut 한 번
  돌리면 자동으로 견줘 준다. 조건이 다르면 비교를 건너뛴다(#79 리뷰, 정묵).

  ⚠️ **센서 경로는 새 게 아니다.** get_external_torque()(조인트 6)는
  처음부터 이 도구가 쓰던 것이다(f788f56). 여기서 새로 더하는 건 '정상 줄 대
  무장력 줄' 비교 축이지 측정 경로가 아니다. 샘플링도 회전 중 연속이 아니라
  movej 스텝이 끝날 때마다다 — '조인트 6은 실측이 없다'는 예전 설명은 틀렸다.
  이미 있던 파지-허공 비교(0.023Nm 차이, 0.11Nm 잡음)가 바로 그 실측이고,
  결과가 안 좋다 — 자세한 판정은 바로 아래 경고를 볼 것.

  ⚠️ **자동 판정("가릴 수 있다")을 믿지 말 것.** `step_from_rest` 차이가
  잡음의 2배를 넘는지만 보는데, 이 값은 **정지 baseline 이 조건마다 흔들리는
  것**(예: 파지 +0.07 vs 허공 +0.433)까지 회전 저항으로 잡아낸다. 9/19 파지·
  9/21 허공 기록을 이 식에 넣으면 "가릴 수 있다"로 통과하지만, 그건 우리가
  이미 아니라고 결론 낸 사례다(CLAUDE.md 3.6). **숫자만 보고 판단은 사람이
  한다** — 같은 세션에서 조건 맞춰 반복 측정한 뒤에.

쓰는 법
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/peg_turn_check.py --peg 6                    # 푸는 쪽 20도
  python3 tools/peg_turn_check.py --peg 6 --angle 30 --step 2.5
  python3 tools/peg_turn_check.py --peg 6 --no-grip          # 허공 바닥값
  python3 tools/peg_turn_check.py --peg 6 --direction tighten   # 방향 확인 뒤에만
  python3 tools/peg_turn_check.py --peg 6 --string cut        # 줄 없이(장력 0) 회전
  python3 tools/peg_turn_check.py --peg 6 --dry-run
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

DEFAULT_YAML = os.path.join(os.path.dirname(__file__), "..", "src", "tunio_robot",
                            "config", "teach_points.yaml")
LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")


def MP_turn(cfg, percent):
    """회전용 속도 설정. motion.turn_vel 에 비율을 곱한다."""
    out = dict(cfg)
    scale = max(0.05, percent / 100.0)
    out["vel"] = cfg.get("turn_vel", cfg.get("vel", 5.0)) * scale
    out["acc"] = cfg.get("turn_acc", cfg.get("acc", 10.0)) * scale
    return out


def fmt(vals, d=2):
    return "[" + ", ".join(f"{v:7.{d}f}" for v in vals) + "]"


def confirm(msg):
    if input(f"{msg} (엔터=진행, n=중단) ").strip().lower().startswith("n"):
        raise KeyboardInterrupt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--peg", type=int, required=True, choices=[1, 2, 5, 6])
    ap.add_argument("--yaml", default=DEFAULT_YAML)
    ap.add_argument("--direction", choices=["loosen", "tighten"], default="loosen",
                    help="첫 시험은 푸는 쪽(loosen)으로 한다. 기본값이다")
    ap.add_argument("--angle", type=float, default=20.0, help="총 회전량(도)")
    ap.add_argument("--step", type=float, default=2.5, help="한 번에 도는 각도(도)")
    ap.add_argument("--approach", type=float, default=None)
    ap.add_argument("--speed", type=float, default=100.0,
                    help="**이동** 속도. 설정값의 몇 %% 로. "
                         "처음 가보는 경로는 30 쯤으로 낮춰라")
    ap.add_argument("--turn-speed", type=float, default=100.0,
                    help="**회전** 속도. motion.turn_vel 의 몇 %% 로. "
                         "이동과 따로 둔다")
    ap.add_argument("--no-grip", action="store_true",
                    help="그리퍼를 안 닫고 허공에서 돈다. 토크 바닥값을 얻는다")
    ap.add_argument("--string", choices=["intact", "cut"], default="intact",
                    help="줄이 정상(intact)인지 끊겨서 장력이 없는지(cut). "
                         "--no-grip 과는 다른 축 — 페그를 문 상태에서만 뜻이 있다")
    ap.add_argument("--no-return", action="store_true",
                    help="돌린 만큼 되돌리지 않는다. 기본은 되돌린다")
    ap.add_argument("--continuous", action="store_true",
                    help="끊어서(movej) 돌리지 않고 실제 운영 코드처럼 amovej 로 "
                         "한 번에 연속으로 돌면서 폴링마다 토크를 읽는다. "
                         "2026-09-23 실기에서 끊어서 잰 문턱(0.25Nm)이 연속회전 "
                         "실측(diff 0.05~0.12)과 3~6배 차이나서 추가함 — "
                         "_turn_peg_async_with_string_check 와 같은 방식으로 재야 "
                         "그 코드가 실제로 쓸 문턱이 나온다")
    ap.add_argument("--poll-sec", type=float, default=0.2,
                    help="--continuous 일 때 폴링 간격(초). motion.string_break_poll_sec 기본값과 맞춘다")
    ap.add_argument("--preset", default="peg")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--id", default="dsr01")
    ap.add_argument("--model", default="m0609")
    args = ap.parse_args()

    from tunio_robot import runlog
    from tunio_robot import teach_points as TP
    points = TP.load(os.path.abspath(args.yaml))
    peg = points.peg(args.peg)
    back_mm = args.approach if args.approach is not None \
        else points.motion.get("approach_mm", 100.0)
    back = TP.backed_off(peg.task, back_mm)
    route = points.route()
    if route is None:
        raise SystemExit("경유지(routes.transit)가 미교시다")

    if args.no_grip and args.string == "cut":
        print("--no-grip 과 --string cut 은 같이 못 쓴다. "
              "줄 유무는 페그를 문 상태에서만 뜻이 있다 (허공은 애초에 줄이 안 걸린다).")
        return 1

    direction = TP.TIGHTEN if args.direction == "tighten" else TP.LOOSEN
    joint, total = points.joint_delta(args.peg, direction, args.angle)
    step_signed = (total / abs(total)) * min(args.step, abs(total))
    n_steps = int(round(abs(total) / abs(step_signed)))

    motion = dict(points.motion)
    motion.pop("transit_tcp_vel", None)  # 실측 도구의 --speed를 우선
    motion.pop("transit_tcp_acc", None)
    scale = max(0.05, args.speed / 100.0)
    for key, default in (("vel", 30), ("acc", 30)):
        motion[key] = points.motion.get(key, default) * scale
    for key, default in (("travel_vel", [50.0, 30.0]), ("travel_acc", [100.0, 60.0])):
        motion[key] = [v * scale for v in points.motion.get(key, default)]

    # 회전은 이동과 따로 간다. 줄에 장력을 거는 구간이라 느려야 한다
    turn = MP_turn(points.motion, args.turn_speed)

    print()
    print("=" * 70)
    print(f"{args.peg}번 페그 회전 확인  ({TP.side_of(args.peg)} 쪽)")
    print("=" * 70)
    print(f"  방향       {args.direction}  "
          f"({'조임 — 줄이 끊어질 수 있다' if direction == TP.TIGHTEN else '풀기 — 안전한 쪽'})")
    print(f"  회전 관절  J{joint}   turn_sign {peg.turn_sign}")
    print(f"  총 {args.angle:.1f}도 를 {abs(step_signed):.2f}도씩 {n_steps}번 나눠 돈다")
    print(f"  이동 속도  {motion['vel']:.1f} deg/s   "
          f"회전 속도  {turn['vel']:.2f} deg/s")
    print(f"  관절 변화  {total:+.2f}도 (부호 포함)")
    print(f"  파지       {'안 한다 — 허공 바닥값' if args.no_grip else args.preset}")
    if not args.no_grip:
        print(f"  줄         {'정상(intact)' if args.string == 'intact' else '끊음(cut) — 장력 0 기준값'}")
    print(f"  되돌리기   {'안 한다' if args.no_return else '돌린 만큼 되돌린다'}")
    if direction == TP.TIGHTEN:
        print()
        print("  ⚠️  조이는 방향이다. 푸는 쪽으로 방향을 먼저 확인했나?")
        print("     확인 안 했으면 지금 멈추고 --direction loosen 부터 하라.")
    if args.dry_run:
        print("\n--dry-run 이라 여기서 멈춘다")
        return 0

    import rclpy

    import DR_init
    DR_init.__dsr__id = args.id
    DR_init.__dsr__model = args.model
    rclpy.init()
    node = rclpy.create_node("tunio_turn_check", namespace=args.id)
    DR_init.__dsr__node = node

    from tunio_robot import arm_pose
    from tunio_robot import motion_primitives as MP
    MP.connect(points.robot, points.gripper)
    from DR_common2 import posj, posx
    from DSR_ROBOT2 import (amovej, check_motion, get_current_posj,
                            get_external_torque, ikin)

    # 팔 모양을 안 고르면 경유지(특이점)의 모호한 현재 자세를 따라가 매번 달라진다
    sol = peg.sol
    if sol is None:
        got = arm_pose.choose(lambda t: ikin(posx(back), t), route.joints[0])
        if got is None:
            raise SystemExit("쓸 만한 팔 모양이 없다. tools/sol_check.py 로 보라")
        sol, _, casing = got
        print(f"  팔 모양   자동 선택 {sol} (손목 케이싱 X {casing[0]:.0f}mm)")
        print(f"            pegs.{args.peg}.sol 에 적어 두면 매번 같은 길로 간다")
    else:
        print(f"  팔 모양   yaml 에 적힌 값 {sol}")

    rows = []
    heard = ""
    log = {
        "tool": "peg_turn_check", "peg": args.peg, "side": TP.side_of(args.peg),
        "direction": args.direction, "gripped": not args.no_grip,
        "preset": None if args.no_grip else args.preset,
        "string": None if args.no_grip else args.string,
        "torque_source": "joint6_external",
        "continuous": args.continuous,
        "angle_deg": args.angle, "step_deg": args.step,
        "joint": joint, "turn_sign": peg.turn_sign, "sol": sol,
        "turn_vel": turn["vel"], "travel_vel": motion["vel"],
        "approach_mm": back_mm, "steps": [],
    }
    try:
        confirm("\n그리퍼를 연다(open 프리셋). 손이 끼지 않게 치워라.")
        MP._grip("open")
        confirm("경유지로 올라간다. 팔이 크게 움직인다.")
        MP.goto_route(route, motion)
        confirm("접근점으로 간다.")
        MP._movejx(back, motion, sol=sol)
        confirm("페그 앞까지 직선으로 들어간다.")
        MP._movel(list(peg.task), motion)

        if not args.no_grip:
            confirm(f"'{args.preset}' 로 문다.")
            MP._grip(args.preset)

        base_j = list(get_current_posj())
        base_t = list(get_external_torque())
        log["before"] = {"joints": base_j, "torque": base_t}
        print(f"\n  돌리기 전  J{joint} {base_j[joint - 1]:.2f}도   "
              f"토크 {base_t[joint - 1]:+.2f}")
        print("\n  줄 소리를 들어라. 음이 내려가면 turn_sign 이 맞는 것이다.")
        confirm(f"\n{args.angle:.1f}도를 {abs(step_signed):.2f}도씩 돌린다.")

        if args.continuous:
            # 끊어서 움직이지 않는다 — 운영 코드(motion_primitives.py
            # _turn_peg_async_with_string_check)와 같은 방식: amovej 로 전체
            # 각도를 한 번에 걸고, 도는 동안 폴링마다(끊지 않은 채로) 토크를 읽는다.
            # 문턱은 여기서 안 판정한다 — 그냥 있는 그대로 기록만 한다.
            print(f"\n  {'t(s)':>8}  {'토크':>8}  {'바닥대비':>8}  상태")
            d = [0.0] * 6
            d[joint - 1] = total
            target_angle = base_j[joint - 1] + total
            ret = amovej(posj(d), vel=turn["vel"], acc=turn["acc"],
                        mod=MP._api.DR_MV_MOD_REL)
            MP._moved(ret, "amovej")
            waited = 0.0
            seen_busy = False
            arrive_tol = points.motion.get("string_break_arrive_tolerance_deg", 0.5)
            max_wait = points.motion.get("string_break_max_wait_sec", 30.0)
            while True:
                status = check_motion()
                j = list(get_current_posj())
                t = list(get_external_torque())
                moved = j[joint - 1] - base_j[joint - 1]
                tq = t[joint - 1]
                diff = tq - base_t[joint - 1]
                rows.append((waited, moved, tq, diff))
                log["steps"].append({"t": round(waited, 3), "moved_deg": moved,
                                     "torque": tq, "torque_all": t, "joints": j,
                                     "status": status})
                print(f"  {waited:8.2f}  {tq:+8.2f}  {diff:+8.2f}  {status}")
                if status not in (0, 1, 2):
                    raise SystemExit(f"check_motion 이 알 수 없는 값: {status}")
                if status == 2:
                    seen_busy = True
                if status == 0 and seen_busy and abs(j[joint - 1] - target_angle) <= arrive_tol:
                    break
                if waited >= max_wait:
                    raise SystemExit(f"{max_wait:.0f}초 안에 안 끝났다 — 직접 확인할 것")
                time.sleep(args.poll_sec)
                waited += args.poll_sec
        else:
            print(f"\n  {'누적각':>8}  {'실제 J':>8}  {'토크':>8}  {'바닥대비':>8}")
            for i in range(1, n_steps + 1):
                d = [0.0] * 6
                d[joint - 1] = step_signed
                MP._movej(d, turn, relative=True)
                j = list(get_current_posj())
                t = list(get_external_torque())
                moved = j[joint - 1] - base_j[joint - 1]
                tq = t[joint - 1]
                rows.append((i * step_signed, moved, tq, tq - base_t[joint - 1]))
                log["steps"].append({"commanded_deg": i * step_signed,
                                     "moved_deg": moved, "torque": tq,
                                     "torque_all": t, "joints": j})
                print(f"  {i * step_signed:8.2f}  {moved:8.2f}  {tq:+8.2f}  "
                      f"{tq - base_t[joint - 1]:+8.2f}")

        print("\n  다 돌았다.")
        heard = input("  소리가 어떻게 됐나? (내려감 / 올라감 / 모르겠음) ").strip()
        log["heard"] = heard

        if not args.no_return:
            confirm("  돌린 만큼 되돌린다. 원래 자리로만 가니 줄이 더 당겨지지 않는다.")
            d = [0.0] * 6
            d[joint - 1] = -total
            MP._movej(d, turn, relative=True)
            print(f"  되돌렸다. J{joint} {get_current_posj()[joint - 1]:.2f}도")

        if not args.no_grip:
            confirm("  놓는다.")
            MP._release()
        confirm("  물러나 경유지로 돌아간다.")
        MP._movel(back, motion)
        MP.goto_route(route, motion)
        print("  끝났다.")
    except KeyboardInterrupt:
        print("\n중단했다. 그리퍼를 열고 멈춘다. 팔은 그 자리에 선다.")
        try:
            MP._release()
        except Exception as e:                          # noqa: B902
            print(f"  여는 데 실패: {e}")
    except Exception as e:                              # noqa: B902
        print(f"\n실패: {e}")
        try:
            MP._release()
        except Exception:                               # noqa: B902
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"\n{'=' * 70}")
    print(f"결과 — {args.peg}번, {args.direction}, "
          f"{'허공' if args.no_grip else args.preset}")
    print("=" * 70)
    if not rows:
        print("  회전 기록 없음")
        return 0

    if args.continuous:
        # 연속회전은 moved 가 0에서 total 까지 점점 커지는 게 정상이라
        # 이산 스텝용 "칸마다 0.1도 이상 움직였나" 검사는 안 맞는다.
        # 마지막 폴링(도착 확인을 이미 통과한 시점)만 목표와 견준다
        arrive_ok = abs(rows[-1][1] - total) <= points.motion.get(
            "string_break_arrive_tolerance_deg", 0.5)
        print(f"  관절은 명령대로 움직였다 (마지막 {rows[-1][1]:+.2f}도 / "
              f"명령 {total:+.2f}도)" if arrive_ok else
              f"  ⚠️ 마지막 폴링이 목표 각도 근처가 아니다 (마지막 {rows[-1][1]:+.2f}도 / 명령 {total:+.2f}도)")
    else:
        lag = sum(1 for _, moved, _, _ in rows if abs(moved) < 0.1)
        print(f"  관절은 명령대로 움직였다 (마지막 {rows[-1][1]:+.2f}도 / "
              f"명령 {rows[-1][0]:+.2f}도)" if lag == 0 else
              f"  ⚠️ 관절이 명령만큼 안 움직인 구간이 {lag}개 있다")

    # 정지 상태와 도는 중의 차이가 후보 신호다. 도는 순간 한 번 바뀌고
    # 그 뒤로 평평하면 그건 마찰이고, 각도에 따라 커지면 장력이다
    during = [r[2] for r in rows]
    mean = sum(during) / len(during)
    spread = max(during) - min(during)
    print(f"  회전 전(정지)  {base_t[joint - 1]:+.3f}")
    print(f"  회전 중 평균   {mean:+.3f}   흩어짐 {spread:.3f}")
    print(f"  정지→회전 단차 {mean - base_t[joint - 1]:+.3f}  <- 파지 확인의 후보 신호")

    # 기울기가 있나. 앞 1/3 과 뒤 1/3 의 평균을 흩어짐과 견준다
    k = max(2, len(during) // 3)
    head, tail = during[:k], during[-k:]
    slope = abs(sum(tail) / k) - abs(sum(head) / k)
    if spread > 0 and slope > max(0.5 * spread, 0.1):
        knee = next((r for r in rows
                     if abs(r[2]) > abs(sum(head) / k) + 0.5 * slope), None)
        print(f"  각도가 커질수록 저항이 늘었다 (뒤쪽이 {slope:+.3f} 더 크다)")
        if knee:
            unit = "초" if args.continuous else "도"
            print(f"  꺾이는 지점 약 {abs(knee[0]):.2f}{unit} — 헛돎(백래시) 대략치다")
    else:
        print(f"  **기울기 없음.** 앞뒤 차이 {slope:+.3f} 가 흩어짐 {spread:.3f} 안에 있다.")
        print("  각도가 늘어도 저항이 안 는다는 뜻이다. 두 가지로 읽힌다 —")
        print("    · 저항이 마찰뿐이고 줄 장력 몫은 이 센서로 안 보인다")
        print("    · 또는 헛돌아서 애초에 페그를 안 돌렸다")
        if not args.no_grip:
            print("  --no-grip 으로 허공 값을 재서 단차를 비교해야 갈린다.")

    if heard:
        print(f"\n  소리: {heard}")
        if heard.startswith("올라"):
            print("  ⚠️ 푸는 쪽으로 돌렸는데 음이 올라갔다면 turn_sign 이 반대다.")
            print("     teach_points.yaml 의 네 페그 turn_sign 을 전부 +1 로 뒤집어라.")
        elif heard.startswith("내려"):
            print("  turn_sign 이 맞다. 조이는 쪽도 시험해도 된다.")
    log["summary"] = {"mean": mean, "spread": spread,
                      "step_from_rest": mean - base_t[joint - 1],
                      "slope": slope}
    string_tag = "" if args.no_grip or args.string == "intact" else "_stringcut"
    cont_tag = "_continuous" if args.continuous else ""
    path = runlog.save(f"turn_peg{args.peg}_{args.direction}"
                       f"{'_air' if args.no_grip else ''}{string_tag}{cont_tag}", log, LOG_DIR)
    if path:
        print(f"\n  기록 저장: {os.path.relpath(path)}")

    def _conditions_match(b, check_preset=True):
        """조건이 다른 기록을 견주면 숫자가 무의미해진다 (#79 리뷰, 정묵).
        각도·스텝·속도·팔 모양·(문 상태끼리면) 파지 프리셋까지 맞아야 견준다."""
        if b.get("direction") != args.direction:
            return False
        if b.get("angle_deg") != args.angle or b.get("step_deg") != args.step:
            return False
        if b.get("sol") != sol:
            return False
        if abs((b.get("turn_vel") or 0) - turn["vel"]) > 0.01:
            return False
        if check_preset and b.get("preset") != (None if args.no_grip else args.preset):
            return False
        return True

    def _print_verdict(gap, noise, a, b, label_a, label_b, other_saved_at, what):
        # 이 식(step_from_rest 차이 vs 잡음)은 "가릴 수 있다/없다"를 못 정한다 —
        # 정지 baseline 이 조건마다 흔들리는 것까지 회전 저항으로 잡아낸다.
        # 9/19 파지·9/21 허공 기록을 넣으면 이 식은 "가릴 수 있다"로 통과하지만
        # 그건 이미 아니라고 결론 낸 사례다(CLAUDE.md 3.6, #79 리뷰). 그래서
        # 숫자만 보여주고 판단은 사람이 한다.
        print(f"  {label_a} 단차 {a:.3f}   {label_b} 단차 {b:.3f}   "
              f"({other_saved_at})")
        print(f"  차이 {gap:.3f}   흩어짐(잡음) {noise:.3f}   "
              f"(2배 문턱 {2 * noise:.3f} — 참고용, 단정 아님)")
        print(f"  **자동 판정 없음.** 이 식은 정지 baseline 흔들림을 {what} 신호로")
        print("  착각할 수 있다(실제 오판 사례 있음). 같은 세션에서 조건을 맞춰")
        print("  반복 측정한 뒤 사람이 판단할 것.")

    # 같은 페그·같은 조건의 반대 조건(파지↔허공) 기록을 찾아 견준다
    other = runlog.latest(LOG_DIR, lambda b: (
        b.get("tool") == "peg_turn_check" and b.get("peg") == args.peg
        and b.get("gripped") is args.no_grip and b.get("summary")
        and _conditions_match(b, check_preset=False)))[1]
    if other:
        mine = "허공" if args.no_grip else "파지"
        theirs = "파지" if args.no_grip else "허공"
        a = abs(log["summary"]["step_from_rest"])
        b = abs(other["summary"]["step_from_rest"])
        print(f"\n{'=' * 70}")
        print(f"비교 — {mine} vs {theirs}")
        print("=" * 70)
        gap = abs(a - b)
        noise = max(log["summary"]["spread"], other["summary"]["spread"])
        _print_verdict(gap, noise, a, b, mine, theirs,
                       other.get("saved_at"), "파지 저항")
    elif not args.no_grip:
        print("\n  같은 조건의 허공 값이 아직 없다. 같은 각도·속도·팔 모양으로 "
              "한 번 더 재면 자동으로 견줘 준다 —")
        print(f"    python3 tools/peg_turn_check.py --peg {args.peg} "
              f"--angle {args.angle:g} --step {args.step:g} --no-grip")

    # 줄 끊어짐 감지 (2026-09-22, 예외처리 B안) — 파지는 둘 다 한 상태에서
    # 줄만 있다/없다를 견준다. 센서 경로(get_external_torque, 조인트 6)는
    # 새 게 아니다 — 위 파지 비교와 같다. 새로운 건 비교 축뿐이다.
    if not args.no_grip:
        wanted_string = "cut" if args.string == "intact" else "intact"
        str_other = runlog.latest(LOG_DIR, lambda b: (
            b.get("tool") == "peg_turn_check" and b.get("peg") == args.peg
            and b.get("gripped") is True and b.get("string") == wanted_string
            and b.get("summary") and _conditions_match(b)))[1]
        if str_other:
            mine_s = "정상" if args.string == "intact" else "끊음"
            theirs_s = "끊음" if args.string == "intact" else "정상"
            a = abs(log["summary"]["step_from_rest"])
            b = abs(str_other["summary"]["step_from_rest"])
            print(f"\n{'=' * 70}")
            print(f"줄 끊어짐 감지 — 줄 {mine_s} vs {theirs_s} (조인트 6)")
            print("=" * 70)
            gap = abs(a - b)
            noise = max(log["summary"]["spread"], str_other["summary"]["spread"])
            _print_verdict(gap, noise, a, b, f"줄 {mine_s}", f"줄 {theirs_s}",
                           str_other.get("saved_at"), "줄 끊어짐")
        else:
            other_flag = "--string cut" if args.string == "intact" else "--string intact"
            print(f"\n  같은 조건의 줄 {'정상' if args.string == 'cut' else '끊음'} 값이 "
                  f"아직 없다. 같은 페그·방향·각도·속도·팔 모양으로 한 번 더 재면 "
                  f"자동으로 견줘 준다 —")
            print(f"    python3 tools/peg_turn_check.py --peg {args.peg} "
                  f"--direction {args.direction} --angle {args.angle:g} "
                  f"--step {args.step:g} {other_flag}")
    print()
    print("정밀 백래시는 마이크를 붙이고 tools/peg_cal.py --backlash --step 15")
    return 0


if __name__ == "__main__":
    sys.exit(main())
