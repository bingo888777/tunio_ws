#!/usr/bin/env python3
"""
페그 각도-센트 캘리브레이션.

"손잡이를 몇 도 돌리면 음이 몇 센트 변하는가"를 구한다.
이 값이 있어야 task_manager 가 측정 오차로부터 AdjustPeg 의 angle_deg 를 계산한다.
현재 tuning_logic.peg_angle() 의 k 는 임시값이다.

정동휘가 쓴 수동 측정 가이드(PEG_ANGLE_MEASUREMENT.md, 저장소에는 없고 본인 로컬에 있음)의
절차를 그대로 따르되, 주파수 측정·센트 환산·회귀·편차 점검을 자동으로 한다.
사람은 페그를 돌리고 실제로 몇 도 돌렸는지만 입력하면 된다.

실행 (워크스페이스를 source 해야 한다):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/peg_cal.py --string 6
    python3 tools/peg_cal.py --string 1 --step 45      # 고음현은 작게
    python3 tools/peg_cal.py --string 6 --backlash     # 백래시 양
    python3 tools/peg_cal.py --string 6 --min-angle    # 최소 유효 각도
    python3 tools/peg_cal.py --string 6 --simulate 0.2 # 기타 없이 흐름만 확인

측정 원칙 (가이드 2절, 지키지 않으면 값이 튄다):
  - 항상 조이는 방향으로만 돌린다. 방향을 바꾸면 백래시로 헛돈다
  - 목표 음 +-1반음 안에서 잰다. 느슨하면 같은 각도라도 덜 변한다
  - 손을 뗀 뒤 2~3초 기다렸다가 측정한다
  - 줄을 갈고 하루 이상 지난 상태에서 잰다. 새 줄은 계속 늘어난다
"""
import argparse
from datetime import datetime
import json
import math
import sys
import time

import numpy as np

try:
    from tunio_audio import pitch
    from tunio_audio.recorder import find_onsets, Recorder, RecorderError
except ImportError:
    print("tunio_audio 를 찾을 수 없습니다. 워크스페이스를 source 하세요:")
    print("  source /opt/ros/jazzy/setup.bash && source install/setup.bash")
    sys.exit(1)

# 목표 주파수 (A4=440, 12평균율). 노드는 서버에서 받지만 이 도구는 단독 실행이라 들고 있다.
TARGETS = {6: ("E2", 82.4069), 5: ("A2", 110.0000),
           2: ("B3", 246.9417), 1: ("E4", 329.6276)}

SAFETY_CENTS = 200.0    # 목표보다 이만큼 높아지면 멈춘다 (줄 보호)
SETTLE_SEC = 3.0        # 손 떼고 기다리는 시간 (가이드 2절)


def cents(f_new, f_old):
    return 1200.0 * math.log2(f_new / f_old)


# ---------------------------------------------------------------
# 측정
# ---------------------------------------------------------------
def measure_once(rec, center_hz, duration, sim=None):
    """
    한 번 튕겨서 주파수를 잰다. 실패하면 None.

    탐색 중심은 목표가 아니라 '직전 측정값'이다. 조여 올리는 동안 음이 계속 올라가므로
    고정 목표 +-2반음으로 두면 범위를 벗어난다.
    """
    if sim is not None:
        sr = 44100
        t = np.arange(int(duration * sr)) / sr
        x = (np.sin(2 * np.pi * sim * t) + 0.5 * np.sin(4 * np.pi * sim * t)
             + 0.3 * np.sin(6 * np.pi * sim * t))
        x = (x * np.exp(-t / 4.0) * 0.3).astype(np.float32)
        x += np.random.default_rng().normal(0, 0.003, len(t)).astype(np.float32)
        buf = np.concatenate([np.zeros(int(0.4 * sr), np.float32), x])
    else:
        sr = rec.sample_rate
        rec.flush()
        buf = rec.read(duration + 0.6)

    onsets = find_onsets(buf, sr)
    if not onsets:
        return None, "발현을 못 찾음"
    st = int(onsets[0] * sr)
    seg = buf[st + int(0.3 * sr):st + int(duration * sr)]
    if len(seg) < int(0.5 * sr):
        return None, "발현 뒤 구간이 짧음"

    # 직전 값 +-2반음. 한 번에 크게 돌렸으면 못 잡으므로 넓혀서 재시도한다.
    for semis in (2.0, 5.0):
        a = pitch.analyze(seg, sr, center_hz, semitones=semis)
        if a.ok:
            if semis > 2.0:
                print(f"    (탐색 범위를 ±{semis:.0f}반음으로 넓혀 잡음 — 한 번에 크게 돌렸습니다)")
            return a.hz, a
    return None, a.summary()


def measure_stable(rec, center_hz, duration, tries=4, sim=None):
    """여러 번 재서 중앙값. 2회 이상 성공해야 값을 준다."""
    vals, last = [], ""
    for i in range(tries):
        if i:
            time.sleep(0.4)
        print(f"    [{i + 1}/{tries}] 엔터 후 줄을 뜯으세요 ", end="", flush=True)
        if sim is None:
            input()
        else:
            print("(모의)")
        hz, info = measure_once(rec, center_hz, duration, sim)
        if hz is None:
            last = info
            print(f"       실패 — {info}")
            continue
        vals.append(hz)
        print(f"       {hz:.3f} Hz")
        if len(vals) >= 2 and abs(cents(max(vals), min(vals))) < 3.0:
            break
    if len(vals) < 2:
        print(f"    측정 실패 ({last})")
        return None
    med = float(np.median(vals))
    spread = abs(cents(max(vals), min(vals)))
    if spread > 5.0:
        print(f"    ⚠ 측정이 {spread:.1f}센트 흔들립니다. 마이크나 발현을 확인하세요")
    return med


def ask_angle(default):
    """실제로 몇 도 돌렸는지. 손으로는 정확히 안 되므로 입력받는다."""
    while True:
        s = input(f"    조이는 방향으로 돌린 뒤 실제 각도 입력 [{default:.0f}도, 엔터=기본]: ").strip()
        if not s:
            return float(default)
        try:
            v = float(s)
            if v <= 0:
                print("    양수를 입력하세요 (항상 조이는 방향)")
                continue
            return v
        except ValueError:
            print("    숫자를 입력하세요")


# ---------------------------------------------------------------
# 모드
# ---------------------------------------------------------------
def save(out, path):
    if not path:
        return
    with open(path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)


def run_sweep(rec, args, name, target, out=None):
    """목표보다 낮게 시작해 조이며 단계별로 잰다. 회차 반복 (가이드 3절)."""
    rounds = []
    for rnd in range(1, args.rounds + 1):
        print(f"\n{'=' * 64}")
        print(f"회차 {rnd}/{args.rounds}  —  {args.string}번 줄 {name} (목표 {target:.4f} Hz)")
        print(f"{'=' * 64}")
        print(f"  목표보다 약 {args.start_cents:.0f}센트 낮게 맞추세요 "
              f"(약 {target * 2 ** (args.start_cents / 1200):.2f} Hz)")
        if rnd > 1:
            print("  ※ 시작 위치보다 아래로 충분히 풀었다가 조여 올려서 맞추세요 (백래시)")
        if args.sim is None:
            input("  준비되면 엔터 ")

        sim_hz = target * 2 ** (args.start_cents / 1200) if args.sim is not None else None
        print(f"  [시작값 측정] {SETTLE_SEC:.0f}초 기다린 뒤 재세요")
        f0 = measure_stable(rec, target, args.duration, sim=sim_hz)
        if f0 is None:
            print("  시작값을 못 재서 이 회차를 건너뜁니다")
            continue

        pts = [(0.0, f0)]
        total = 0.0
        for step in range(1, args.steps + 1):
            print(f"\n  [{step}/{args.steps}] 누적 {total:.0f}도 → "
                  f"{total + args.step:.0f}도")
            deg = ask_angle(args.step) if args.sim is None else args.step
            total += deg
            if args.sim is not None:
                sim_hz = f0 * 2 ** (total * args.sim / 1200)
            print(f"  {SETTLE_SEC:.0f}초 기다린 뒤 측정합니다")
            if args.sim is None:
                time.sleep(SETTLE_SEC)
            f = measure_stable(rec, pts[-1][1], args.duration, sim=sim_hz)
            if f is None:
                print("  이 단계를 못 재서 회차를 중단합니다")
                break
            c_total = cents(f, f0)
            c_step = cents(f, pts[-1][1])
            pts.append((total, f))
            print(f"    → {f:.3f} Hz   이번 구간 {c_step:+.1f}센트 "
                  f"({c_step / deg:+.3f} 센트/도)   누적 {c_total:+.1f}센트")
            over = cents(f, target)
            if over > SAFETY_CENTS:
                print(f"  ⚠ 목표보다 {over:.0f}센트 높습니다. 줄 보호를 위해 중단합니다")
                break

        # 구간 1개(측정점 2개)도 유효한 데이터다. --steps 1 로 큰 각도를
        # 한 번만 돌리는 방식은 중간 지점을 눈대중할 필요가 없어 오히려 정확하다.
        if len(pts) >= 2:
            rounds.append(pts)
            report_round(pts)
            # 회차마다 저장한다. 중간에 끊어도 여기까지는 남는다
            if out is not None:
                out["점"] = [[[round(d, 1), round(h, 4)] for d, h in r] for r in rounds]
                save(out, args.save)
                if args.save:
                    print(f"  (여기까지 {args.save} 에 저장됨)")
        else:
            print("  측정점이 2개 미만이라 이 회차는 버립니다")
    return rounds


def report_round(pts):
    """구간별 센트/도가 서로 비슷한지 본다 (가이드 5절)."""
    print("\n  구간별:")
    per = []
    for (a_deg, a_hz), (b_deg, b_hz) in zip(pts, pts[1:]):
        d = b_deg - a_deg
        c = cents(b_hz, a_hz)
        per.append(c / d)
        print(f"    {a_deg:5.0f}→{b_deg:5.0f}도  {c:+7.2f}센트  {c / d:+.4f} 센트/도")
    arr = np.array(per)
    m = arr.mean()
    if len(arr) < 2:
        print(f"    {m:+.4f} 센트/도 (구간이 1개라 회차 안에서는 편차를 못 본다. "
              f"회차끼리 비교해 판단한다)")
        return
    dev = (arr.max() - arr.min()) / abs(m) * 100 if m else float("inf")
    print(f"    평균 {m:+.4f} 센트/도, 구간 편차 {dev:.0f}%", end="")
    print("  ⚠ 20% 초과 — 이 회차는 버리는 게 좋습니다" if dev > 20 else "  (양호)")


def all_segments(rounds):
    """모든 회차의 구간을 (중간 주파수, 센트/도)로 펼친다."""
    out = []
    for pts in rounds:
        for (d1, f1), (d2, f2) in zip(pts, pts[1:]):
            if d2 <= d1:
                continue
            out.append(((f1 + f2) / 2.0, cents(f2, f1) / (d2 - d1)))
    return out


def value_at_target(segs, target):
    """
    목표 주파수에서의 센트/도.

    센트/도는 상수가 아니다. 장력이 높을수록 작아진다.
    f ∝ sqrt(T) 이고 같은 각도가 주는 장력 증가는 거의 일정하므로,
    이론상 센트/도 ∝ 1/f^2 이다. 실측에서도 log-log 기울기가 -1.7 근처로 나온다.
    따라서 전 구간 평균을 쓰면 안 되고 목표 지점의 값을 뽑아야 한다.
    실제 조율은 목표 ±30센트 안에서만 일어나므로 그 지점에서는 상수로 써도 된다.

    반환: (값, 방법). 구간이 부족하거나 주파수 범위가 좁으면 단순 평균으로 후퇴한다.
    """
    f = np.array([x[0] for x in segs], dtype=float)
    v = np.array([x[1] for x in segs], dtype=float)
    span = 1200.0 * np.log2(f.max() / f.min()) if len(f) > 1 else 0.0
    if len(segs) >= 4 and span > 80 and v.min() > 0:
        slope, icept = np.polyfit(np.log(f), np.log(v), 1)
        return float(np.exp(icept + slope * np.log(target))), f"회귀(기울기 {slope:.2f})"
    near = np.abs(1200.0 * np.log2(f / target)) < 80
    if near.sum() >= 2:
        return float(v[near].mean()), f"목표 ±80센트 구간 평균 (n={int(near.sum())})"
    return float(v.mean()), "전 구간 평균 (주파수 범위가 좁음)"


def run_backlash(rec, args, name, target):
    """푸는 방향으로 돌릴 때 음이 떨어지기 시작하는 누적 각도 (가이드 7절)."""
    print(f"\n{'=' * 64}")
    print(f"백래시 측정 — {args.string}번 줄 {name}")
    print(f"{'=' * 64}")
    print("  먼저 조이는 방향으로 목표 근처에 맞춰두세요")
    if args.sim is None:
        input("  준비되면 엔터 ")
    base = measure_stable(rec, target, args.duration,
                          sim=target if args.sim is not None else None)
    if base is None:
        return None
    print(f"  기준 {base:.3f} Hz. 이제 **푸는 방향**으로 {args.step:.0f}도씩 돌립니다")
    total, seen = 0.0, []
    for i in range(1, 13):
        deg = ask_angle(args.step) if args.sim is None else args.step
        total += deg
        sim_hz = None
        if args.sim is not None:
            slack = max(0.0, total - 40.0)      # 모의: 백래시 40도
            sim_hz = base * 2 ** (-slack * args.sim / 1200)
        if args.sim is None:
            time.sleep(SETTLE_SEC)
        f = measure_stable(rec, base, args.duration, sim=sim_hz)
        if f is None:
            continue
        c = cents(f, base)
        seen.append((total, c))
        print(f"    누적 {total:.0f}도 → {f:.3f} Hz  {c:+.1f}센트")
        if c < -5.0:
            # -5센트에 닿은 각도가 아니라 '떨어지기 시작한' 각도가 백래시다.
            # 마지막 두 점의 기울기로 0센트 지점까지 역산한다.
            start = total
            if len(seen) >= 2:
                (d1, c1), (d2, c2) = seen[-2], seen[-1]
                slope = (c2 - c1) / (d2 - d1)
                if slope < -1e-6:
                    start = max(0.0, d2 - c2 / slope)
            print(f"\n  {total:.0f}도에서 {c:+.1f}센트 — 기울기로 역산한 백래시 "
                  f"= 약 {start:.0f}도")
            print(f"  → 되돌릴 때는 여유를 둬 {start * 2:.0f}~{start * 2.5:.0f}도쯤 "
                  f"풀었다가 조여 올리면 됩니다")
            return round(start, 1)
    print("  12단계 안에 변화를 못 봤습니다. 단위 각도를 키워 다시 재세요")
    return None


# 실측한 줄별 민감도 (목표 ±30센트 구간, docs/페그_실측표.md).
# 최소 유효 각도의 사다리를 이 값으로 짠다. --cpd 로 덮어쓸 수 있다.
MEASURED_CPD = {6: 2.219, 5: 0.865, 2: 1.065, 1: 0.502}

# 사다리의 각 단이 만들어야 할 음정 변화(센트). 각도가 아니라 센트로 잡는다.
MIN_ANGLE_RUNGS_CENTS = (2.0, 3.0, 5.0, 8.0, 12.0)

# 판정 두 가지를 다 만족해야 '그 각도는 먹힌다'고 본다.
#   누적비  — N번 이어 돌린 총 변화가 기대의 이 비율 이상인가
#   고른비  — 각 회가 기대의 절반~1.5배 안에 드는 비율
# 누적만 보면 안 된다. 줄이 너트에 걸렸다 튕기면 0/0/큰값이 나오는데
# 누적은 기대치를 채운다. 그 각도로 한 번 명령하면 결과가 0 이거나 3배라
# 조율 루프가 진동한다. 실제로 쓸 수 있으려면 매번 고르게 움직여야 한다.
MIN_ANGLE_RATIO = 0.7
MIN_ANGLE_STEADY = 0.6


def run_min_angle(rec, args, name, target):
    """
    유격에 묻히지 않고 실제로 음이 변하기 시작하는 최소 각도 (가이드 7절).

    예전에는 10/15/20/30/45도 고정 사다리에 "3센트 이상 변하면 통과"였다.
    두 가지가 틀렸다.

    1) 사다리가 각도 고정이라 줄마다 묻는 질문이 달라진다. 6번 줄은 1도에
       2.2센트라 10도면 22센트다. 무조건 첫 단에서 통과하고 끝나므로
       "4.5도 이하냐"는 정작 알아야 할 것에 답을 못 한다.
       (4.5도는 min_effective_deg x cents_per_deg <= 허용 10센트 조건에서 나온다)
    2) 3센트 단일 판정은 측정 잡음과 구분이 안 된다. measure_stable 이
       중앙값을 내도 1~3센트는 흔들린다.

    그래서 사다리를 센트 기준으로 짜고(줄마다 각도가 달라진다), 판정도
    "기대한 만큼 움직였는가"로 바꿨다. 같은 각도를 여러 번 이어 돌려
    누적 변화를 보므로 신호는 쌓이고 잡음은 안 쌓인다.
    """
    cpd = args.cpd if args.cpd else MEASURED_CPD.get(args.string)
    if not cpd:
        print("  이 줄의 cents_per_deg 를 모릅니다. --cpd 로 주세요")
        return None

    print(f"\n{'=' * 64}")
    print(f"최소 유효 각도 — {args.string}번 줄 {name}")
    print(f"{'=' * 64}")
    print(f"  기준 민감도 {cpd:.3f} 센트/도. 이 값으로 사다리를 짭니다")
    print(f"  수렴 조건상 이 줄은 {10.0 / cpd:.1f}도 이하여야 허용 10센트에 들어갑니다")
    print("  목표 근처에서, 조이는 방향으로만 돌립니다")
    print(f"  각 단을 {args.repeat}번 이어 돌려 누적 변화를 봅니다\n")

    found, rows = None, []
    for want_cents in MIN_ANGLE_RUNGS_CENTS:
        deg = round(want_cents / cpd, 1)
        print(f"\n  --- {deg}도 단계 (한 번에 약 {want_cents:.0f}센트 기대) ---")
        if deg < 3.0:
            print("      ※ 손으로는 이 각도를 정확히 못 돌립니다. 실제로 돌린 만큼을")
            print("        입력하면 그 각도 기준으로 판정합니다")
        if args.sim is None:
            input("      준비되면 엔터 ")
        base = measure_stable(rec, target, args.duration,
                              sim=target if args.sim is not None else None)
        if base is None:
            continue

        prev, total_deg, ok_count, steps = base, 0.0, 0, []
        for i in range(args.repeat):
            if args.sim is None:
                print(f"      [{i + 1}/{args.repeat}] {deg}도 조이세요")
                actual = ask_angle(deg)
                time.sleep(SETTLE_SEC)
            else:
                actual = deg
            total_deg += actual
            # 모의: 스틱-슬립. 8도가 쌓여야 한 번에 풀린다.
            # 작은 각도는 누적으로는 따라오지만 회마다 들쭉날쭉해진다.
            if args.sim is not None:
                slipped = (total_deg // 8.0) * 8.0
                sim_hz = base * 2 ** (slipped * args.sim / 1200)
            else:
                sim_hz = None
            f = measure_stable(rec, prev, args.duration, sim=sim_hz)
            if f is None:
                continue
            step_c = cents(f, prev)
            want = actual * cpd
            steps.append(step_c / want if want else 0.0)
            print(f"        {actual:.1f}도 → {step_c:+.2f}센트 "
                  f"(기대 {want:.2f}, {100 * steps[-1]:.0f}%)"
                  f"  누적 {total_deg:.1f}도 / {cents(f, base):+.2f}센트")
            prev = f
            ok_count += 1

        if ok_count < 2 or total_deg <= 0:
            print("      측정이 모자라 이 단계는 건너뜁니다")
            continue
        got = cents(prev, base)
        eff = got / total_deg
        ratio = eff / cpd
        steady = sum(1 for r in steps if 0.5 <= r <= 1.5) / len(steps)
        ok = ratio >= MIN_ANGLE_RATIO and steady >= MIN_ANGLE_STEADY
        rows.append((deg, total_deg, eff, ratio, steady))
        print(f"      누적 {total_deg:.1f}도에 {got:+.2f}센트 = {eff:.3f} 센트/도")
        print(f"      누적비 {ratio * 100:.0f}%  고른비 {steady * 100:.0f}%"
              f"  →  {'먹힘' if ok else '못 씀'}")
        if ratio >= MIN_ANGLE_RATIO and steady < MIN_ANGLE_STEADY:
            print("      (누적으로는 움직이는데 회마다 들쭉날쭉합니다. 줄이 너트에")
            print("       걸렸다 튕기는 것으로 보입니다. 이 각도로 한 번 명령하면")
            print("       결과를 예측할 수 없습니다)")
        if ok and found is None:
            found = deg
            print(f"      ★ 최소 유효 각도 = 약 {deg}도")
            break

    print(f"\n{'=' * 64}")
    if found is None:
        print("  사다리 안에서 '먹히는' 각도를 못 찾았습니다.")
        print(f"  유격이 {MIN_ANGLE_RUNGS_CENTS[-1] / cpd:.1f}도보다 큽니다. "
              f"--cpd 나 사다리를 키워 다시 재세요")
    else:
        limit = 10.0 / cpd
        print(f"  최소 유효 각도 {found}도  = {found * cpd:.1f}센트")
        if found > limit:
            print(f"  ⚠ 수렴 한계 {limit:.1f}도를 넘습니다. 이 줄은 허용 10센트로")
            print("    수렴할 수 없습니다. 허용 센트를 늘리거나 기구를 손봐야 합니다")
        else:
            print(f"  수렴 한계 {limit:.1f}도 안에 들어옵니다 (허용 10센트 기준)")
        print(f"\n  peg_calibration.yaml 에 넣을 값:  min_effective_deg: {found}")
    for deg, td, eff, ratio, steady in rows:
        print(f"    {deg:5.1f}도 x {td / deg:.0f}회 → {eff:.3f} 센트/도 "
              f"(누적비 {ratio * 100:3.0f}%  고른비 {steady * 100:3.0f}%)")
    return found


# ---------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="페그 각도-센트 캘리브레이션",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--string", type=int, choices=[1, 2, 5, 6], required=True)
    ap.add_argument("--device", type=int, default=None, help="입력 장치 번호")
    ap.add_argument("--step", type=float, default=90.0, help="단위 회전 각도(도)")
    ap.add_argument("--steps", type=int, default=4, help="회차당 단계 수")
    ap.add_argument("--rounds", type=int, default=3, help="반복 회차")
    ap.add_argument("--start-cents", type=float, default=-50.0,
                    help="시작 위치 (목표 대비 센트)")
    ap.add_argument("--duration", type=float, default=2.0, help="녹음 길이(초)")
    ap.add_argument("--backlash", action="store_true", help="백래시 양 측정")
    ap.add_argument("--min-angle", action="store_true", help="최소 유효 각도 측정")
    ap.add_argument("--cpd", type=float, default=None,
                    help="센트/도 (--min-angle 사다리 기준. 기본은 실측표 값)")
    ap.add_argument("--repeat", type=int, default=3,
                    help="--min-angle 에서 같은 각도를 이어 돌리는 횟수")
    ap.add_argument("--simulate", dest="sim", type=float, default=None,
                    metavar="CENTS_PER_DEG",
                    help="기타 없이 흐름만 확인 (예: 0.2). 이 값이 다시 나오면 정상")
    ap.add_argument("--save", type=str, default=None, help="결과 JSON 경로")
    args = ap.parse_args()

    name, target = TARGETS[args.string]
    rec = None
    if args.sim is None:
        rec = Recorder(device=args.device)
        try:
            sr = rec.open()
        except RecorderError as exc:
            print(f"입력 장치를 열 수 없습니다: {exc}")
            sys.exit(1)
        print(f"입력 장치: {rec.device_name()}  {sr} Hz")
    else:
        print(f"[모의 모드] 실제 값 {args.sim} 센트/도 로 흐름만 확인합니다")

    print(f"대상: {args.string}번 줄 {name} = {target:.4f} Hz")
    print("주의: 항상 조이는 방향으로만 돌리세요. 방향을 바꾸면 백래시로 값이 튑니다.")

    out = {"측정일": datetime.now().isoformat(timespec="seconds"),
           "줄": args.string, "음": name, "목표_hz": target,
           "단위각도": args.step}

    if args.backlash:
        out["백래시_도"] = run_backlash(rec, args, name, target)
    elif args.min_angle:
        out["최소유효각도_도"] = run_min_angle(rec, args, name, target)
    else:
        rounds = run_sweep(rec, args, name, target, out)
        if not rounds:
            print("\n쓸 수 있는 회차가 없습니다.")
            sys.exit(1)

        segs = all_segments(rounds)
        v = np.array([x[1] for x in segs])
        at_target, how = value_at_target(segs, target)
        scatter = (v.max() - v.min()) / v.mean() * 100

        out["구간"] = [[round(f, 3), round(c, 4)] for f, c in segs]
        out["센트per도_목표지점"] = round(at_target, 4)
        out["산출방법"] = how
        out["구간_최소"] = round(float(v.min()), 4)
        out["구간_최대"] = round(float(v.max()), 4)
        out["구간_흩어짐_퍼센트"] = round(float(scatter), 1)
        out["k_deg_per_cent"] = round(1.0 / at_target, 4)

        print(f"\n{'=' * 64}")
        print(f"최종 — {args.string}번 줄 {name}")
        print(f"{'=' * 64}")
        print(f"  구간 {len(segs)}개  범위 {v.min():.3f} ~ {v.max():.3f} 센트/도"
              f"  (흩어짐 {scatter:.0f}%)")
        print()
        print("  센트/도는 상수가 아닙니다. 장력이 높을수록 작아집니다.")
        print(f"  목표 {target:.2f} Hz 지점의 값을 씁니다 — {how}")
        print()
        print(f"  ★ {at_target:.3f} 센트/도   →   k = {1.0 / at_target:.3f} deg/cent")
        print(f"     예) 15센트 틀어지면 {15.0 * (1.0 / at_target):.0f}도 돌린다")
        if scatter > 40:
            print()
            print("  ⚠ 구간 흩어짐이 큽니다. 장력 의존성 외에 다른 원인이 섞였을 수 있습니다:")
            print("     방향을 섞어 돌림 / 대기 시간 부족 / 굵은 줄이 너트에 걸렸다 풀림")
            print("     통합 첫 시도용으로는 쓸 수 있지만 로봇으로 재측정할 것")
        print()
        print("  정동휘에게 넘길 형식:")
        print(f"    {args.string}번 {name}   {at_target:.3f} 센트/도 (목표 지점)  "
              f"k = {1.0 / at_target:.3f}")

    save(out, args.save)
    if args.save:
        print(f"\n저장: {args.save}")


if __name__ == "__main__":
    main()
