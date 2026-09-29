#!/usr/bin/env python3
"""
pluck_strength — 발현 세기를 바꿔 가며 돌출과 음높이를 같이 잰다 (이슈 #32, #6).

## 왜 필요한가

#32 가 "4줄 전부 돌출 >= 20dB 를 만족하는 가장 약한 타현" 을 찾으라고 한다.
그런데 세기를 바꾸면 **돌출만 변하는 게 아니다.** 2026-09-21 실측에서
발현 세기와 읽히는 음높이가 r=0.913 으로 붙어 다녔다 (6번 E2, 기울기
0.516 센트/dB). 크게 튕기면 진동 중 장력이 올라가 샤프하게 울린다.

그래서 세기를 정하는 일은 **조율 기준점을 정하는 일이기도 하다.**
조율 루프는 "측정값 == 목표" 가 될 때까지 돌리므로, 로봇이 튕기는 그 세기에서
줄이 맞춰진다. 나중에 사람이 다른 세기로 확인하면 그 차이만큼 다르게 읽힌다.

이 도구는 **로봇 없이** 그 두 곡선을 같이 낸다. 로봇이 더하는 것은 구동음뿐이고,
"어느 세기에서 돌출이 얼마 나오는가" 는 줄 자체의 성질이라 지금 잴 수 있다.
로봇 세션에서는 "로봇이 이 세기를 낼 수 있는가" 와 "구동음이 얼마나 깎는가" 만
보면 된다.

## 쓰는 법

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/pluck_strength.py --strings 6 5 2 1
    python3 tools/pluck_strength.py --string 6 --repeat 8      # 한 줄만 촘촘히
    python3 tools/pluck_strength.py --strings 6 1 --levels 약 강

페그는 건드리지 않는다. 줄만 뜯는다. 각 단계에서 **그 세기를 유지**하는 게
전부다 — 화면에 RMS 가 찍히므로 보면서 맞추면 된다.

## 읽는 법

줄마다 두 가지가 나온다.

  센트/dB        조율 결과가 세기에 얼마나 묶이는지. 클수록 세기를 엄히 정해야 한다
  돌출 20dB 선   그 줄이 관문을 넘는 최소 RMS. #32 의 타현 세기 목표가 이것이다

네 줄의 "돌출 20dB 선" 중 **가장 높은 값**이 4줄 공통 하한이다. 로봇 구동음이
3~8dB 를 깎으므로(docs/로봇구동음_측정결과.md) 그만큼 여유를 더 둬야 한다.
"""
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_audio"))
try:
    from tunio_audio import pitch
    from tunio_audio.recorder import find_onsets, Recorder, RecorderError
except ImportError:
    print("tunio_audio 를 찾을 수 없습니다. 워크스페이스를 source 하세요:")
    print("  source /opt/ros/jazzy/setup.bash && source install/setup.bash")
    raise SystemExit(1)

# 목표 주파수 (A4=440, 12평균율). peg_cal.py 와 같은 표다.
TARGETS = {6: ("E2", 82.4069), 5: ("A2", 110.0000),
           2: ("B3", 246.9417), 1: ("E4", 329.6276)}

# 줄별 녹음 길이 (CLAUDE.md 4절). 저음현은 2.0초가 필요하다.
DURATION = {6: 2.0, 5: 2.0, 2: 1.5, 1: 1.5}

# 세기 단계. 이름과 안내 문구만 다르고 측정은 같다.
LEVELS = {
    "약": "겨우 소리 나게, 살살",
    "중": "보통으로 — 평소 연주하듯",
    "강": "세게, 다만 줄이 프렛을 때리지 않을 만큼",
}

PROMINENCE_TARGET = 20.0    # pitch.MIN_PROMINENCE_DB 와 같은 값 (#32 의 관문)


def cents(f, target):
    return 1200.0 * math.log2(f / target)


def measure_once(rec, target_hz, duration):
    """한 번 뜯게 하고 분석한다. 실패해도 Analysis 를 돌려준다(레벨은 들어 있다)."""
    rec.flush()
    buf = rec.read(duration + 0.6)
    sr = rec.sample_rate
    onsets = find_onsets(buf, sr)
    if not onsets:
        return None, "발현을 못 찾음"
    st = int(onsets[0] * sr)
    seg = buf[st + int(0.5 * sr):st + int(duration * sr)]
    if len(seg) < int(0.5 * sr):
        return None, "발현 뒤 구간이 짧음"
    return pitch.analyze(seg, sr, target_hz, semitones=2.0), None


def linfit(xs, ys):
    """최소제곱 기울기·절편·상관계수. 점이 2개 미만이면 None."""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0:
        return None
    slope = sxy / sxx
    syy = sum((y - my) ** 2 for y in ys)
    r = sxy / math.sqrt(sxx * syy) if syy > 0 else float("nan")
    return slope, my - slope * mx, r


def run_string(rec, no, levels, repeat):
    name, target = TARGETS[no]
    dur = DURATION[no]
    print(f"\n{'=' * 66}")
    print(f"{no}번 줄 {name}  목표 {target:.4f} Hz   녹음 {dur}초")
    print(f"{'=' * 66}")

    rows = []
    for lv in levels:
        print(f"\n  --- {lv} 세기 — {LEVELS[lv]} ---")
        print(f"      {repeat}번 뜯습니다. 같은 세기를 유지하세요.")
        got = 0
        for i in range(repeat):
            print(f"    [{i + 1}/{repeat}] 엔터 후 줄을 뜯으세요 ", end="", flush=True)
            input()
            a, err = measure_once(rec, target, dur)
            if a is None:
                print(f"       실패 — {err}")
                continue
            if not a.ok:
                # 실패해도 레벨·돌출은 쓸 수 있다. 관문을 못 넘은 것 자체가 자료다.
                print(f"       ✗ {a.summary()}")
                rows.append({"level": lv, "ok": False,
                             "rms_db": a.level_db, "prominence_db": a.prominence_db,
                             "cents": None, "reason": int(a.failure_reason)})
                got += 1
                continue
            c = cents(a.hz, target)
            print(f"       {a.hz:8.3f} Hz  {c:+6.2f}센트   "
                  f"RMS {a.level_db:+6.1f}   돌출 {a.prominence_db:5.1f} dB   "
                  f"일치 {a.agreement_cents:4.1f}")
            rows.append({"level": lv, "ok": True, "rms_db": a.level_db,
                         "prominence_db": a.prominence_db, "cents": c,
                         "hz": a.hz, "agreement_cents": a.agreement_cents})
            got += 1
        if got < 2:
            print("      이 단계는 표본이 모자랍니다")
    return rows


def report_string(no, rows):
    name, _ = TARGETS[no]
    print(f"\n  ── {no}번 {name} 정리 ──")
    if not rows:
        print("     자료 없음")
        return {}

    for lv in ("약", "중", "강"):
        sub = [r for r in rows if r["level"] == lv]
        if not sub:
            continue
        rms = [r["rms_db"] for r in sub]
        prom = [r["prominence_db"] for r in sub if not math.isnan(r["prominence_db"])]
        cs = [r["cents"] for r in sub if r["ok"]]
        passed = sum(1 for r in sub if r["ok"])
        line = (f"     {lv}  RMS {np.mean(rms):+6.1f}±{np.std(rms):4.1f}   ")
        line += (f"돌출 {np.mean(prom):5.1f}±{np.std(prom):4.1f}   "
                 if prom else "돌출   —        ")
        line += (f"센트 {np.mean(cs):+6.2f}±{np.std(cs):4.2f}   "
                 if cs else "센트    —         ")
        line += f"관문 통과 {passed}/{len(sub)}"
        print(line)

    ok = [r for r in rows if r["ok"]]
    out = {}

    fit = linfit([q["rms_db"] for q in ok], [q["cents"] for q in ok]) if len(ok) >= 2 else None
    if fit:
        slope, _, corr = fit
        span = max(q["rms_db"] for q in ok) - min(q["rms_db"] for q in ok)
        out["cents_per_db"] = round(slope, 3)
        out["cents_r"] = round(corr, 3)
        out["rms_span_db"] = round(span, 1)
        print(f"     세기 → 음높이   {slope:+.3f} 센트/dB   (r = {corr:+.3f})")
        print(f"                     가장 약한 것과 센 것이 {span:.1f}dB 차이고, "
              f"그것만으로 {slope * span:+.2f}센트가 생긴다")

    pr = [(q["rms_db"], q["prominence_db"]) for q in rows
          if not math.isnan(q["prominence_db"])]
    fit2 = linfit([a for a, _ in pr], [b for _, b in pr]) if len(pr) >= 2 else None
    if fit2:
        slope2, b2, r2 = fit2
        out["prom_per_db"] = round(slope2, 3)
        print(f"     세기 → 돌출     {slope2:+.3f} dB/dB    (r = {r2:+.3f})")
        if slope2 > 1e-6:
            need = (PROMINENCE_TARGET - b2) / slope2
            out["rms_for_20db"] = round(need, 1)
            print(f"     ★ 돌출 {PROMINENCE_TARGET:.0f}dB 선 = RMS {need:+.1f} dBFS "
                  f"— 이 줄이 관문을 넘는 최소 세기")
    return out


def main():
    ap = argparse.ArgumentParser(
        description="발현 세기를 바꿔 가며 돌출과 음높이를 같이 잰다 (#32)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--string", type=int, choices=[1, 2, 5, 6])
    g.add_argument("--strings", type=int, nargs="+", choices=[1, 2, 5, 6])
    ap.add_argument("--levels", nargs="+", choices=list(LEVELS), default=["약", "중", "강"])
    ap.add_argument("--repeat", type=int, default=5, help="단계마다 몇 번 뜯을지 (기본 5)")
    ap.add_argument("--device", type=int, default=None, help="입력 장치 번호 (기본 default)")
    ap.add_argument("--save", type=str, default=None, help="결과 JSON 경로")
    a = ap.parse_args()

    strings = a.strings or ([a.string] if a.string else [6, 5, 2, 1])

    print(f"입력 장치: {'default' if a.device is None else a.device}")
    print(f"대상: {' '.join(f'{n}번' for n in strings)}   "
          f"세기 {' / '.join(a.levels)}   단계마다 {a.repeat}번")
    print("페그는 건드리지 마세요. 줄만 뜯습니다.")

    try:
        rec = Recorder(device=a.device)
    except RecorderError as exc:
        print(f"녹음 장치를 못 엽니다: {exc}")
        return 1

    out = {"tool": "pluck_strength", "strings": strings, "levels": a.levels,
           "repeat": a.repeat, "prominence_target": PROMINENCE_TARGET,
           "results": {}}
    with rec:
        for no in strings:
            rows = run_string(rec, no, a.levels, a.repeat)
            out["results"][str(no)] = {"rows": rows, "summary": report_string(no, rows)}

    print(f"\n{'=' * 66}")
    print("4줄 공통 하한")
    print(f"{'=' * 66}")
    need = [(n, out["results"][str(n)]["summary"].get("rms_for_20db"))
            for n in strings if out["results"][str(n)]["summary"].get("rms_for_20db")]
    if need:
        worst = max(need, key=lambda t: t[1])
        for n, v in need:
            mark = "  ← 가장 높다" if n == worst[0] else ""
            print(f"  {n}번 {TARGETS[n][0]}   돌출 20dB 선  RMS {v:+.1f} dBFS{mark}")
        print(f"\n  4줄을 다 넘기려면 최소 RMS {worst[1]:+.1f} dBFS 로 튕겨야 한다.")
        print(f"  로봇 구동음이 돌출을 3~8dB 깎으므로(docs/로봇구동음_측정결과.md)")
        print(f"  로봇 세션에서는 그만큼 더 세게 잡아야 한다.")
    else:
        print("  돌출-세기 관계를 못 냈습니다. 표본을 늘리세요.")

    slopes = [(n, out["results"][str(n)]["summary"].get("cents_per_db"))
              for n in strings if out["results"][str(n)]["summary"].get("cents_per_db")]
    if slopes:
        print(f"\n  세기 → 음높이 기울기 (조율 결과가 세기에 묶이는 정도)")
        for n, v in slopes:
            print(f"    {n}번 {TARGETS[n][0]}   {v:+.3f} 센트/dB")
        print(f"\n  허용 10센트 기준, 이 기울기가 0.5 면 세기를 20dB 안에서 유지해야")
        print(f"  오차가 허용을 다 먹지 않는다.")

    if a.save:
        os.makedirs(os.path.dirname(os.path.abspath(a.save)), exist_ok=True)
        out["saved_at"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(a.save, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        print(f"\n저장: {a.save}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
