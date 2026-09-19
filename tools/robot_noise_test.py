#!/usr/bin/env python3
"""
로봇 구동음이 측정을 방해하는지 실측한다.

기타를 건드리지 않은 채 로봇만 움직이며 녹음한다. 이 구간에는 발현이 하나도
없으므로, 여기서 onset 이 잡히고 pitch.analyze 가 통과하면 그것이 전부 오검출이다.

확인하는 것 두 가지:
  1) 헛 onset 이 몇 번 나는가 — 타임아웃 기준점이 로봇 소리에 먼저 걸리는지
  2) 헛 onset 중 몇 개가 유효성 관문을 통과하는가 — 이게 진짜 위험이다.
     통과하면 로봇 소리를 줄 소리로 착각해 엉뚱한 주파수를 보고한다.

방 소음만으로 잰 기준선은 180구간 중 0~1개(약 0.6%) 통과였다.
로봇 구동음에서 이 값이 크게 오르면 문턱값을 다시 잡아야 한다.

실행 (워크스페이스를 source 해야 한다):
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/robot_noise_test.py --sec 300 --save robot_noise.wav

    # 이미 녹음한 파일을 다시 분석만
    python3 tools/robot_noise_test.py --wav robot_noise.wav

측정 조건 (지키지 않으면 결과가 무의미하다):
  - 마이크는 실제 측정 때와 같은 자리(기타 울림통 구멍)에 같은 게인으로 둔다
  - OS 의 노이즈 제거·자동 음량(AGC)·에코 제거는 전부 끈다 (CLAUDE.md 3.5)
  - 녹음 중 줄을 건드리지 않는다. 말도 하지 않는다
  - 실제 조율 때 할 동작을 시킨다. 페그로 가는 이동, 그리퍼 여닫기, 발현 자세
    이동. 조그로 관절을 움직이는 것만으로도 된다
"""
import argparse
import sys
import time
import wave

import numpy as np

try:
    from tunio_audio import pitch
    from tunio_audio.recorder import find_onsets, frame_levels, read_wav
    from tunio_audio.recorder import Recorder, RecorderError
except ImportError:
    print("tunio_audio 를 찾을 수 없습니다. 워크스페이스를 source 하세요:")
    print("  source /opt/ros/jazzy/setup.bash && source install/setup.bash")
    sys.exit(1)

# audio_analyzer 와 같은 값을 쓴다. 여기서 바꾸면 실제 노드와 다른 것을 재게 된다.
TARGETS = {6: ("E2", 82.4069), 5: ("A2", 110.0000),
           2: ("B3", 246.9417), 1: ("E4", 329.6276)}
SKIP_HEAD_SEC = 0.3       # onset 직후 과도음 제외
RECORD_SEC_HIGH = 1.5
RECORD_SEC_LOW = 2.0
LOW_FREQ_HZ = 150.0


def window_sec(target_hz):
    return RECORD_SEC_LOW if target_hz < LOW_FREQ_HZ else RECORD_SEC_HIGH


def record(sec, device, chunk=5.0):
    """주어진 길이만큼 녹음한다. 진행 상황을 찍으려고 조각내어 읽는다."""
    rec = Recorder(device=device)
    rec.open()
    print(f"입력 장치: {rec.device_name()}  {rec.sample_rate} Hz")
    print(f"{sec:.0f}초 녹음합니다. 지금부터 로봇을 움직이세요.")
    print("줄은 건드리지 마세요.\n")
    rec.flush()
    parts, got, t0 = [], 0.0, time.monotonic()
    try:
        while got < sec:
            want = min(chunk, sec - got)
            parts.append(rec.read(want))
            got += want
            bar = int(40 * got / sec)
            print(f"\r  [{'#' * bar}{'.' * (40 - bar)}] {got:5.0f}/{sec:.0f}초",
                  end="", flush=True)
    except KeyboardInterrupt:
        print("\n  중단됨. 여기까지로 분석합니다.")
    finally:
        rec.close()
    print(f"\n  실제 소요 {time.monotonic() - t0:.1f}초, "
          f"버퍼 넘침 {rec.overflow_count}회\n")
    if rec.overflow_count:
        print("  ⚠ 버퍼가 넘쳤습니다. 일부 구간이 빠졌을 수 있습니다\n")
    return np.concatenate(parts), rec.sample_rate


def save_wav(path, x, sr):
    data = np.clip(x, -1.0, 1.0)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(sr))
        w.writeframes((data * 32767).astype("<i2").tobytes())


def level_report(x, sr):
    """구간 전체의 음량 분포. 실제 발현과 겹치는지 보는 기준."""
    lv = frame_levels(x, sr)
    lv = lv[np.isfinite(lv)]
    if not len(lv):
        return
    qs = np.percentile(lv, [50, 90, 99, 100])
    print("음량 분포 (10ms 프레임 RMS)")
    print(f"  중앙 {qs[0]:.1f} / 90% {qs[1]:.1f} / 99% {qs[2]:.1f} / "
          f"최대 {qs[3]:.1f} dBFS")
    print("  참고: 실제 발현은 -30 dBFS 근처, 조용한 방은 -60 dBFS 근처였다\n")


def analyze_onsets(x, sr, onsets):
    """헛 onset 각각을 4줄 목표로 분석해 관문 통과 여부를 센다."""
    passed = {s: [] for s in TARGETS}
    checked = {s: 0 for s in TARGETS}
    for t in onsets:
        st = int(t * sr)
        for s, (name, hz) in TARGETS.items():
            end = st + int(window_sec(hz) * sr)
            seg = x[st + int(SKIP_HEAD_SEC * sr):end]
            if len(seg) < int(0.7 * sr):
                continue                      # 끝자락이라 창이 모자람
            checked[s] += 1
            a = pitch.analyze(seg, sr, hz)
            if a.ok:
                passed[s].append((t, a))
    return checked, passed


def scan_windows(x, sr, step_sec=1.0):
    """발현 지점과 무관하게 일정 간격으로 잘라 본다. 기준선 비교용."""
    checked = {s: 0 for s in TARGETS}
    passed = {s: [] for s in TARGETS}
    for s, (name, hz) in TARGETS.items():
        w = int(window_sec(hz) * sr)
        for st in range(0, max(0, len(x) - w), int(step_sec * sr)):
            checked[s] += 1
            a = pitch.analyze(x[st:st + w], sr, hz)
            if a.ok:
                passed[s].append((st / sr, a))
    return checked, passed


def print_gate_report(title, checked, passed):
    print(title)
    worst = 0.0
    for s, (name, hz) in TARGETS.items():
        n, hits = checked[s], passed[s]
        if not n:
            print(f"  {s}번 {name}: 검사할 구간 없음")
            continue
        rate = 100.0 * len(hits) / n
        worst = max(worst, rate)
        mark = "  ← 확인 필요" if rate > 2.0 else ""
        print(f"  {s}번 {name}: {len(hits)}/{n} 통과 ({rate:.1f}%){mark}")
        for t, a in hits[:3]:
            err = 1200.0 * np.log2(a.hz / hz)
            print(f"      {t:7.2f}초  {a.hz:.2f} Hz ({err:+.0f}센트)  "
                  f"돌출 {a.prominence_db:.1f} dB  일치 {a.agreement_cents:.2f}센트")
        if len(hits) > 3:
            print(f"      … 외 {len(hits) - 3}개")
    print()
    return worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sec", type=float, default=300.0, help="녹음 길이(초)")
    ap.add_argument("--device", type=int, default=None, help="입력 장치 번호")
    ap.add_argument("--save", type=str, default="robot_noise.wav",
                    help="녹음을 저장할 경로 (빈 문자열이면 저장 안 함)")
    ap.add_argument("--wav", type=str, default=None,
                    help="녹음 대신 기존 WAV 를 분석한다")
    args = ap.parse_args()

    if args.wav:
        x, sr = read_wav(args.wav)
        print(f"{args.wav} 읽음 — {len(x) / sr:.1f}초, {sr} Hz\n")
    else:
        try:
            x, sr = record(args.sec, args.device)
        except RecorderError as exc:
            print(f"녹음 실패: {exc}")
            return 1
        if args.save:
            save_wav(args.save, x, sr)
            print(f"{args.save} 에 저장했습니다. "
                  f"나중에 --wav 로 다시 분석할 수 있습니다\n")

    dur = len(x) / sr
    print("=" * 64)
    level_report(x, sr)

    onsets = find_onsets(x, sr)
    print(f"헛 onset {len(onsets)}회  ({dur / 60:.1f}분 동안, "
          f"분당 {len(onsets) / (dur / 60):.1f}회)")
    print("  onset 은 느슨하게 잡도록 설계했으므로 여기서 몇 번 나는 것 자체는")
    print("  문제가 아니다. 아래 관문 통과율이 판정 기준이다.\n")

    w1 = 0.0
    if onsets:
        checked, passed = analyze_onsets(x, sr, onsets)
        w1 = print_gate_report("[A] 헛 onset 지점을 분석했을 때", checked, passed)

    checked2, passed2 = scan_windows(x, sr)
    w2 = print_gate_report("[B] 1초 간격으로 전 구간을 훑었을 때 (기준선)",
                           checked2, passed2)

    print("=" * 64)
    worst = max(w1, w2)
    if worst <= 2.0:
        print(f"판정: 통과. 최대 오검출률 {worst:.1f}% 로 방 소음 기준선"
              f"(0.6%) 과 같은 수준이다.")
        print("      로봇 구동음은 현재 문턱값으로 걸러진다.")
    else:
        print(f"판정: 확인 필요. 최대 오검출률 {worst:.1f}%.")
        print("      로봇 소리를 줄 소리로 착각할 수 있다. 위에 찍힌 통과 사례의")
        print("      돌출·일치 값을 보고 pitch.py 의 MIN_PROMINENCE_DB /")
        print("      MAX_AGREEMENT_CENTS 를 다시 잡아야 한다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
