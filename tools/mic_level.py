#!/usr/bin/env python3
"""
입력 장치별 레벨 비교 — 어느 마이크가 실제로 소리를 받는지 찾는다.

pitch_test.py에서 "소리 부족"이 뜰 때, 마이크 게인 문제인지
아예 연결이 안 된 것인지 가리는 용도다.

사용법:
    # 소리를 계속 재생해 둔 상태에서 실행
    python3 mic_level.py
    python3 mic_level.py --duration 3      # 장치당 녹음 길이
    python3 mic_level.py --device 4 --loop # 한 장치만 반복 측정(게인 조절용)

읽는 법:
    RMS -20 ~ -12 dBFS  → 적정
    RMS -40 이하        → 게인 부족
    RMS -50 이하        → 사실상 무음. 연결 자체를 의심할 것
    피크 주파수가 재생음과 일치해야 그 장치가 소리를 받고 있는 것
"""

import argparse
import sys

import numpy as np

try:
    import sounddevice as sd
except ImportError:
    print("sounddevice가 없습니다. pip install sounddevice")
    sys.exit(1)

SR = 44100


def rms_db(x):
    return 20 * np.log10(np.sqrt(np.mean(x ** 2)) + 1e-12)


def peak_freq(x, sr, fmin=50, fmax=2000):
    """최대 피크 주파수와, 그게 잡음 바닥보다 얼마나 솟았는지(dB)."""
    N = len(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(N)))
    freqs = np.fft.rfftfreq(N, 1 / sr)
    m = (freqs >= fmin) & (freqs <= fmax)
    if not m.any():
        return None, 0.0
    spec, freqs = spec[m], freqs[m]
    k = int(np.argmax(spec))
    prom = 20 * np.log10((spec[k] + 1e-12) / (np.median(spec) + 1e-12))
    return float(freqs[k]), float(prom)


def measure(device, duration, info):
    """
    한 장치에서 녹음 → (RMS, 피크Hz, 돌출dB, 클리핑비율, 실제SR). 실패 시 None.

    SOF 내장 코덱과 USB 오디오는 대개 48kHz 고정이라 44100으로는 안 열린다.
    44100 → 장치 기본값 → 48000 순으로 시도한다.
    """
    rates = []
    for r in (SR, int(info.get("default_samplerate") or 0), 48000):
        if r and r not in rates:
            rates.append(r)

    last = ""
    for sr in rates:
        try:
            x = sd.rec(int(duration * sr), samplerate=sr, channels=1,
                       dtype="float32", device=device)
            sd.wait()
            x = x[:, 0]
        except Exception as e:
            last = str(e)
            continue
        f, prom = peak_freq(x, sr)
        return (rms_db(x), f, prom, float(np.mean(np.abs(x) >= 0.98)), sr), None
    return None, last


def verdict(rms, prom):
    if rms < -50:
        return "무음 — 연결 의심"
    if prom < 6:
        return "신호 불명확"
    if rms < -40:
        return "게인 부족"
    if rms > -6:
        return "과다 — 낮출 것"
    if -20 <= rms <= -12:
        return "적정"
    return "사용 가능"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=2.0, help="장치당 녹음 길이(초)")
    ap.add_argument("--device", type=int, default=None, help="이 장치만 측정")
    ap.add_argument("--loop", action="store_true", help="반복 측정(게인 조절하며 보기)")
    args = ap.parse_args()

    devs = sd.query_devices()
    if args.device is not None:
        targets = [args.device]
    else:
        targets = [i for i, d in enumerate(devs) if d["max_input_channels"] > 0]

    if not targets:
        print("입력 장치가 없습니다.")
        return

    print(f"측정할 소리를 재생해 두세요. 장치당 {args.duration}초씩 녹음합니다.\n")

    while True:
        default_name = sd.query_devices(kind="input")["name"]
        for i in targets:
            name = devs[i]["name"]
            res, err = measure(i, args.duration, devs[i])
            mark = "*" if name == default_name else " "
            if res is None:
                print(f" {mark}[{i:2d}] {name:<40.40s}  열기 실패 ({err[:45]})")
                continue
            rms, f, prom, clip, sr = res
            fs = f"{f:7.1f}Hz(+{prom:4.1f}dB)" if f else "        -       "
            print(f" {mark}[{i:2d}] {name:<40.40s}  {sr/1000:4.1f}k  "
                  f"RMS {rms:6.1f} dBFS  피크 {fs}  "
                  f"클립 {clip*100:4.1f}%  {verdict(rms, prom)}")
        if not args.loop:
            break
        print("-" * 100)


if __name__ == "__main__":
    main()
