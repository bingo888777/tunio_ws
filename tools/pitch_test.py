#!/usr/bin/env python3
"""
기타 음정 검출 테스트 스크립트 (노트북 마이크용)

설치:
    pip install numpy sounddevice
    # Ubuntu에서 sounddevice가 안 되면: sudo apt install libportaudio2

사용 예:
    python pitch_test.py --list                 # 마이크 장치 목록
    python pitch_test.py --string 6             # 6번 줄(E2) 1회 측정
    python pitch_test.py --string 1 --repeat 10 # 1번 줄 10회 반복, 편차 확인
    python pitch_test.py --string 5 --device 2  # 장치 번호 지정
    python pitch_test.py --free                 # 목표 없이 아무 소리나 주파수만 측정

합격 기준: 같은 줄 10회 반복했을 때 표준편차 1센트 이내
"""

import argparse
import sys
import time

import numpy as np

try:
    import sounddevice as sd
except ImportError:
    # 분석 함수만 가져다 쓰는 경우(pitch_bench.py 등)에는 오디오 장치가 필요 없다.
    # 녹음이 실제로 필요한 시점(main)에서 다시 확인한다.
    sd = None


# ---------------------------------------------------------------
# 목표 주파수 (A4 = 440Hz, 12평균율)
# ---------------------------------------------------------------
STRINGS = {
    6: ("E2", 82.4069),
    5: ("A2", 110.0000),
    4: ("D3", 146.8324),
    3: ("G3", 195.9977),
    2: ("B3", 246.9417),
    1: ("E4", 329.6276),
}

SR = 44100  # 샘플링 레이트


def cents(f, f_target):
    return 1200.0 * np.log2(f / f_target)


# ---------------------------------------------------------------
# 전처리: 탐색 범위 밖 잡음 제거 (FFT 대역통과)
# ---------------------------------------------------------------
def bandpass(x, sr, lo, hi):
    """lo~hi Hz만 남긴다. 저주파 팬 소음·험을 잘라내는 용도."""
    N = len(x)
    X = np.fft.rfft(x)
    freqs = np.fft.rfftfreq(N, 1 / sr)
    X[(freqs < lo) | (freqs > hi)] = 0
    return np.fft.irfft(X, n=N).astype(np.float32)


# ---------------------------------------------------------------
# 보조 추정: FFT 최대 피크 + 포물선 보간
# ---------------------------------------------------------------
def fft_peak(x, sr, fmin, fmax):
    """탐색 범위 안 최대 피크를 보간해서 반환. 실패 시 None."""
    N = len(x)
    win = np.hanning(N)
    spec = np.abs(np.fft.rfft(x * win))
    freqs = np.fft.rfftfreq(N, 1 / sr)
    mask = (freqs >= fmin) & (freqs <= fmax)
    idx = np.where(mask)[0]
    if len(idx) < 3:
        return None
    k = idx[np.argmax(spec[idx])]
    if k <= 0 or k >= len(spec) - 1:
        return None
    a, b, c = np.log(spec[k - 1] + 1e-12), np.log(spec[k] + 1e-12), np.log(spec[k + 1] + 1e-12)
    denom = a - 2 * b + c
    delta = 0.5 * (a - c) / denom if abs(denom) > 1e-12 else 0.0
    f = float((k + delta) * sr / N)
    # 피크가 범위 내 중앙값보다 얼마나 두드러지는지 (dB)
    prom = 20 * np.log10((spec[k] + 1e-12) / (np.median(spec[idx]) + 1e-12))
    return f, float(prom)


# ---------------------------------------------------------------
# YIN 기본 주파수 추정 (de Cheveigné & Kawahara, 2002)
# ---------------------------------------------------------------
def yin_frame(frame, sr, fmin, fmax, threshold=0.20):
    """한 프레임에서 f0 추정. 실패하면 None."""
    x = frame.astype(np.float64)
    tau_max = int(sr / fmin) + 1
    tau_min = max(2, int(sr / fmax))
    W = len(x) - tau_max
    if W < tau_max:
        return None

    # 1) 차이 함수 d(tau)
    d = np.zeros(tau_max)
    x0 = x[:W]
    for tau in range(1, tau_max):
        diff = x0 - x[tau:tau + W]
        d[tau] = np.dot(diff, diff)

    # 2) 누적 평균 정규화 차이 함수 (CMNDF)
    cmndf = np.ones(tau_max)
    cumsum = np.cumsum(d[1:])
    cmndf[1:] = d[1:] * np.arange(1, tau_max) / np.maximum(cumsum, 1e-12)

    # 3) 절대 임계값: 탐색 범위 안에서 처음 threshold 아래로 내려가는 tau
    tau = None
    for t in range(tau_min, tau_max - 1):
        if cmndf[t] < threshold:
            # 국소 최소점까지 내려감
            while t + 1 < tau_max - 1 and cmndf[t + 1] < cmndf[t]:
                t += 1
            tau = t
            break
    if tau is None:
        # 임계값 못 넘으면 범위 안 최소점 사용 (신뢰도 낮음)
        seg = cmndf[tau_min:tau_max - 1]
        if len(seg) == 0:
            return None
        tau = tau_min + int(np.argmin(seg))
        if cmndf[tau] > 0.5:
            return None

    # 4) 포물선 보간으로 서브샘플 정밀도
    if 1 <= tau < tau_max - 1:
        a, b, c = cmndf[tau - 1], cmndf[tau], cmndf[tau + 1]
        denom = 2.0 * (a - 2.0 * b + c)
        if abs(denom) > 1e-12:
            tau = tau + (a - c) / denom

    return sr / tau


def estimate_pitch(signal, sr, fmin, fmax, frame_len=None, hop=None):
    """
    2단계 추정:
      1) 넓은 대역통과 → FFT 최대 피크로 대략 위치 (잡음에 강함)
      2) 그 피크 ±100센트만 남기고 YIN → 정밀값
    FFT 피크가 불명확하면 넓은 범위 YIN으로 후퇴.
    반환: (f0, 유효 프레임 수, 전체 프레임 수, 프레임별 f0, 방법)
    """
    wide = bandpass(signal, sr, fmin * 0.85, fmax * 4.2)
    res = fft_peak(wide, sr, fmin, fmax)
    f_fft, prom = (res if res else (None, 0.0))

    if f_fft is not None and prom >= 6.0:
        # 피크 주변 ±100센트로 좁혀서 YIN
        lo, hi = f_fft * 2 ** (-1 / 12), f_fft * 2 ** (1 / 12)
        narrow = bandpass(signal, sr, lo, hi)
        f_yin, ok, n, f0s = _yin_frames(narrow, sr, lo, hi, frame_len, hop)
        if f_yin is not None and ok >= 0.3 * n:
            return f_yin, ok, n, f0s, "yin"
        return f_fft, ok, n, f0s, "fft"

    # FFT 피크 불명확 → 넓은 범위 YIN
    f_yin, ok, n, f0s = _yin_frames(wide, sr, fmin, fmax, frame_len, hop)
    if f_yin is not None:
        return f_yin, ok, n, f0s, "yin-wide"
    if f_fft is not None:
        return f_fft, 0, n, np.array([]), "fft-weak"
    return None, 0, n, np.array([]), "none"


def _yin_frames(signal, sr, fmin, fmax, frame_len=None, hop=None):
    """여러 프레임에 YIN → 중앙값. 반환: (f0, 유효, 전체, 배열)"""
    if frame_len is None:
        frame_len = max(int(4.5 * sr / fmin), 2048)
    if hop is None:
        hop = frame_len // 2

    f0s, n = [], 0
    for start in range(0, len(signal) - frame_len + 1, hop):
        n += 1
        f = yin_frame(signal[start:start + frame_len], sr, fmin, fmax)
        if f is not None:
            f0s.append(f)
    if not f0s:
        return None, 0, n, np.array([])

    f0s = np.array(f0s)
    med = float(np.median(f0s))
    keep = np.abs(cents(f0s, med)) < 50
    if keep.sum() >= 2:
        med = float(np.median(f0s[keep]))
    return med, int(keep.sum()), n, f0s


# ---------------------------------------------------------------
# 녹음
# ---------------------------------------------------------------
def record(duration, device=None):
    audio = sd.rec(int(duration * SR), samplerate=SR, channels=1,
                   dtype="float32", device=device)
    sd.wait()
    return audio[:, 0]


def rms_db(x):
    r = np.sqrt(np.mean(x ** 2)) + 1e-12
    return 20 * np.log10(r)


def clip_ratio(x, thr=0.98):
    """클리핑 의심 샘플 비율"""
    return float(np.mean(np.abs(x) >= thr))


def top_peaks(x, sr, n=5, fmin=40, fmax=2000):
    """FFT 상위 피크 (진단용). 반환: [(Hz, dB), ...]"""
    N = len(x)
    win = np.hanning(N)
    spec = np.abs(np.fft.rfft(x * win))
    freqs = np.fft.rfftfreq(N, 1 / sr)
    mask = (freqs >= fmin) & (freqs <= fmax)
    spec, freqs = spec[mask], freqs[mask]
    db = 20 * np.log10(spec + 1e-12)
    db -= db.max()
    # 국소 최대만 골라서 상위 n개
    idx = [i for i in range(1, len(db) - 1) if db[i] > db[i - 1] and db[i] >= db[i + 1]]
    idx.sort(key=lambda i: -db[i])
    return [(float(freqs[i]), float(db[i])) for i in idx[:n]]


def save_wav(path, x, sr):
    import wave
    pcm = (np.clip(x, -1, 1) * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


# ---------------------------------------------------------------
# 메인
# ---------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="오디오 장치 목록 출력")
    ap.add_argument("--device", type=int, default=None, help="입력 장치 번호")
    ap.add_argument("--string", type=int, choices=[1, 2, 3, 4, 5, 6],
                    help="대상 줄 번호 (탐색 범위를 목표 ±2반음으로 제한)")
    ap.add_argument("--free", action="store_true",
                    help="목표 없이 60~500Hz 전체 범위에서 측정")
    ap.add_argument("--repeat", type=int, default=1, help="반복 횟수")
    ap.add_argument("--duration", type=float, default=2.0, help="녹음 길이(초)")
    ap.add_argument("--skip", type=float, default=0.3,
                    help="앞부분 제외 길이(초). 뜯는 순간의 타격음을 뺀다")
    ap.add_argument("--min-db", type=float, default=-50.0,
                    help="이 값보다 작으면 '소리 부족'으로 판정 (dBFS)")
    ap.add_argument("--debug", action="store_true",
                    help="스펙트럼 피크·클리핑 비율 출력")
    ap.add_argument("--save", type=str, default=None,
                    help="녹음을 WAV로 저장 (예: --save test → test_1.wav, test_2.wav ...)")
    args = ap.parse_args()

    if sd is None:
        print("sounddevice가 없습니다. pip install sounddevice")
        sys.exit(1)

    if args.list:
        print(sd.query_devices())
        return

    if not args.string and not args.free:
        ap.error("--string N 또는 --free 중 하나를 지정하세요")

    if args.free:
        name, f_target = "자유", None
        fmin, fmax = 60.0, 500.0
    else:
        name, f_target = STRINGS[args.string]
        fmin = f_target * 2 ** (-2 / 12)   # 목표 -2반음
        fmax = f_target * 2 ** (+2 / 12)   # 목표 +2반음

    dev_info = sd.query_devices(args.device, "input") if args.device is not None \
        else sd.query_devices(kind="input")
    print(f"입력 장치: {dev_info['name']}")
    if f_target:
        print(f"대상: {args.string}번 줄 {name} = {f_target:.4f} Hz  "
              f"(탐색 범위 {fmin:.1f} ~ {fmax:.1f} Hz)")
    else:
        print(f"자유 측정 (탐색 범위 {fmin:.0f} ~ {fmax:.0f} Hz)")
    print(f"녹음 {args.duration}초, 앞 {args.skip}초 제외, {args.repeat}회 반복\n")

    results = []
    for i in range(args.repeat):
        input(f"[{i + 1}/{args.repeat}] 엔터를 누른 뒤 줄을 뜯으세요 ")
        for c in (3, 2, 1):
            print(f"  {c}...", end="", flush=True)
            time.sleep(0.4)
        print(" 뜯으세요!")
        raw = record(args.duration, args.device)

        sig = raw[int(args.skip * SR):]
        level = rms_db(sig)

        if args.save:
            fn = f"{args.save}_{i + 1}.wav"
            save_wav(fn, raw, SR)
            print(f"  저장: {fn}")

        if args.debug:
            cr = clip_ratio(raw)
            peaks = top_peaks(sig, SR)
            pk = "  ".join(f"{f:.1f}Hz({d:+.0f}dB)" for f, d in peaks)
            print(f"  클리핑 {cr * 100:.2f}%   피크: {pk}")

        if level < args.min_db:
            print(f"  → 소리 부족 (RMS {level:.1f} dBFS < {args.min_db})\n")
            continue

        f0, ok, total, frames, method = estimate_pitch(sig, SR, fmin, fmax)
        if f0 is None:
            print(f"  → 주파수 판별 불가 (RMS {level:.1f} dBFS, 유효 프레임 0/{total})\n")
            continue

        line = (f"  → f0 = {f0:.3f} Hz [{method}]   유효 프레임 {ok}/{total}   "
                f"RMS {level:.1f} dBFS")
        if f_target:
            c = cents(f0, f_target)
            line += f"   오차 {c:+.2f} 센트 ({f0 - f_target:+.3f} Hz)"
            results.append(c)
        else:
            results.append(f0)
        print(line + "\n")

    # ------ 요약 ------
    if len(results) >= 2:
        arr = np.array(results)
        print("=" * 60)
        if f_target:
            print(f"측정 {len(arr)}회  평균 {arr.mean():+.2f} 센트  "
                  f"표준편차 {arr.std(ddof=1):.2f} 센트  "
                  f"범위 {arr.min():+.2f} ~ {arr.max():+.2f}")
            sd_ = arr.std(ddof=1)
            if sd_ <= 1.0:
                print("판정: 합격 — 표준편차 1센트 이내")
            elif sd_ <= 3.0:
                print("판정: 보통 — 쓸 수는 있으나 마이크 위치나 녹음 길이 조정 권장")
            else:
                print("판정: 불합격 — 마이크 교체 또는 환경 개선 필요")
        else:
            print(f"측정 {len(arr)}회  평균 {arr.mean():.3f} Hz  "
                  f"표준편차 {arr.std(ddof=1):.3f} Hz")
        print("=" * 60)


if __name__ == "__main__":
    main()
