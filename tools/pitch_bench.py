#!/usr/bin/env python3
"""
주파수 추정 알고리즘 비교 벤치마크

"왜 FFT 유도 + 협대역 YIN을 택했는가"에 수치로 답하기 위한 도구다.
같은 신호를 여러 알고리즘에 돌려 정확도·재현성·강건성·처리시간을 비교한다.

비교 대상:
    fft     FFT 최대 피크 + 포물선 보간      (베이스라인, 의존성 없음)
    acf     자기상관                          (YIN의 전신, 의존성 없음)
    hps     Harmonic Product Spectrum        (기본파 약할 때 유리, 의존성 없음)
    yin     FFT 유도 + 협대역 YIN            (현재 채택안, pitch_test.py)
    pyin    librosa.pyin                     (YIN + HMM, librosa 필요)
    crepe   CREPE CNN                        (딥러닝, crepe + TensorFlow 필요)

설치 (선택 항목은 ROS 환경을 건드리지 않도록 별도 venv 권장):
    python3 -m venv ~/pitch_bench_venv
    source ~/pitch_bench_venv/bin/activate
    pip install numpy librosa crepe

사용 예:
    python3 pitch_bench.py --synth                    # 합성 신호 (참값 확실)
    python3 pitch_bench.py --synth --string 6         # 저음현 조건
    python3 pitch_bench.py --wav "t3_e4_*.wav" --string 1
    python3 pitch_bench.py --robust --string 6        # 기본파 감쇠·잡음 스윕
    python3 pitch_bench.py --octave --string 6        # 탐색 범위 해제 시 옥타브 오인율
    python3 pitch_bench.py --all --string 6           # 위 전부

읽는 법은 docs/주파수분석_테스트가이드.md 를 볼 것.
"""

import argparse
import glob
import sys
import time
import wave

import numpy as np

# 현재 채택안은 pitch_test.py에서 그대로 가져온다 (같은 디렉토리)
try:
    from pitch_test import estimate_pitch as _tunio_estimate
    from pitch_test import STRINGS
except ImportError:
    print("pitch_test.py를 찾을 수 없습니다. tools/ 디렉토리에서 실행하세요.")
    sys.exit(1)

SR = 44100


def cents(f, ref):
    return 1200.0 * np.log2(f / ref)


# ===============================================================
# 알고리즘들 — 전부 (signal, sr, fmin, fmax) → f0 또는 None
# ===============================================================

def algo_fft(x, sr, fmin, fmax):
    """FFT 최대 피크 + 로그 포물선 보간. 가장 단순한 베이스라인."""
    N = len(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(N)))
    freqs = np.fft.rfftfreq(N, 1 / sr)
    idx = np.where((freqs >= fmin) & (freqs <= fmax))[0]
    if len(idx) < 3:
        return None
    k = idx[np.argmax(spec[idx])]
    if k <= 0 or k >= len(spec) - 1:
        return None
    a, b, c = (np.log(spec[k + d] + 1e-12) for d in (-1, 0, 1))
    den = a - 2 * b + c
    dk = 0.5 * (a - c) / den if abs(den) > 1e-12 else 0.0
    return float((k + dk) * sr / N)


def algo_acf(x, sr, fmin, fmax):
    """
    자기상관(ACF). YIN 이전의 고전 방식.

    FFT로 자기상관을 구하고 탐색 범위 안 최대점을 찾는다.
    정규화를 하지 않아 짧은 지연(높은 주파수)이 유리해지는 편향이 있고,
    이것이 옥타브 오인의 주된 원인이다 — YIN의 CMNDF가 고치려 한 지점.
    """
    x = np.asarray(x, dtype=np.float64)
    x = x - x.mean()
    N = len(x)
    nfft = 1 << int(np.ceil(np.log2(2 * N)))
    X = np.fft.rfft(x, nfft)
    r = np.fft.irfft(X * np.conj(X), nfft)[:N]

    tau_min = max(1, int(sr / fmax))
    tau_max = min(N - 2, int(sr / fmin) + 1)
    if tau_max - tau_min < 3:
        return None
    seg = r[tau_min:tau_max]
    tau = tau_min + int(np.argmax(seg))
    if tau <= 0 or tau >= N - 1:
        return None
    a, b, c = r[tau - 1], r[tau], r[tau + 1]
    den = 2.0 * (a - 2.0 * b + c)
    if abs(den) > 1e-12:
        tau = tau + (a - c) / den
    return float(sr / tau) if tau > 0 else None


def algo_hps(x, sr, fmin, fmax, n_harm=5):
    """
    Harmonic Product Spectrum.

    스펙트럼을 2·3·…배로 다운샘플해 곱하면 배음이 기본파 위치에 겹쳐 쌓인다.
    기본파가 약해도 배음만 있으면 찾아내므로 저음현 조건에 이론상 유리하다.
    곱셈은 로그 합으로 계산해 수치 문제를 피한다.
    """
    N = len(x)
    spec = np.abs(np.fft.rfft(x * np.hanning(N)))
    freqs = np.fft.rfftfreq(N, 1 / sr)
    logspec = np.log(spec + 1e-12)

    acc = logspec.copy()
    for h in range(2, n_harm + 1):
        dec = logspec[::h]
        acc[:len(dec)] += dec

    idx = np.where((freqs >= fmin) & (freqs <= fmax))[0]
    idx = idx[idx < len(acc)]
    if len(idx) < 3:
        return None
    k = idx[np.argmax(acc[idx])]
    if k <= 0 or k >= len(acc) - 1:
        return None
    # 보간은 원 스펙트럼에서 (HPS 곡선은 뾰족해 보간 편향이 큼)
    a, b, c = (np.log(spec[k + d] + 1e-12) for d in (-1, 0, 1))
    den = a - 2 * b + c
    dk = 0.5 * (a - c) / den if abs(den) > 1e-12 else 0.0
    return float((k + dk) * sr / N)


def algo_yin(x, sr, fmin, fmax):
    """현재 채택안 — FFT 유도 + 협대역 YIN (pitch_test.estimate_pitch)."""
    f0, _ok, _n, _frames, _method = _tunio_estimate(x, sr, fmin, fmax)
    return f0


def algo_pyin(x, sr, fmin, fmax, resolution=0.1):
    """
    librosa.pyin — YIN + HMM 궤적 추적. 유효 프레임의 중앙값을 쓴다.

    탐색 범위를 목표 ±2반음으로 좁게 잡으면 기본 설정으로는 터진다.
    pyin은 주파수를 격자로 나눠 HMM을 돌리는데, 4반음 범위에서 격자가 41칸뿐인 반면
    기본 전이 폭(max_transition_rate=35.92 기준)은 101칸을 요구하기 때문이다.
    우리 신호는 2초짜리 정지음이라 음정이 크게 움직이지 않으므로,
    전이율을 격자 크기에 맞춰 낮춰 잡는다.
    """
    import librosa

    # 저음에서도 최소 4주기가 들어가도록 프레임 길이를 잡는다
    frame_length = int(2 ** np.ceil(np.log2(max(4.0 * sr / fmin, 2048))))
    hop_length = frame_length // 4

    # 전이 폭이 격자를 넘지 않도록 max_transition_rate 상한을 역산한다
    #   격자 수      n_bins = floor(12 * nbps * log2(fmax/fmin)) + 1
    #   전이 폭      width  = round(mtr * 12 * hop/sr) * nbps + 1  <=  n_bins
    nbps = int(np.ceil(1.0 / resolution))
    n_bins = int(np.floor(12 * nbps * np.log2(fmax / fmin))) + 1
    frame_period = hop_length / sr
    max_semitones = max(1, (n_bins - 1) // nbps)
    mtr = min(max_semitones / (12 * frame_period), 35.92) * 0.9

    f0, _voiced, _prob = librosa.pyin(
        np.asarray(x, dtype=np.float32), fmin=fmin, fmax=fmax, sr=sr,
        frame_length=frame_length, hop_length=hop_length,
        resolution=resolution, max_transition_rate=mtr)
    f0 = f0[~np.isnan(f0)]
    return float(np.median(f0)) if len(f0) else None


def algo_crepe(x, sr, fmin, fmax):
    """
    CREPE — CNN 기반 추정기.

    주의: 논문의 성능 지표는 50센트 이내 정확도(RPA)이고 출력이 20센트 간격
    빈의 가중평균이라, 애초에 1센트급 정밀도를 겨냥한 모델이 아니다.
    내부적으로 16kHz로 리샘플링해 동작한다.
    """
    import crepe
    _t, freq, conf, _act = crepe.predict(
        np.asarray(x, dtype=np.float32), sr, viterbi=True,
        model_capacity="full", verbose=0)
    m = (freq >= fmin) & (freq <= fmax) & (conf > 0.5)
    if not m.any():
        m = (freq >= fmin) & (freq <= fmax)
    return float(np.median(freq[m])) if m.any() else None


ALGOS = [
    ("fft", algo_fft, None),
    ("acf", algo_acf, None),
    ("hps", algo_hps, None),
    ("yin", algo_yin, None),
    ("pyin", algo_pyin, "librosa"),
    ("crepe", algo_crepe, "crepe"),
]


def available_algos(only=None):
    """설치된 것만 추린다. (이름, 함수) 목록과 건너뛴 사유를 반환."""
    out, skipped = [], []
    for name, fn, dep in ALGOS:
        if only and name not in only:
            continue
        if dep:
            try:
                __import__(dep)
            except Exception:
                skipped.append((name, dep))
                continue
        out.append((name, fn))
    return out, skipped


def run(fn, x, sr, fmin, fmax):
    """한 알고리즘 실행 → (f0, 소요초). 예외는 None으로 삼킨다."""
    t0 = time.perf_counter()
    try:
        f0 = fn(x, sr, fmin, fmax)
    except Exception:
        f0 = None
    return f0, time.perf_counter() - t0


# ===============================================================
# 합성 신호 — 참값을 아는 유일한 조건
# ===============================================================

# 실측 배음 프로필 (dB, 1배음 기준). 둘 다 실제 기타 녹음 10개 평균이다.
#   1번 E4 — g1_e4_*.wav
#   6번 E2 — g6_e2_*.wav. 기본파보다 2배음이 6.5dB, 3배음이 10.3dB 강하다
#
# 주의: 1번 줄 프로필은 튕기는 방식에 따라 크게 달라진다. 같은 줄을 같은 사람이
# 쳤는데도 t3_e4 에서는 고차 배음이 거의 없었고(4f=-29.0, 5f=-23.6dB)
# g1_e4 에서는 5배음이 기본파보다 강했다(+0.7dB). 브릿지에 가까이 칠수록
# 고차 배음이 강조되기 때문으로, 로봇 타현 위치가 정해지면 다시 재야 한다.
HARMONICS = {
    1: [0.0, -6.6, -3.4, -1.1, 0.7, -5.7],
    6: [0.0, 6.5, 10.3, -0.9, 3.2, -4.5],
}

# 실측 비조화성 계수. f_k = k*f0*sqrt(1 + B*k^2)
# 감긴 줄인 6번은 3.33e-4 로 통상값(1e-5~1e-4)보다 크고,
# 민 줄인 1번은 사실상 0이다(피팅값 -1.0e-5, 음수는 비물리적이므로 측정 하한).
# 6번처럼 B가 크면 배음을 차수로 나눈 f0 역산이 실제보다 높게 나온다 —
# 참값 추정에 쓰지 말 것. 1번에서는 그 편차가 1센트 이내였다.
INHARM = {1: 1.0e-5, 6: 3.33e-4}


def synth(f0, sr=SR, dur=1.7, profile=None, decay=3.0,
          snr_db=None, inharm=0.0, fund_atten_db=0.0, seed=0):
    """
    기타 음을 모사한 합성 신호.

    profile         배음별 진폭(dB, 1배음 기준)
    decay           지수 감쇠 시상수(초)
    snr_db          백색잡음을 섞어 이 SNR로 맞춤. None이면 잡음 없음
    inharm          비조화성 계수. f_k = k*f0*sqrt(1 + B*k^2)
    fund_atten_db   기본파만 추가로 깎는다 (저음현 조건 모사)
    """
    rng = np.random.default_rng(seed)
    profile = profile or HARMONICS[1]
    t = np.arange(int(dur * sr)) / sr
    x = np.zeros_like(t)

    for k, db in enumerate(profile, start=1):
        fk = f0 * k * np.sqrt(1.0 + inharm * k * k)
        if fk >= sr / 2:
            break
        amp = 10 ** ((db - (fund_atten_db if k == 1 else 0.0)) / 20.0)
        x += amp * np.sin(2 * np.pi * fk * t + rng.uniform(0, 2 * np.pi))

    x *= np.exp(-t / decay)

    if snr_db is not None:
        sig_p = np.mean(x ** 2)
        noise = rng.normal(0, 1, len(x))
        noise *= np.sqrt(sig_p / (10 ** (snr_db / 10.0)) / np.mean(noise ** 2))
        x = x + noise

    peak = np.max(np.abs(x))
    return (x / peak * 0.5).astype(np.float64)


def load_wav(path, skip=0.3):
    with wave.open(path, "rb") as w:
        sr = w.getframerate()
        pcm = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16)
    x = pcm.astype(np.float64) / 32768.0
    return x[int(skip * sr):], sr


# ===============================================================
# 리포트
# ===============================================================

def header(title):
    print("\n" + "=" * 78)
    print(title)
    print("=" * 78)


def report_single(algos, x, sr, fmin, fmax, truth, label):
    """한 신호에 대한 알고리즘별 결과."""
    print(f"\n{label}")
    print(f"  {'알고리즘':<8s} {'추정 f0':>12s} {'오차(센트)':>12s} {'소요(ms)':>10s}")
    rows = []
    for name, fn in algos:
        f0, dt = run(fn, x, sr, fmin, fmax)
        if f0 is None:
            print(f"  {name:<8s} {'실패':>12s} {'-':>12s} {dt*1000:>9.1f}")
            rows.append((name, None, None, dt))
            continue
        err = cents(f0, truth) if truth else None
        es = f"{err:+12.2f}" if err is not None else f"{'-':>12s}"
        print(f"  {name:<8s} {f0:>12.4f} {es} {dt*1000:>9.1f}")
        rows.append((name, f0, err, dt))
    return rows


def report_repeat(algos, signals, sr, fmin, fmax, truth, label):
    """
    여러 신호(반복 측정)에 대한 평균·표준편차.
    종합표에서 다시 쓸 수 있도록 알고리즘별 결과를 반환한다.
    """
    print(f"\n{label}")
    print(f"  {'알고리즘':<8s} {'평균 f0':>11s} {'평균오차':>10s} "
          f"{'표준편차':>10s} {'실패':>6s} {'평균(ms)':>10s}")
    out = {}
    for name, fn in algos:
        vals, times, fail = [], [], 0
        for x in signals:
            f0, dt = run(fn, x, sr, fmin, fmax)
            times.append(dt)
            if f0 is None:
                fail += 1
            else:
                vals.append(f0)
        if not vals:
            print(f"  {name:<8s} {'전부 실패':>11s}")
            out[name] = None
            continue
        a = np.array(vals)
        sd_c = np.std(cents(a, a.mean()), ddof=1) if len(a) > 1 else 0.0
        err_v = cents(a.mean(), truth) if truth else None
        err = f"{err_v:+10.2f}" if err_v is not None else f"{'-':>10s}"
        print(f"  {name:<8s} {a.mean():>11.4f} {err} {sd_c:>9.2f}c "
              f"{fail:>4d}/{len(signals):<2d} {np.mean(times)*1000:>9.1f}")
        out[name] = {"mean": float(a.mean()), "err": err_v, "sd": float(sd_c),
                     "fail": fail, "n": len(signals),
                     "ms": float(np.mean(times) * 1000)}
    return out


def report_summary(algos, synth_res, wav_res, wav_label):
    """
    합성 신호와 실제 녹음을 나란히 놓는다.

    둘을 같이 봐야 하는 이유: 합성은 참값을 아는 대신 현실(비조화성·맥놀이·
    시간 변화)을 덜 담고, 실제 녹음은 현실을 담는 대신 참값을 모른다.
    한쪽만 보면 결론이 뒤집힌다 — 실제로 합성에서 가장 약했던 알고리즘이
    실제 저음 녹음에서는 가장 정확했다.
    """
    header("종합 — 합성 신호 vs 실제 녹음")
    print(f"  실제 녹음: {wav_label}")
    print()
    print(f"  {'알고리즘':<8s} │ {'합성 오차':>10s} {'합성 편차':>10s} │ "
          f"{'실제 오차':>10s} {'실제 편차':>10s} {'실패':>6s} │ {'시간(ms)':>9s}")
    print("  " + "─" * 8 + "─┼─" + "─" * 21 + "─┼─" + "─" * 28 + "─┼─" + "─" * 9)

    for name, _fn in algos:
        s = synth_res.get(name) if synth_res else None
        w = wav_res.get(name) if wav_res else None

        def fmt(d, key, unit="c", width=10):
            if not d or d.get(key) is None:
                return f"{'-':>{width}s}"
            v = d[key]
            return f"{v:+9.2f}{unit}" if key == "err" else f"{v:9.2f}{unit}"

        fails = f"{w['fail']}/{w['n']}" if w else "-"
        ms = f"{w['ms']:.0f}" if w else (f"{s['ms']:.0f}" if s else "-")
        print(f"  {name:<8s} │ {fmt(s,'err')} {fmt(s,'sd')} │ "
              f"{fmt(w,'err')} {fmt(w,'sd')} {fails:>6s} │ {ms:>9s}")

    print()
    print("  합성 오차는 참값 대비 절대 정확도, 편차는 재현성이다.")
    if wav_res and all(v is None or v.get("err") is None
                       for v in wav_res.values()):
        print("  실제 녹음은 참값을 주지 않아 오차 열이 비어 있다 (--truth 로 지정).")
    print("  두 결과가 갈리면 실제 녹음을 따른다. 합성은 현실을 덜 담는다.")


def report_robust(algos, f0_true, sr, fmin, fmax, profile, snr_floor=30.0):
    """
    기본파 감쇠와 잡음을 스윕하며 어느 지점에서 무너지는지 본다.

    감쇠 스윕에는 반드시 잡음을 함께 넣는다. 잡음이 없으면 탐색 범위 안에
    기본파밖에 없어서 아무리 깎아도 여전히 최대 피크이고, 감쇠가 무의미해진다.
    실제로는 기본파가 약해지면 잡음에 묻히므로 그 조건을 같이 재야 한다.
    """
    header(f"강건성 — 기본파 감쇠 스윕 (SNR {snr_floor:.0f}dB 고정, 오차 센트, X=실패)")
    print(f"  {'알고리즘':<8s}" + "".join(f"{d:>10s}"
          for d in ("0dB", "-10dB", "-20dB", "-30dB", "-40dB")))
    for name, fn in algos:
        cells = []
        for att in (0, 10, 20, 30, 40):
            x = synth(f0_true, sr, profile=profile,
                      fund_atten_db=att, snr_db=snr_floor)
            f0, _ = run(fn, x, sr, fmin, fmax)
            cells.append(f"{cents(f0, f0_true):>10.2f}" if f0 else f"{'X':>10s}")
        print(f"  {name:<8s}" + "".join(cells))

    header("강건성 — 잡음 SNR 스윕 (오차 센트, X=실패)")
    print(f"  {'알고리즘':<8s}" + "".join(f"{d:>10s}"
          for d in ("40dB", "30dB", "20dB", "10dB", "0dB")))
    for name, fn in algos:
        cells = []
        for snr in (40, 30, 20, 10, 0):
            x = synth(f0_true, sr, profile=profile, snr_db=snr)
            f0, _ = run(fn, x, sr, fmin, fmax)
            cells.append(f"{cents(f0, f0_true):>10.2f}" if f0 else f"{'X':>10s}")
        print(f"  {name:<8s}" + "".join(cells))


def report_octave(algos, f0_true, sr, profile, trials=12):
    """
    탐색 범위를 목표 ±2반음이 아니라 60~500Hz로 풀었을 때의 옥타브 오인율.
    CLAUDE.md 3.3의 범위 제한이 왜 필요한지 보이는 항목이다.
    """
    header(f"옥타브 오인 — 탐색 범위 60~500Hz로 해제, 조건 {trials}종")
    print(f"  {'알고리즘':<8s} {'정답':>6s} {'2배':>6s} {'1/2배':>6s} "
          f"{'3배':>6s} {'기타오류':>8s} {'실패':>6s}")
    for name, fn in algos:
        ok = oct_hi = oct_lo = tri = other = fail = 0
        for i in range(trials):
            x = synth(f0_true, sr, profile=profile, seed=i,
                      fund_atten_db=(i % 4) * 10, snr_db=40 - (i % 3) * 15)
            f0, _ = run(fn, x, sr, 60.0, 500.0)
            if f0 is None:
                fail += 1
                continue
            r = f0 / f0_true
            if abs(cents(f0, f0_true)) < 50:
                ok += 1
            elif abs(cents(f0, f0_true * 2)) < 50:
                oct_hi += 1
            elif abs(cents(f0, f0_true / 2)) < 50:
                oct_lo += 1
            elif abs(cents(f0, f0_true * 3)) < 50:
                tri += 1
            else:
                other += 1
        print(f"  {name:<8s} {ok:>6d} {oct_hi:>6d} {oct_lo:>6d} "
              f"{tri:>6d} {other:>8d} {fail:>6d}")


# ===============================================================
# 메인
# ===============================================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--string", type=int, choices=[1, 2, 3, 4, 5, 6], default=1,
                    help="대상 줄 (탐색 범위와 합성 배음 프로필을 정한다)")
    ap.add_argument("--synth", action="store_true", help="합성 신호 비교")
    ap.add_argument("--wav", type=str, default=None,
                    help="WAV 글롭 패턴 (예: 't3_e4_*.wav')")
    ap.add_argument("--robust", action="store_true", help="기본파 감쇠·잡음 스윕")
    ap.add_argument("--octave", action="store_true", help="옥타브 오인율")
    ap.add_argument("--all", action="store_true", help="전부 실행")
    ap.add_argument("--algos", type=str, default=None,
                    help="쉼표로 구분해 일부만 (예: fft,acf,hps,yin)")
    ap.add_argument("--truth", type=float, default=None,
                    help="WAV의 참값 f0. 없으면 재현성만 본다")
    ap.add_argument("--skip", type=float, default=0.3, help="WAV 앞부분 제외(초)")
    ap.add_argument("--detune", type=float, default=-37.3,
                    help="합성 신호의 f0를 목표에서 이만큼(센트) 벗어나게 한다. "
                         "0으로 두면 pyin·crepe처럼 주파수를 격자로 양자화하는 "
                         "알고리즘에 유리한 편향이 생긴다 (목표가 격자에 정확히 "
                         "떨어지기 때문). 실제 기타는 항상 목표에서 벗어나 있다")
    args = ap.parse_args()

    if not any((args.synth, args.wav, args.robust, args.octave, args.all)):
        ap.error("--synth / --wav / --robust / --octave / --all 중 하나는 지정하세요")

    only = set(args.algos.split(",")) if args.algos else None
    algos, skipped = available_algos(only)
    if not algos:
        print("실행할 알고리즘이 없습니다.")
        return

    name, f_target = STRINGS[args.string]
    fmin = f_target * 2 ** (-2 / 12)
    fmax = f_target * 2 ** (+2 / 12)
    profile = HARMONICS.get(args.string, HARMONICS[1])
    inharm = INHARM.get(args.string, 1.0e-4)

    # 탐색 범위는 목표 기준으로 잡되(실제 운용과 동일), 합성 신호의 실제 f0는
    # 목표에서 벗어나게 둔다. 격자 기반 알고리즘에 공짜 점수를 주지 않기 위해서다.
    f_true = f_target * 2 ** (args.detune / 1200.0)

    synth_res, wav_res, wav_label = None, None, None

    print(f"대상: {args.string}번 줄 {name} = {f_target:.4f} Hz")
    print(f"탐색 범위: {fmin:.2f} ~ {fmax:.2f} Hz (목표 ±2반음)")
    print(f"합성 신호 실제 f0: {f_true:.4f} Hz (목표 {args.detune:+.1f}센트)")
    print(f"비교 알고리즘: {', '.join(n for n, _ in algos)}")
    if skipped:
        for n, dep in skipped:
            print(f"  건너뜀: {n} (모듈 '{dep}' 없음 — pip install {dep})")

    if args.synth or args.all:
        header("합성 신호 — 참값이 정확히 알려진 조건")
        x = synth(f_true, SR, profile=profile)
        report_single(algos, x, SR, fmin, fmax, f_true, "이상적 조건 (잡음 없음)")

        x = synth(f_true, SR, profile=profile, inharm=inharm)
        report_single(algos, x, SR, fmin, fmax, f_true,
                      f"비조화성 포함 (B={inharm:.2e}, 해당 줄 실측값)")

        sigs = [synth(f_true, SR, profile=profile, snr_db=30, seed=i)
                for i in range(10)]
        synth_res = report_repeat(algos, sigs, SR, fmin, fmax, f_true,
                                  "반복 10회 (SNR 30dB, 잡음 시드만 다름)")

    if args.wav:
        paths = sorted(glob.glob(args.wav))
        if not paths:
            print(f"\nWAV를 찾지 못했습니다: {args.wav}")
        else:
            header(f"실제 녹음 — {len(paths)}개 파일")
            sigs, sr = [], SR
            for p in paths:
                x, sr = load_wav(p, args.skip)
                sigs.append(x)
            wav_label = f"{paths[0]} … {paths[-1]} ({len(paths)}개)"
            wav_res = report_repeat(algos, sigs, sr, fmin, fmax, args.truth,
                                    wav_label)
            if args.truth is None:
                print("\n  참값을 모르므로 표준편차(재현성)만 유효합니다.")
                print("  --truth 로 참값을 주면 절대 오차도 계산합니다.")

    if args.robust or args.all:
        report_robust(algos, f_true, SR, fmin, fmax, profile)

    if args.octave or args.all:
        report_octave(algos, f_true, SR, profile)

    # 합성과 실제를 둘 다 돌렸으면 나란히 놓고 마무리한다
    if synth_res and wav_res:
        report_summary(algos, synth_res, wav_res, wav_label)

    print()


if __name__ == "__main__":
    main()
