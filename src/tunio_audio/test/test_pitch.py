"""
pitch.py 단위 테스트. 마이크·ROS 없이 돈다.

합성 신호로 정확도를 보고, 잡음으로 유효 측정 판정이 실제로 막는지 본다.
실제 녹음(tools/*.wav)이 있으면 그것으로도 확인한다.
"""
import glob
import os
import wave

import numpy as np

from tunio_audio import pitch

SR_CASES = (44100, 48000)   # 장치가 48kHz 전용일 수 있다


def synth(f0, sr, sec, harmonics=(1.0, 0.5, 0.3), noise=0.003, seed=0):
    t = np.arange(int(sec * sr)) / sr
    x = np.zeros_like(t)
    for k, amp in enumerate(harmonics, start=1):
        x += amp * np.sin(2 * np.pi * f0 * k * t)
    x *= np.exp(-t / 4.0) * 0.3
    x += np.random.default_rng(seed).normal(0, noise, len(t))
    return x.astype(np.float32)


def test_search_range_is_two_semitones():
    """탐색 범위는 목표 +-2반음이어야 한다 (옥타브 오인 차단, CLAUDE.md 3.3)."""
    for s, (_, ft) in pitch.STRINGS.items():
        lo, hi = pitch.search_range(s)
        assert abs(pitch.cents(lo, ft) + 200) < 1e-6
        assert abs(pitch.cents(hi, ft) - 200) < 1e-6
        # 한 옥타브 위아래는 범위 밖이어야 한다
        assert not (lo <= ft / 2 <= hi)
        assert not (lo <= ft * 2 <= hi)


def test_accuracy_on_synthetic_tones():
    """합성 신호에서 전 줄 1센트 이내."""
    for sr in SR_CASES:
        for s, (_, ft) in pitch.STRINGS.items():
            a = pitch.analyze(synth(ft, sr, 2.0), sr, s)
            assert a.ok, f"{sr}Hz {s}번 줄 실패: {a.summary()}"
            assert abs(a.error_cents) < 1.0, f"{sr}Hz {s}번 줄 {a.error_cents:+.2f}센트"


def test_detuned_tone_is_measured_not_snapped():
    """목표에서 벗어난 음도 그 값 그대로 나와야 한다 (격자에 붙지 않는다)."""
    sr = 44100
    for s in (1, 6):
        for detune in (-150.0, -37.3, +80.0):
            ft = pitch.target_hz(s) * 2 ** (detune / 1200.0)
            a = pitch.analyze(synth(ft, sr, 2.0), sr, s)
            assert a.ok, f"{s}번 줄 {detune}센트 실패: {a.summary()}"
            assert abs(a.error_cents - detune) < 1.5


def test_silence_is_rejected():
    """무음은 '소리 부족'으로 막힌다."""
    sr = 44100
    a = pitch.analyze(np.zeros(int(2.0 * sr), np.float32), sr, 6)
    assert not a.ok and a.fail_reason == pitch.FAIL_LOW_LEVEL


def test_noise_is_rejected():
    """
    RMS는 충분하지만 음이 없는 잡음은 유효 측정으로 나가면 안 된다.

    이게 뚫리면 task_manager가 소음 기준으로 페그를 돌린다.
    """
    sr = 44100
    rng = np.random.default_rng(1)
    passed = 0
    trials = 30
    for i in range(trials):
        noise = rng.normal(0, 0.05, int(2.0 * sr)).astype(np.float32)
        assert pitch.rms_db(noise) > -50.0      # 레벨 판정은 통과하는 수준
        for s in (1, 6):
            if pitch.analyze(noise, sr, s).ok:
                passed += 1
    assert passed == 0, f"잡음 {passed}/{trials * 2}개가 유효 측정으로 통과했다"


def test_agreement_rejects_frequency_jump():
    """앞뒤가 다른 음이면 안정화 판정이 막는다."""
    sr = 44100
    s = 1
    ft = pitch.target_hz(s)
    a = synth(ft, sr, 1.0)
    b = synth(ft * 2 ** (150 / 1200.0), sr, 1.0, seed=2)
    mixed = np.concatenate([a, b])
    r = pitch.analyze(mixed, sr, s)
    assert not r.ok and r.fail_reason == pitch.FAIL_UNSTABLE, r.summary()


def _wavs(pattern):
    root = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools")
    return sorted(glob.glob(os.path.join(root, pattern)))


def _load(path):
    with wave.open(path) as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16)
    return x.astype(np.float32) / 32768.0, sr


def test_real_recordings_if_present():
    """실제 기타 녹음이 있으면 대부분 통과하고 편차가 작아야 한다."""
    for pattern, s, rec_sec, max_sd in (("g1_e4_*.wav", 1, 1.5, 1.5),
                                        ("g6_e2_*.wav", 6, 2.0, 3.0)):
        files = _wavs(pattern)
        if not files:
            continue
        errs = []
        for p in files:
            sig, sr = _load(p)
            seg = sig[:int(rec_sec * sr)][int(0.3 * sr):]
            a = pitch.analyze(seg, sr, s)
            if a.ok:
                errs.append(a.error_cents)
        assert len(errs) >= len(files) - 1, f"{pattern} 통과 {len(errs)}/{len(files)}"
        assert np.std(errs, ddof=1) <= max_sd, f"{pattern} 표준편차 {np.std(errs, ddof=1):.2f}센트"
