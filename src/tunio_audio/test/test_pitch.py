"""
pitch.py / recorder.py 단위 테스트. 마이크·ROS 그래프 없이 돈다.

합성 신호로 정확도를 보고, 잡음으로 유효 측정 판정과 onset 검출이 막는지 본다.
실제 녹음(tools/*.wav)이 있으면 그것으로도 확인한다.
"""
import glob
import os
import wave

import numpy as np

from tunio_audio import pitch
from tunio_audio import recorder

# 목표 주파수는 서버가 Goal로 준다. 테스트에서만 쓰는 값이다.
TARGETS = {6: 82.4069, 5: 110.0000, 2: 246.9417, 1: 329.6276}
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
    for ft in TARGETS.values():
        lo, hi = pitch.search_range(ft)
        assert abs(pitch.cents(lo, ft) + 200) < 1e-6
        assert abs(pitch.cents(hi, ft) - 200) < 1e-6
        # 한 옥타브 위아래는 범위 밖이어야 한다
        assert not (lo <= ft / 2 <= hi)
        assert not (lo <= ft * 2 <= hi)


def test_failure_reasons_come_from_the_message():
    """실패 사유는 MeasurePitch.Result 상수를 그대로 쓴다. 자체 정의 금지."""
    from tunio_interfaces.action import MeasurePitch
    r = MeasurePitch.Result
    assert pitch.FAIL_NONE == r.FAIL_NONE
    assert pitch.FAIL_LOW_SIGNAL == r.FAIL_LOW_SIGNAL
    assert pitch.FAIL_NO_PITCH == r.FAIL_NO_PITCH
    assert pitch.FAIL_TIMEOUT == r.FAIL_TIMEOUT
    assert pitch.FAIL_CANCELED == r.FAIL_CANCELED


def test_accuracy_on_synthetic_tones():
    """합성 신호에서 전 줄 1센트 이내."""
    for sr in SR_CASES:
        for s, ft in TARGETS.items():
            a = pitch.analyze(synth(ft, sr, 2.0), sr, ft)
            assert a.ok, f"{sr}Hz {s}번 줄 실패: {a.summary()}"
            assert abs(a.error_cents) < 1.0, f"{sr}Hz {s}번 줄 {a.error_cents:+.2f}센트"


def test_detuned_tone_is_measured_not_snapped():
    """목표에서 벗어난 음도 그 값 그대로 나와야 한다 (격자에 붙지 않는다)."""
    sr = 44100
    for s in (1, 6):
        for detune in (-150.0, -37.3, +80.0):
            ft = TARGETS[s]
            a = pitch.analyze(synth(ft * 2 ** (detune / 1200.0), sr, 2.0), sr, ft)
            assert a.ok, f"{s}번 줄 {detune}센트 실패: {a.summary()}"
            assert abs(a.error_cents - detune) < 1.5


def test_silence_is_rejected():
    """무음은 소리 부족으로 막힌다."""
    sr = 44100
    a = pitch.analyze(np.zeros(int(2.0 * sr), np.float32), sr, TARGETS[6])
    assert not a.ok and a.failure_reason == pitch.FAIL_LOW_SIGNAL


def test_noise_is_rejected():
    """
    RMS는 충분하지만 음이 없는 잡음은 유효 측정으로 나가면 안 된다.

    이게 뚫리면 task_manager가 소음 기준으로 페그를 돌린다.
    """
    sr = 44100
    rng = np.random.default_rng(1)
    passed, trials = 0, 30
    for _ in range(trials):
        noise = rng.normal(0, 0.05, int(2.0 * sr)).astype(np.float32)
        assert pitch.rms_db(noise) > -50.0      # 레벨 판정은 통과하는 수준
        for s in (1, 6):
            if pitch.analyze(noise, sr, TARGETS[s]).ok:
                passed += 1
    assert passed == 0, f"잡음 {passed}/{trials * 2}개가 유효 측정으로 통과했다"


def test_agreement_rejects_frequency_jump():
    """앞뒤가 다른 음이면 안정화 판정이 막는다."""
    sr, ft = 44100, TARGETS[1]
    mixed = np.concatenate([synth(ft, sr, 1.0),
                            synth(ft * 2 ** (150 / 1200.0), sr, 1.0, seed=2)])
    r = pitch.analyze(mixed, sr, ft)
    assert not r.ok and r.failure_reason == pitch.FAIL_NO_PITCH, r.summary()
    assert r.detail == "창 불일치", r.summary()


# ---------------- onset ----------------
def test_onset_found_on_pluck_after_silence():
    """무음 뒤에 음이 시작되면 그 시점을 찾는다."""
    sr, ft = 44100, TARGETS[1]
    rng = np.random.default_rng(3)
    quiet = rng.normal(0, 0.002, int(0.8 * sr)).astype(np.float32)
    sig = np.concatenate([quiet, synth(ft, sr, 1.5)])
    onsets = recorder.find_onsets(sig, sr)
    assert onsets, "onset을 못 찾았다"
    assert abs(onsets[0] - 0.8) < 0.1, f"발현 0.8초인데 {onsets[0]:.2f}초로 잡았다"


def test_onset_does_not_fire_on_steady_noise():
    """정상 상태 잡음만 있으면 onset이 나오면 안 된다."""
    sr = 44100
    rng = np.random.default_rng(4)
    for _ in range(5):
        noise = rng.normal(0, 0.02, int(5.0 * sr)).astype(np.float32)
        assert not recorder.find_onsets(noise, sr), "정상 잡음에서 onset이 나왔다"


# ---------------- 실제 녹음 ----------------
def _wavs(pattern):
    root = os.path.join(os.path.dirname(__file__), "..", "..", "..", "tools")
    return sorted(glob.glob(os.path.join(root, pattern)))


def _load(path):
    with wave.open(path) as w:
        sr = w.getframerate()
        x = np.frombuffer(w.readframes(w.getnframes()), np.int16)
    return x.astype(np.float32) / 32768.0, sr


def test_real_recordings_if_present():
    """실제 기타 녹음이 있으면 onset을 찾고, 대부분 통과하며 편차가 작아야 한다."""
    for pattern, s, rec_sec, max_sd in (("g1_e4_*.wav", 1, 1.5, 1.5),
                                        ("g6_e2_*.wav", 6, 2.0, 3.0)):
        files = _wavs(pattern)
        if not files:
            continue
        ft = TARGETS[s]
        errs = []
        for p in files:
            sig, sr = _load(p)
            onsets = recorder.find_onsets(sig, sr)
            assert onsets, f"{p} 에서 onset을 못 찾았다"
            st = int(onsets[0] * sr)
            seg = sig[st + int(0.3 * sr):st + int(rec_sec * sr)]
            a = pitch.analyze(seg, sr, ft)
            if a.ok:
                errs.append(a.error_cents)
        assert len(errs) >= len(files) - 2, f"{pattern} 통과 {len(errs)}/{len(files)}"
        assert np.std(errs, ddof=1) <= max_sd, \
            f"{pattern} 표준편차 {np.std(errs, ddof=1):.2f}센트"


# ---------------------------------------------------------------
# replay_wav 파일 회전 — audio_analyzer._resolve_replay
#
# 이 함수는 깨져도 조용하다. 회전이 망가져 같은 파일만 반복해도 sim 은 그대로
# 돌고 측정도 성공한다. 에러가 안 나고 흩어짐만 사라진다. 그래서 고정해 둔다.
#
# 노드를 띄우지 않는다. 이 함수가 self 에서 쓰는 것은 _replay_turn 하나뿐이라
# 그것만 가진 대역 객체로 부를 수 있다 (ROS 도 오디오도 필요 없다).
# ---------------------------------------------------------------
def _resolver():
    from types import SimpleNamespace

    from tunio_audio.audio_analyzer import AudioAnalyzer
    fake = SimpleNamespace(_replay_turn={})
    return lambda pattern, n: AudioAnalyzer._resolve_replay(fake, pattern, n)


def _make_wavs(d, names):
    for n in names:
        open(os.path.join(d, n), 'wb').close()
    return os.path.join(d, 'g{string_number}_*.wav')


def test_replay_rotates_through_matching_files(tmp_path):
    """회차마다 다음 파일을 쓴다. 같은 파일이 연속으로 나오면 안 된다."""
    pat = _make_wavs(str(tmp_path), ['g6_e2_1.wav', 'g6_e2_2.wav', 'g6_e2_3.wav'])
    r = _resolver()
    got = [os.path.basename(r(pat, 6)) for _ in range(3)]
    assert got == ['g6_e2_1.wav', 'g6_e2_2.wav', 'g6_e2_3.wav']


def test_replay_wraps_around_after_last_file(tmp_path):
    """파일 수를 넘기면 처음으로 돌아온다. 나머지 연산을 빼면 여기서 IndexError 가 난다."""
    pat = _make_wavs(str(tmp_path), ['g6_e2_1.wav', 'g6_e2_2.wav'])
    r = _resolver()
    got = [os.path.basename(r(pat, 6)) for _ in range(5)]
    assert got == ['g6_e2_1.wav', 'g6_e2_2.wav'] * 2 + ['g6_e2_1.wav']


def test_replay_counter_is_per_string_not_shared(tmp_path):
    """
    줄마다 카운터가 따로 돈다.

    키를 원본 패턴으로 잡으면 4줄이 카운터 하나를 공유해, 줄을 옮길 때마다
    회차가 건너뛰어진다. 그때 이 테스트가 깨진다.
    """
    pat = _make_wavs(str(tmp_path), ['g6_e2_1.wav', 'g6_e2_2.wav',
                                     'g5_a2_1.wav', 'g5_a2_2.wav'])
    r = _resolver()
    assert os.path.basename(r(pat, 6)) == 'g6_e2_1.wav'
    assert os.path.basename(r(pat, 5)) == 'g5_a2_1.wav'   # 6번이 먼저 돌았어도 1번째
    assert os.path.basename(r(pat, 6)) == 'g6_e2_2.wav'
    assert os.path.basename(r(pat, 5)) == 'g5_a2_2.wav'


def test_replay_without_wildcard_is_returned_as_is(tmp_path):
    """와일드카드가 없으면 glob 하지 않는다. 파일이 없어도 그대로 돌려준다."""
    r = _resolver()
    plain = str(tmp_path / 'g{string_number}_only.wav')
    assert r(plain, 2) == str(tmp_path / 'g2_only.wav')


def test_replay_raises_when_pattern_matches_nothing(tmp_path):
    """
    맞는 파일이 없으면 던진다.

    조용히 빈 값을 돌려주면 read_wav 가 엉뚱한 예외를 내고, 로그에 원인이 안 남는다.
    """
    import pytest
    r = _resolver()
    with pytest.raises(FileNotFoundError):
        r(str(tmp_path / 'g{string_number}_*.wav'), 6)
