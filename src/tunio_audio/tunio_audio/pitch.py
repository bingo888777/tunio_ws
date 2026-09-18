"""
주파수 분석 코어. ROS·오디오 장치에 의존하지 않는다.

tools/pitch_test.py에서 검증한 2단계 추정(FFT 유도 + 협대역 YIN)을 그대로 옮긴 것이다.
알고리즘 선정 근거는 docs/주파수분석_테스트가이드.md, 비교 도구는 tools/pitch_bench.py.

샘플레이트는 전역 상수가 아니라 인자로 받는다. 장치에 따라 48kHz 전용일 수 있다.
"""
import numpy as np

# 목표 주파수 (A4 = 440Hz, 12평균율)
STRINGS = {
    6: ("E2", 82.4069),
    5: ("A2", 110.0000),
    4: ("D3", 146.8324),
    3: ("G3", 195.9977),
    2: ("B3", 246.9417),
    1: ("E4", 329.6276),
}

# 유효 측정 판정 기준.
#
# RMS만으로는 방 소음을 못 거른다. 실측에서 방 소음 RMS(-25.7~-16 dBFS)가
# 실제 연주(-24.9~-10 dBFS)와 완전히 겹쳐, min_db 판정만 두면 소음을 음정으로 잡는다.
# 그대로 두면 task_manager가 소음 기준으로 페그를 돌린다.
#
# 대신 탐색 범위 안 FFT 피크가 주변보다 얼마나 솟았는지(prominence)로 가른다.
#   실제 신호 40개(기타 1·6번 줄 + 톤 330·82Hz): 최소 19.5 dB
#   방 소음 30구간                              : 최대 13.1 dB
# 사이 6.4 dB가 비어 있어 가운데인 16 dB를 경계로 둔다. 이 값에서 실제 100% 통과,
# 소음 0% 통과였다. 측정 스크립트는 세션 기록 참조.
#
# 주의: 로봇 타현 세기가 확정되면 다시 재야 한다. 사람 손보다 약하게 튕기면
#       돌출이 낮아져 정상 측정이 걸러질 수 있다.
MIN_PROMINENCE_DB = 16.0

# 안정화 판정 (CLAUDE.md 8절).
#
# 원안은 "짧게 여러 번 독립 측정해 값이 모이면 채택"이었는데 저음현에서 성립하지 않는다.
# 6번 줄을 1.0초로 자르면 그것만으로 50센트가 흔들려, 짧은 측정끼리는 절대 안 모인다.
# 그래서 짧게 여러 번이 아니라 "긴 창을 겹쳐서 여러 번" 재고 서로 일치하는지를 본다.
# 반환값은 전체 구간으로 낸 값(정밀도 최대)이고, 겹친 창들은 신뢰 확인에만 쓴다.
#
# 실측 (실제 40개 / 방 소음 180구간):
#   실제 신호의 창 일치 폭 : 중앙 1.34센트, 1번 줄 최대 2.65, 6번 줄 최대 28.59
#   방 소음               : 중앙 166.55센트, 최소 4.53
#
# 한계: 돌출·일치 두 관문을 다 걸어도 방 소음의 약 1%는 통과한다. 팬이나 전원 험은
#       그 자체가 안정된 음이라 신호만 보고는 줄 소리와 가를 수 없다. 남은 1%는
#       측정 재시도(최대 3회)와 최종 검증 패스로 잡는다.
MAX_AGREEMENT_CENTS = 12.0
AGREEMENT_WINDOWS = 3
AGREEMENT_FRAC = 0.6

# 실패 사유 (MeasurePitch.Result.fail_reason에 그대로 들어간다)
FAIL_LOW_LEVEL = "소리 부족"
FAIL_UNDETECTED = "주파수 판별 불가"
FAIL_UNSTABLE = "측정값 불안정"
FAIL_TIMEOUT = "시간 초과"
FAIL_CANCELED = "취소됨"
FAIL_NO_DEVICE = "오디오 장치 열기 실패"


def cents(f, f_target):
    return 1200.0 * np.log2(f / f_target)


def target_hz(string_no):
    return STRINGS[string_no][1]


def search_range(string_no, semitones=2.0):
    """탐색 범위를 목표 ±semitones 반음으로 제한한다 (옥타브 오인 차단, CLAUDE.md 3.3)."""
    ft = STRINGS[string_no][1]
    return ft * 2 ** (-semitones / 12), ft * 2 ** (semitones / 12)


def rms_db(x):
    r = np.sqrt(np.mean(x ** 2)) + 1e-12
    return float(20 * np.log10(r))


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
    a, b, c = (np.log(spec[k - 1] + 1e-12), np.log(spec[k] + 1e-12),
               np.log(spec[k + 1] + 1e-12))
    denom = a - 2 * b + c
    delta = 0.5 * (a - c) / denom if abs(denom) > 1e-12 else 0.0
    f = float((k + delta) * sr / N)
    # 피크가 범위 내 중앙값보다 얼마나 두드러지는지 (dB)
    prom = 20 * np.log10((spec[k] + 1e-12) / (np.median(spec[idx]) + 1e-12))
    return f, float(prom)


# ---------------------------------------------------------------
# YIN 기본 주파수 추정 (de Cheveigne & Kawahara, 2002)
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
    #    d(tau) = sum(x0^2) + sum(x[tau:tau+W]^2) - 2*corr(tau) 로 풀어 쓰면
    #    tau 루프 없이 누적합 + FFT 상호상관으로 한 번에 나온다.
    #    tau마다 내적하던 원본과 결과는 같고(상대오차 1e-16 수준) 9배 빠르다.
    #    저음현은 tau_max가 600을 넘어 이 차이가 응답 시간에 그대로 드러난다.
    x0 = x[:W]
    cum = np.concatenate(([0.0], np.cumsum(x * x)))
    taus = np.arange(tau_max)
    n_fft = 1 << int(np.ceil(np.log2(len(x) + W)))
    corr = np.fft.irfft(
        np.fft.rfft(x, n_fft) * np.conj(np.fft.rfft(x0, n_fft)), n_fft)[:tau_max]
    d = (cum[W] - cum[0]) + (cum[taus + W] - cum[taus]) - 2.0 * corr
    d[0] = 0.0
    np.maximum(d, 0.0, out=d)      # 반올림으로 음수가 나오는 것만 막는다

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


def _yin_frames(signal, sr, fmin, fmax, frame_len=None, hop=None):
    """여러 프레임에 YIN을 걸어 중앙값을 낸다. 반환: (f0, 유효, 전체, 배열)."""
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


def estimate_pitch(signal, sr, fmin, fmax, frame_len=None, hop=None):
    """
    2단계로 f0를 추정한다.

      1) 넓은 대역통과 -> FFT 최대 피크로 대략 위치 (잡음에 강함)
      2) 그 피크 +-100센트만 남기고 YIN -> 정밀값

    FFT 피크가 불명확하면 넓은 범위 YIN으로 후퇴한다.
    반환: (f0, 유효 프레임 수, 전체 프레임 수, 프레임별 f0, 방법, 피크 돌출 dB)
    """
    wide = bandpass(signal, sr, fmin * 0.85, fmax * 4.2)
    res = fft_peak(wide, sr, fmin, fmax)
    f_fft, prom = (res if res else (None, 0.0))

    if f_fft is not None and prom >= 6.0:
        # 피크 주변 +-100센트로 좁혀서 YIN
        lo, hi = f_fft * 2 ** (-1 / 12), f_fft * 2 ** (1 / 12)
        narrow = bandpass(signal, sr, lo, hi)
        f_yin, ok, n, f0s = _yin_frames(narrow, sr, lo, hi, frame_len, hop)
        if f_yin is not None and ok >= 0.3 * n:
            return f_yin, ok, n, f0s, "yin", prom
        return f_fft, ok, n, f0s, "fft", prom

    # FFT 피크 불명확 -> 넓은 범위 YIN
    f_yin, ok, n, f0s = _yin_frames(wide, sr, fmin, fmax, frame_len, hop)
    if f_yin is not None:
        return f_yin, ok, n, f0s, "yin-wide", prom
    if f_fft is not None:
        return f_fft, 0, n, np.array([]), "fft-weak", prom
    return None, 0, n, np.array([]), "none", prom


# ---------------------------------------------------------------
# 노드가 쓰는 진입점
# ---------------------------------------------------------------
class Analysis:
    """측정 1회 결과. hz가 None이면 실패이고 fail_reason에 사유가 들어간다."""

    __slots__ = ("hz", "fail_reason", "method", "level_db", "prominence_db",
                 "valid_frames", "total_frames", "spread_cents", "error_cents",
                 "agreement_cents")

    def __init__(self, hz=None, fail_reason="", method="none", level_db=float("-inf"),
                 prominence_db=float("nan"), valid_frames=0, total_frames=0,
                 spread_cents=float("nan"), error_cents=float("nan"),
                 agreement_cents=float("nan")):
        self.hz = hz
        self.fail_reason = fail_reason
        self.method = method
        self.level_db = level_db
        self.prominence_db = prominence_db
        self.valid_frames = valid_frames
        self.total_frames = total_frames
        self.spread_cents = spread_cents
        self.error_cents = error_cents
        self.agreement_cents = agreement_cents

    @property
    def ok(self):
        return self.hz is not None

    def summary(self):
        if not self.ok:
            return (f"{self.fail_reason} (RMS {self.level_db:.1f} dBFS, "
                    f"돌출 {self.prominence_db:.1f} dB, "
                    f"일치 {self.agreement_cents:.2f}센트)")
        return (f"{self.hz:.3f} Hz [{self.method}] {self.error_cents:+.2f}센트  "
                f"유효 {self.valid_frames}/{self.total_frames}  "
                f"산포 {self.spread_cents:.2f}센트  일치 {self.agreement_cents:.2f}센트  "
                f"돌출 {self.prominence_db:.1f} dB  RMS {self.level_db:.1f} dBFS")


def _frame_spread_cents(f0s, center):
    """프레임별 추정값이 얼마나 흩어져 있는지. 측정 신뢰도 참고용(판정에는 안 쓴다)."""
    if f0s is None or len(f0s) < 2:
        return float("nan")
    c = cents(np.asarray(f0s, dtype=float), center)
    keep = np.abs(c) < 50
    if keep.sum() < 2:
        return float("nan")
    return float(np.std(c[keep], ddof=1))


def window_agreement(signal, sr, fmin, fmax,
                     n_windows=AGREEMENT_WINDOWS, frac=AGREEMENT_FRAC):
    """
    겹치는 창들이 서로 얼마나 일치하는지를 센트 폭으로 돌려준다.

    구간을 겹치는 창 n개로 나눠 각각 추정한 뒤 최대 편차를 본다.
    창 하나라도 추정에 실패하면 inf (= 불일치로 본다).
    """
    L = int(len(signal) * frac)
    if L < int(4.5 * sr / fmin) * 2:
        return float("inf")
    vals = []
    for off in np.linspace(0, len(signal) - L, n_windows).astype(int):
        f = estimate_pitch(signal[off:off + L], sr, fmin, fmax)[0]
        if f is None:
            return float("inf")
        vals.append(f)
    v = np.array(vals)
    return float(np.abs(cents(v, np.median(v))).max() * 2)


def analyze(signal, sr, string_no, min_db=-50.0, semitones=2.0,
            min_prominence_db=MIN_PROMINENCE_DB,
            max_agreement_cents=MAX_AGREEMENT_CENTS):
    """
    녹음 구간 하나를 분석한다.

    과도음 제외는 호출 전에 끝나 있어야 한다. 반환: Analysis.
    """
    signal = np.asarray(signal, dtype=np.float32)
    level = rms_db(signal) if len(signal) else float("-inf")

    if level < min_db:
        return Analysis(fail_reason=FAIL_LOW_LEVEL, level_db=level)

    fmin, fmax = search_range(string_no, semitones)
    f0, ok, total, f0s, method, prom = estimate_pitch(signal, sr, fmin, fmax)

    if f0 is None:
        return Analysis(fail_reason=FAIL_UNDETECTED, level_db=level,
                        prominence_db=prom, method=method, total_frames=total)

    # 소리는 충분한데 탐색 범위에 뚜렷한 음이 없는 경우 (방 소음·잡음).
    # 여기서 막지 않으면 소음이 유효 측정값으로 나간다.
    if prom < min_prominence_db:
        return Analysis(fail_reason=FAIL_UNDETECTED, level_db=level,
                        prominence_db=prom, method=method,
                        valid_frames=ok, total_frames=total,
                        spread_cents=_frame_spread_cents(f0s, f0))

    # 안정화 판정: 겹치는 창들이 서로 일치해야 채택한다.
    agree = window_agreement(signal, sr, fmin, fmax)
    if agree > max_agreement_cents:
        return Analysis(fail_reason=FAIL_UNSTABLE, level_db=level,
                        prominence_db=prom, method=method,
                        valid_frames=ok, total_frames=total,
                        spread_cents=_frame_spread_cents(f0s, f0),
                        agreement_cents=agree)

    return Analysis(
        hz=float(f0), method=method, level_db=level, prominence_db=prom,
        valid_frames=ok, total_frames=total,
        spread_cents=_frame_spread_cents(f0s, f0),
        error_cents=float(cents(f0, target_hz(string_no))),
        agreement_cents=agree,
    )
