"""
마이크 녹음 래퍼. sounddevice로 상시 스트림을 열어두고 요청 구간만 떠간다.

상시 스트림으로 둔 이유:
  measure_pitch는 "녹음 준비됨"을 Feedback으로 알린 뒤에 로봇이 줄을 튕긴다.
  장치를 그때 열면 여는 시간(실측 21ms)만큼 발현 시작을 놓친다.
  스트림을 먼저 띄워두면 ready는 즉시 나가고, 버린 구간 없이 발현 직후부터 받는다.

샘플레이트는 요청값 -> 장치 기본값 -> 48000 -> 44100 순으로 시도한다.
장치가 48kHz 전용인 경우가 있어 44100을 고정하면 열리지 않는다.
실제로 열린 값은 self.sample_rate에 들어가고, 분석에는 이 값을 넘겨야 한다.

OS의 마이크 향상 기능(노이즈 제거·AGC·에코 제거)은 반드시 꺼둘 것 (CLAUDE.md 3.5).
"""
import queue
import time

import numpy as np

try:
    import sounddevice as sd
except ImportError:      # 장치 없는 환경에서 import만 하는 경우 (테스트 등)
    sd = None


class RecorderError(RuntimeError):
    pass


# ---------------------------------------------------------------
# onset(무음 -> 소리) 검출
# ---------------------------------------------------------------
#
# 왜 '직전 구간 대비 상승'으로 보나:
#   절대 레벨로는 못 가른다. 실측에서 방 소음 RMS(-25.7~-16 dBFS)가 실제 연주
#   (-24.9~-10 dBFS)와 완전히 겹쳤다. "커지면 발현"은 성립하지 않는다.
#   고정 기준선 대비로 봐도 방 소음이 자기 중앙값보다 최대 +13.8 dB까지 출렁여서,
#   임계 +12 dB 한 프레임 기준이면 30초에 117번 오발했다.
#
# 왜 임계값을 높이지 않나:
#   발현의 공격음이 짧아서다. 임계를 +24 dB로 올리면 소음은 막히지만 실제 발현을
#   20개 중 14개밖에 못 잡았다. 효과가 있던 건 임계가 아니라 '지속' 조건이다.
#   소음 스파이크는 한두 프레임이고 발현은 이어진다.
#
# 왜 느슨하게 잡나 (RISE_DB 8.0, PERSIST 2):
#   엄격하게 잡아 오발을 0으로 만들면 조용한 조건에서만 통한다. 잡음을 섞어 재보니
#   어느 설정이든 잡음이 커지면 무너지는데, 그보다 먼저 주파수 측정 자체가 무너졌다.
#     잡음 -6dB : 엄격 onset 20/20, 측정은 10/20
#     잡음  0dB : 엄격 onset  8/20, 측정은 10/20
#   그래서 onset은 후보를 넉넉히 내놓고, 진짜 발현인지는 pitch.analyze 의 관문
#   (돌출 16dB + 창 일치 12센트)이 판정하게 한다. 이 방식이 모든 잡음 레벨에서
#   "발현 위치를 정답으로 알려준 경우"와 같은 성적을 냈다. onset 때문에 잃는 게 없다.
#
# 아직 못 잰 것: 로봇 구동음. 그리퍼 접촉음이 발현처럼 뾰족할 수 있다.
#   오발이 나도 분석 관문에서 걸러지므로 측정이 틀리지는 않고 대기 시간만 늘어난다.
#   로봇이 붙는 대로 확인할 것.
FRAME_SEC = 0.010
HOP_SEC = 0.005
ONSET_LOOKBACK_FRAMES = 40    # 직전 200ms를 기준 구간으로
ONSET_GAP_FRAMES = 4          # 20ms 띄운다 (상승 시작이 기준에 섞이지 않게)
ONSET_RISE_DB = 8.0
ONSET_PERSIST_FRAMES = 2
ONSET_MIN_GAP_SEC = 0.3       # 같은 발현을 여러 번 세지 않도록


def frame_levels(x, sr, frame_sec=FRAME_SEC, hop_sec=HOP_SEC):
    """프레임별 RMS를 dB로. 반환 길이는 (len(x) - frame) // hop + 1."""
    n, h = int(frame_sec * sr), int(hop_sec * sr)
    if len(x) < n:
        return np.zeros(0)
    starts = np.arange(0, len(x) - n + 1, h)
    out = np.empty(len(starts))
    for i, st in enumerate(starts):
        seg = x[st:st + n]
        out[i] = 20 * np.log10(np.sqrt(np.mean(seg * seg)) + 1e-12)
    return out


def local_rise(levels, lookback=ONSET_LOOKBACK_FRAMES, gap=ONSET_GAP_FRAMES):
    """직전 구간 중앙값 대비 몇 dB 높은지. 잡음 바닥이 변해도 따라간다."""
    out = np.full(len(levels), -99.0)
    for i in range(lookback + gap, len(levels)):
        out[i] = levels[i] - np.median(levels[i - lookback - gap:i - gap])
    return out


def find_onsets(x, sr, rise_db=ONSET_RISE_DB, persist=ONSET_PERSIST_FRAMES,
                min_gap_sec=ONSET_MIN_GAP_SEC, hop_sec=HOP_SEC):
    """
    소리가 시작된 지점(onset) 후보들을 초 단위로 돌려준다.

    느슨하게 잡는다. 진짜 발현인지는 부르는 쪽이 pitch.analyze 로 판정한다.
    """
    rise = local_rise(frame_levels(x, sr, hop_sec=hop_sec))
    out, run, last = [], 0, -1e9
    for i, over in enumerate(rise > rise_db):
        run = run + 1 if over else 0
        if run == persist:
            t = (i - persist + 1) * hop_sec
            if t - last >= min_gap_sec:
                out.append(t)
                last = t
    return out


def read_wav(path):
    """
    WAV를 모노 float32로 읽는다.

    반환: (신호, 샘플레이트)

    마이크 없이 노드를 돌려볼 때 쓴다. 서버·관제 쪽에서 실제 마이크를 받기 전에
    audio_analyzer를 그대로 띄워 흐름을 확인할 수 있다.
    """
    import wave
    with wave.open(str(path), "rb") as w:
        sr = w.getframerate()
        ch = w.getnchannels()
        width = w.getsampwidth()
        raw = w.readframes(w.getnframes())
    if width != 2:
        raise RecorderError(f"16bit WAV만 지원한다 (이 파일은 {width * 8}bit): {path}")
    x = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
    if ch > 1:
        x = x.reshape(-1, ch).mean(axis=1)
    return x, sr


class Recorder:
    def __init__(self, sample_rate=44100, device=None, queue_sec=8.0):
        self.requested_rate = int(sample_rate)
        self.sample_rate = int(sample_rate)
        self.device = device
        self.queue_sec = float(queue_sec)
        self._stream = None
        self._q = queue.Queue()
        self._queued = 0          # 큐에 쌓인 샘플 수
        self.overflow_count = 0

    # ---------------- 장치 ----------------
    def _candidate_rates(self):
        rates = [self.requested_rate]
        try:
            info = sd.query_devices(self.device, "input")
            default = int(info.get("default_samplerate") or 0)
            if default:
                rates.append(default)
        except Exception:
            pass
        rates += [48000, 44100]
        seen, out = set(), []
        for r in rates:
            if r and r not in seen:
                seen.add(r)
                out.append(int(r))
        return out

    def _callback(self, indata, frames, time_info, status):
        if status:
            self.overflow_count += 1
        self._q.put(indata[:, 0].copy())
        self._queued += frames
        # 아무도 안 읽어가는 동안 무한정 쌓이지 않게 오래된 것부터 버린다
        limit = int(self.queue_sec * self.sample_rate)
        while self._queued > limit:
            try:
                self._queued -= len(self._q.get_nowait())
            except queue.Empty:
                break

    def open(self):  # noqa: A003  (스트림을 여는 동작이라 이 이름이 자연스럽다)
        """스트림을 연다. 이미 열려 있으면 아무 것도 하지 않는다."""
        if sd is None:
            raise RecorderError("sounddevice가 설치되어 있지 않습니다 (pip install sounddevice)")
        if self._stream is not None and self._stream.active:
            return self.sample_rate

        errors = []
        for rate in self._candidate_rates():
            try:
                stream = sd.InputStream(
                    samplerate=rate, channels=1, dtype="float32",
                    device=self.device, callback=self._callback)
                stream.start()
            except Exception as exc:
                errors.append(f"{rate}Hz: {exc}")
                continue
            self._stream = stream
            self.sample_rate = int(rate)
            return self.sample_rate

        raise RecorderError("입력 장치를 열 수 없습니다 — " + " / ".join(errors))

    def close(self):
        if self._stream is not None:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    @property
    def is_open(self):
        return self._stream is not None and self._stream.active

    def device_name(self):
        try:
            return sd.query_devices(self.device, "input")["name"]
        except Exception:
            return "(알 수 없음)"

    # ---------------- 녹음 ----------------
    def flush(self):
        """쌓여 있던 과거 구간을 버린다. 녹음 시작 직전에 부른다."""
        while True:
            try:
                self._q.get_nowait()
            except queue.Empty:
                break
        self._queued = 0

    def read(self, duration, abort=None, timeout=None):
        """
        duration초를 모아서 돌려준다.

        abort: 호출할 때마다 bool을 주는 함수. True가 되면 None을 반환한다(취소).
        timeout: 이 시간 안에 못 모으면 TimeoutError.
        """
        if not self.is_open:
            self.open()

        need = int(duration * self.sample_rate)
        deadline = time.monotonic() + (timeout if timeout else duration * 3 + 1.0)
        chunks, got = [], 0

        while got < need:
            if abort is not None and abort():
                return None
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise TimeoutError(f"녹음 {duration}초를 모으지 못했습니다 "
                                   f"({got}/{need} 샘플)")
            try:
                block = self._q.get(timeout=min(0.1, remain))
            except queue.Empty:
                continue
            self._queued -= len(block)
            chunks.append(block)
            got += len(block)

        return np.concatenate(chunks)[:need]

    # 컨텍스트 매니저로도 쓸 수 있게
    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
