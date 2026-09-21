"""
audio_analyzer — /audio/measure_pitch Action 서버.

요청 시에만 녹음/분석한다. 흐름은 이렇다.

  Goal 수신 -> 녹음 스트림 확인 -> ready Feedback -> 로봇이 줄을 튕김
           -> onset(무음->소리) 후보 검출 -> 후보마다 창을 떠서 분석
           -> 관문을 통과하는 첫 후보를 채택 -> Result

onset을 느슨하게 잡고 분석이 판정하는 이유는 recorder.py 주석에 실측과 함께 적어뒀다.
요약하면, 잡음이 onset을 깨뜨릴 정도면 주파수 측정 자체가 이미 먼저 무너진다.
그래서 onset을 엄격하게 만들 이유가 없고, 느슨하게 두고 pitch.analyze 의 관문
(돌출 16dB + 창 일치 12센트)이 진짜 발현인지 가르게 하는 편이 모든 잡음 레벨에서 낫다.

타임아웃은 두 가지다 (명세 4절).
  onset_timeout_sec — ready 발행 시점부터. 이 안에 onset이 없으면 발현 자체가 실패다
  timeout_sec       — onset 시점부터. 로봇 접근·파지 시간이 타임아웃에 먹히지 않는다

Result에는 요청의 job_id·string_number·measurement_id를 그대로 실어 돌려준다.
늦게 도착한 이전 측정이 현재 결과로 쓰이지 않게 하기 위한 것이다(명세 4절).
같은 이유로 측정은 한 번에 하나만 받는다. 이미 진행 중이면 새 Goal을 거부한다.
"""
import glob
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tunio_audio import pitch
from tunio_audio.recorder import find_onsets, read_wav, Recorder, RecorderError

from tunio_interfaces.action import MeasurePitch

# 녹음 길이(초). onset 이후 이만큼을 분석에 쓴다.
#
# 저음현은 한 주기가 길어 짧게 자르면 값이 흔들린다. 여기에 안정화 판정이 겹치는 창을
# 세 개 쓰므로 창 하나가 충분히 길어야 한다. 실제 녹음 10개씩으로 스윕한 결과:
#   1번 줄(E4)  0.8초 7/10 · 1.0초 8/10 · 1.2초 7/10 · 1.5초 10/10
#   6번 줄(E2)  0.8초 1/10 · 1.0초 7/10 · 1.5초 8/10 · 2.0초 9/10
# 줄 번호가 아니라 목표 주파수로 가른다. 목표 주파수 표의 단일 출처는 서버다.
RECORD_SEC_HIGH = 1.5
RECORD_SEC_LOW = 2.0
LOW_FREQ_HZ = 150.0       # 이 아래면 저음현으로 본다 (E2 82.4 / A2 110 vs B3 246.9)

CHUNK_SEC = 0.1           # onset을 보며 받아들이는 단위
MIN_USABLE_SEC = 0.7      # 입력이 끝난 뒤(WAV 모드) 잘린 꼬리라도 이만큼은 있어야 분석한다
RESPONSE_GOAL_SEC = 2.2   # onset 이후 정상 목표. 넘으면 경고만 남긴다


class AudioAnalyzer(Node):
    """measure_pitch Action 서버 노드."""

    def __init__(self):
        super().__init__("audio_analyzer")

        self.declare_parameter("device", -1)             # -1이면 시스템 기본 입력
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("skip_head_sec", 0.3)     # onset 직후 과도음 제외
        self.declare_parameter("min_db", -50.0)          # 소리 부족 판정
        self.declare_parameter("min_prominence_db", pitch.MIN_PROMINENCE_DB)
        self.declare_parameter("max_agreement_cents", pitch.MAX_AGREEMENT_CENTS)
        self.declare_parameter("search_semitones", 2.0)  # 목표 +-2반음
        self.declare_parameter("record_sec_high", RECORD_SEC_HIGH)
        self.declare_parameter("record_sec_low", RECORD_SEC_LOW)
        self.declare_parameter("low_freq_hz", LOW_FREQ_HZ)
        self.declare_parameter("default_timeout_sec", 10.0)
        self.declare_parameter("default_onset_timeout_sec", 10.0)
        # 마이크 대신 WAV를 읽는다 (마이크 없이 흐름 확인용). 빈 문자열이면 마이크.
        # {string_number}를 넣으면 줄 번호로 치환되고, 와일드카드는 glob로 펼쳐
        # 회차마다 다른 파일을 돌아가며 쓴다. 예: tools/g{string_number}_*.wav
        # 줄마다 음 이름이 달라(g6_e2 / g5_a2 / g2_b3 / g1_e4) 한 패턴으로 4줄을
        # 덮으려면 와일드카드가 있어야 한다. _resolve_replay 참조
        self.declare_parameter("replay_wav", "")

        dev = self.get_parameter("device").value
        self.recorder = Recorder(
            sample_rate=self.get_parameter("sample_rate").value,
            device=None if dev is None or dev < 0 else dev,
        )

        self._busy = False
        # replay 회차 카운터. 키는 **줄 번호를 치환한 뒤, glob 하기 전** 경로다.
        # 키를 원본 패턴으로 바꾸면 4줄이 카운터 하나를 공유해 회전이 망가진다.
        # 자세한 이유는 _resolve_replay 의 "키를 무엇으로 잡는가" 참조.
        self._replay_turn = {}
        self._cb_group = ReentrantCallbackGroup()
        self.srv = ActionServer(
            self, MeasurePitch, "/audio/measure_pitch",
            execute_callback=self.on_measure,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self._cb_group,
        )

        # 첫 측정이 장치 여는 시간을 물지 않도록 미리 열어둔다. 실패해도 측정 때 다시 시도.
        try:
            sr = self.recorder.open()
            self.get_logger().info(
                f"입력 장치 '{self.recorder.device_name()}' {sr} Hz 로 열림")
            if sr != self.recorder.requested_rate:
                self.get_logger().warn(
                    f"요청 {self.recorder.requested_rate} Hz 는 지원되지 않아 {sr} Hz 로 "
                    f"열었다. 분석은 실제 값으로 한다.")
        except RecorderError as exc:
            self.get_logger().warn(f"입력 장치를 아직 열지 못했다: {exc}")

        self.get_logger().info("audio_analyzer 시작 — /audio/measure_pitch 대기")

    # ---------------- Goal 수락 정책 ----------------
    def on_goal(self, goal_request):
        if not (20.0 < goal_request.target_hz < 5000.0):
            self.get_logger().warn(
                f"목표 주파수 {goal_request.target_hz} Hz 가 범위를 벗어난다 — 거부")
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn(
                f"측정 진행 중이라 거부 (job {goal_request.job_id} / "
                f"{goal_request.string_number}번 / m{goal_request.measurement_id})")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info("취소 요청 수신")
        return CancelResponse.ACCEPT

    # ---------------- 보조 ----------------
    def _resolve_replay(self, pattern, string_number):
        """
        replay_wav 패턴을 실제 파일 하나로 푼다.

        {string_number} 를 줄 번호로 바꾸고, 와일드카드가 있으면 glob 로 펼쳐
        **회차마다 다른 파일을 돌아가며** 고른다.

        왜 와일드카드가 필요한가
          녹음 파일 이름에 음 이름이 들어 있어 줄마다 다르다
          (g6_e2 / g5_a2 / g2_b3 / g1_e4). {string_number} 치환만으로는
          한 패턴이 4줄을 못 덮는다.

        왜 돌아가며 쓰는가
          같은 파일만 반복하면 매번 같은 값이 나와 흐름은 돌지만 흩어짐이
          드러나지 않는다. 10개를 번갈아 쓰면 실제 측정에 가까워진다.

        정렬한 순서대로 도는 것은 의도한 것이다. 무작위면 실패를 재현할 수 없다.

        키를 무엇으로 잡는가 — 고치기 전에 읽을 것
          self._replay_turn 의 키는 "줄 번호를 치환한 뒤, glob 하기 전" 경로다
          (예: tools/g6_*.wav). 원본 패턴(tools/g{string_number}_*.wav)으로
          바꾸면 더 단순해 보이지만 **4줄이 카운터 하나를 공유한다.** 줄마다 녹음
          개수가 다르면 건너뛰기가 제각각이 되고, 줄을 옮길 때마다 회차가 엉킨다.

        이 함수는 깨져도 조용하다
          존재 이유가 "회차마다 다른 파일"인데, 그 성질이 깨져서 같은 파일만
          반복해도 sim 은 그대로 돌고 측정도 성공하고 조율도 수렴한다. 에러가 안
          난다. 없어지는 것은 재측정 간의 흩어짐뿐이라 눈으로는 안 보인다.
          그래서 test_pitch.py 에 회전·키 분리·경계를 고정하는 테스트를 두었다
          (test_replay_* 4개). 여기를 고치면 그쪽도 같이 본다.
        """
        path = pattern.replace("{string_number}", str(string_number))
        if not any(c in path for c in "*?["):
            return path
        hits = sorted(glob.glob(path))
        if not hits:
            raise FileNotFoundError(f"패턴에 맞는 파일이 없다: {path}")
        i = self._replay_turn.get(path, 0)
        self._replay_turn[path] = i + 1
        return hits[i % len(hits)]

    def _record_sec(self, target_hz):
        if target_hz < float(self.get_parameter("low_freq_hz").value):
            return float(self.get_parameter("record_sec_low").value)
        return float(self.get_parameter("record_sec_high").value)

    def _analyze(self, sig, sr, target_hz):
        return pitch.analyze(
            sig, sr, target_hz,
            min_db=float(self.get_parameter("min_db").value),
            semitones=float(self.get_parameter("search_semitones").value),
            min_prominence_db=float(self.get_parameter("min_prominence_db").value),
            max_agreement_cents=float(self.get_parameter("max_agreement_cents").value),
        )

    def _result(self, req, analysis=None, failure_reason=None):
        res = MeasurePitch.Result()
        res.job_id = req.job_id
        res.string_number = req.string_number
        res.measurement_id = req.measurement_id
        if analysis is not None and analysis.ok:
            res.success = True
            res.frequency_hz = float(analysis.hz)
            res.failure_reason = pitch.FAIL_NONE
        else:
            res.success = False
            res.frequency_hz = 0.0   # success=false면 읽지 않는다 (명세 4절)
            if failure_reason is not None:
                res.failure_reason = failure_reason
            elif analysis is not None:
                res.failure_reason = analysis.failure_reason
            else:
                res.failure_reason = pitch.FAIL_NO_PITCH
        return res

    # ---------------- 측정 ----------------
    def on_measure(self, goal_handle):
        req = goal_handle.request
        tag = (f"job {req.job_id} / {req.string_number}번 / m{req.measurement_id} "
               f"/ 목표 {req.target_hz:.2f}Hz")
        self._busy = True
        try:
            return self._measure(goal_handle, req, tag)
        except Exception as exc:                     # 어떤 경우에도 Result는 돌려준다
            self.get_logger().error(f"[{tag}] 예외: {exc!r}")
            if goal_handle.is_active:
                goal_handle.abort()
            return self._result(req, failure_reason=pitch.FAIL_NO_PITCH)
        finally:
            self._busy = False

    def _measure(self, goal_handle, req, tag):
        rec_sec = self._record_sec(req.target_hz)
        skip = float(self.get_parameter("skip_head_sec").value)
        timeout = req.timeout_sec if req.timeout_sec > 0 else \
            float(self.get_parameter("default_timeout_sec").value)
        onset_timeout = req.onset_timeout_sec if req.onset_timeout_sec > 0 else \
            float(self.get_parameter("default_onset_timeout_sec").value)
        replay = str(self.get_parameter("replay_wav").value or "")

        # --- 입력 준비 ---
        if replay:
            # path 를 먼저 None 으로 둔다. _resolve_replay 가 던지면 path 가 아직
            # 없는데 로그에서 그대로 쓰면 에러 처리하다 NameError 가 난다.
            # 파일까지 풀렸으면 그 파일을, 못 풀었으면 원본 패턴을 찍는다 —
            # 회차마다 파일이 바뀌므로 어느 파일에서 터졌는지가 중요하다.
            # (wave.Error "file does not start with RIFF id" 처럼 예외 메시지에
            #  파일명이 없는 경우가 있다)
            path = None
            try:
                path = self._resolve_replay(replay, req.string_number)
                buf, sr = read_wav(path)
            except Exception as exc:
                self.get_logger().error(
                    f"[{tag}] WAV를 읽지 못했다 {path or replay}: {exc}")
                goal_handle.abort()
                return self._result(req, failure_reason=pitch.FAIL_LOW_SIGNAL)
            self.get_logger().info(
                f"[{tag}] WAV 재생 모드 — {path} ({sr} Hz, {len(buf) / sr:.2f}초)")
        else:
            try:
                sr = self.recorder.open()
            except RecorderError as exc:
                # 장치가 없으면 재시도해도 소용없지만 전용 상수가 없다. pitch.py 주석 참조
                self.get_logger().error(f"[{tag}] 오디오 장치 열기 실패: {exc}")
                goal_handle.abort()
                return self._result(req, failure_reason=pitch.FAIL_LOW_SIGNAL)
            self.recorder.flush()
            buf = np.zeros(0, dtype=np.float32)

        fb = MeasurePitch.Feedback()
        fb.ready = True
        goal_handle.publish_feedback(fb)
        t_ready = time.monotonic()
        self.get_logger().info(
            f"[{tag}] 준비 완료 — onset 대기 (녹음 {rec_sec}초, {sr} Hz)")

        # --- onset 후보를 보며 분석 ---
        # 마이크는 계속 들어오니 onset 뒤로 rec_sec 이 찰 때까지 기다리면 된다.
        # WAV 모드는 입력에 끝이 있어, 꼬리가 짧으면 있는 만큼으로 분석한다.
        need = int((rec_sec + 0.05) * sr)
        min_need = int((skip + MIN_USABLE_SEC) * sr)
        finite = bool(replay)
        # t_onset: 첫 onset (명세 4절의 타임아웃 기준점)
        # wall: 후보별 실제 시각. 응답 시간은 '성공한 후보' 기준으로 재야 한다.
        #   로봇 구동음에서는 헛 onset 이 분당 19회 나고 2.2초 창의 48%에 들어온다.
        #   첫 onset 으로 재면 진짜 발현은 제때 잡았는데도 목표 초과로 찍힌다.
        tried, t_onset, last, wall = set(), None, None, {}

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"[{tag}] 취소됨")
                return self._result(req, failure_reason=pitch.FAIL_CANCELED)

            now = time.monotonic()
            if t_onset is None and now - t_ready > onset_timeout:
                self.get_logger().warn(
                    f"[{tag}] onset 미검출 {onset_timeout:.1f}초 — 발현 실패")
                goal_handle.abort()
                return self._result(req, failure_reason=pitch.FAIL_LOW_SIGNAL)
            if t_onset is not None and now - t_onset > timeout:
                # 여기 오는 흔한 경우는 발현이 실패해 잡음에 onset이 잘못 걸린 때다.
                # 명세 4절에 따라 사유는 FAIL_TIMEOUT 으로 내보내되,
                # 후보를 몇 개 왜 기각했는지는 로그에 남겨 진단할 수 있게 한다.
                why = f" 마지막 기각: {last.summary()}" if last is not None else \
                      " 분석 가능한 후보가 없었다"
                self.get_logger().error(
                    f"[{tag}] onset 이후 {timeout:.1f}초 초과 "
                    f"(후보 {len(tried)}개 기각).{why}")
                goal_handle.abort()
                return self._result(req, failure_reason=pitch.FAIL_TIMEOUT)

            for t in find_onsets(buf, sr):
                if t in tried:
                    continue
                if t not in wall:
                    wall[t] = time.monotonic() - (len(buf) / sr - t)
                if t_onset is None:
                    t_onset = wall[t]
                start = int(t * sr)
                avail = len(buf) - start
                if avail < need and not (finite and avail >= min_need):
                    continue        # 아직 뒤가 덜 찼다 (WAV는 최소 길이만 넘으면 간다)
                tried.add(t)
                seg = buf[start + int(skip * sr):start + int(rec_sec * sr)]
                a = self._analyze(seg, sr, req.target_hz)
                last = a
                if a.ok:
                    el = time.monotonic() - wall[t]
                    msg = (f"[{tag}] onset {t:.2f}초 → {a.summary()} — {el:.2f}초"
                           + (f" (앞선 후보 {len(tried) - 1}개 기각)"
                              if len(tried) > 1 else ""))
                    if el > RESPONSE_GOAL_SEC:
                        self.get_logger().warn(f"{msg} (목표 {RESPONSE_GOAL_SEC}초 초과)")
                    else:
                        self.get_logger().info(msg)
                    goal_handle.succeed()
                    return self._result(req, a)
                self.get_logger().info(f"[{tag}] onset {t:.2f}초 후보 기각 — {a.summary()}")

            if replay:
                break               # 파일은 더 들어올 게 없다

            try:
                chunk = self.recorder.read(
                    CHUNK_SEC, abort=lambda: goal_handle.is_cancel_requested)
            except TimeoutError as exc:
                self.get_logger().error(f"[{tag}] {exc}")
                goal_handle.abort()
                return self._result(req, failure_reason=pitch.FAIL_TIMEOUT)
            if chunk is None:
                goal_handle.canceled()
                return self._result(req, failure_reason=pitch.FAIL_CANCELED)
            buf = np.concatenate([buf, chunk])

        # 파일 모드에서 후보를 다 써버린 경우
        if last is None:
            self.get_logger().warn(f"[{tag}] onset 후보 없음")
            goal_handle.abort()
            return self._result(req, failure_reason=pitch.FAIL_LOW_SIGNAL)
        self.get_logger().warn(f"[{tag}] 모든 후보 기각 — {last.summary()}")
        goal_handle.abort()
        return self._result(req, last)

    def destroy_node(self):
        self.recorder.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = AudioAnalyzer()
    executor = MultiThreadedExecutor()   # 녹음 중에도 취소 요청을 받기 위해
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
