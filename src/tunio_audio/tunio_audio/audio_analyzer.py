"""
audio_analyzer — /audio/measure_pitch Action 서버.

요청 시에만 녹음/분석한다. 흐름은 이렇다.

  Goal 수신 -> (녹음 스트림 확인) -> ready Feedback -> 로봇이 줄을 튕김
           -> 줄별 녹음 길이만큼 수집 -> 앞 과도음 잘라냄 -> pitch.analyze -> Result

Result에는 요청의 job_id·string_no·measure_id를 그대로 실어 돌려준다.
늦게 도착한 이전 측정이 현재 결과로 쓰이지 않게 하기 위한 것이다(기획서 7절).
같은 이유로 측정은 한 번에 하나만 받는다. 이미 진행 중이면 새 Goal을 거부한다.
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tunio_audio import pitch
from tunio_audio.recorder import read_wav, Recorder, RecorderError

from tunio_interfaces.action import MeasurePitch

# 줄별 녹음 길이 기본값(초).
#
# 저음현은 한 주기가 길어 짧게 자르면 값이 흔들린다. 여기에 안정화 판정이 겹치는 창을
# 세 개 쓰므로 창 하나가 충분히 길어야 한다. 실제 녹음 10개씩으로 스윕한 결과:
#   1번 줄  0.8초 7/10 · 1.0초 8/10 · 1.2초 7/10 · 1.5초 10/10 · 2.0초 10/10
#   6번 줄  0.8초 1/10 · 1.0초 7/10 · 1.5초 8/10 · 2.0초 9/10
# 1번 줄은 1.5초부터 전부 통과하고 6번 줄은 2.0초가 최선이다.
# 응답은 1번 1.53초 / 6번 2.05초로 목표 2.2초 안에 들어온다.
DEFAULT_RECORD_SEC = {1: 1.5, 2: 1.5, 3: 1.8, 4: 1.8, 5: 2.0, 6: 2.0}

RESPONSE_GOAL_SEC = 2.2     # 정상 목표. 넘으면 경고만 남기고 결과는 그대로 반환한다.


class AudioAnalyzer(Node):
    def __init__(self):
        super().__init__("audio_analyzer")

        self.declare_parameter("device", -1)            # -1이면 시스템 기본 입력
        self.declare_parameter("sample_rate", 44100)
        self.declare_parameter("skip_head_sec", 0.3)    # 발현 직후 과도음 제외
        self.declare_parameter("min_db", -50.0)         # 소리 부족 판정
        # 탐색 범위 안 FFT 피크 돌출 하한. RMS만으로는 방 소음을 못 거른다. pitch.py 주석 참조
        self.declare_parameter("min_prominence_db", pitch.MIN_PROMINENCE_DB)
        # 안정화 판정: 겹치는 창들이 이 폭 안으로 일치해야 채택한다
        self.declare_parameter("max_agreement_cents", pitch.MAX_AGREEMENT_CENTS)
        self.declare_parameter("search_semitones", 2.0)  # 목표 +-2반음
        self.declare_parameter("response_timeout_sec", 10.0)
        # 마이크 대신 WAV를 읽는다 (마이크 없이 흐름 확인용). 빈 문자열이면 마이크를 쓴다.
        # {string_no}를 넣으면 줄 번호로 치환된다. 예: tools/g{string_no}_*.wav
        self.declare_parameter("replay_wav", "")
        for s, sec in DEFAULT_RECORD_SEC.items():
            self.declare_parameter(f"record_sec.s{s}", sec)

        dev = self.get_parameter("device").value
        self.recorder = Recorder(
            sample_rate=self.get_parameter("sample_rate").value,
            device=None if dev is None or dev < 0 else dev,
        )

        self._busy = False
        self._cb_group = ReentrantCallbackGroup()
        self.srv = ActionServer(
            self, MeasurePitch, "/audio/measure_pitch",
            execute_callback=self.on_measure,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=self._cb_group,
        )

        # 첫 측정이 장치 여는 시간을 물지 않도록 미리 열어둔다. 실패해도 측정 때 다시 시도한다.
        try:
            sr = self.recorder.open()
            self.get_logger().info(
                f"입력 장치 '{self.recorder.device_name()}' {sr} Hz 로 열림")
            if sr != self.recorder.requested_rate:
                self.get_logger().warn(
                    f"요청 {self.recorder.requested_rate} Hz 는 지원되지 않아 {sr} Hz 로 열었다. "
                    f"분석은 실제 값으로 한다.")
        except RecorderError as exc:
            self.get_logger().warn(f"입력 장치를 아직 열지 못했다: {exc}")

        self.get_logger().info("audio_analyzer 시작 — /audio/measure_pitch 대기")

    # ---------------- Goal 수락 정책 ----------------
    def on_goal(self, goal_request):
        if goal_request.string_no not in pitch.STRINGS:
            self.get_logger().warn(f"알 수 없는 줄 번호 {goal_request.string_no} — 거부")
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn(
                f"측정 진행 중이라 거부 "
                f"(job {goal_request.job_id} / {goal_request.string_no}번 / "
                f"measure {goal_request.measure_id})")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info("취소 요청 수신")
        return CancelResponse.ACCEPT

    # ---------------- 측정 ----------------
    def _record_sec(self, string_no):
        return float(self.get_parameter(f"record_sec.s{string_no}").value)

    def _result(self, req, analysis=None, fail_reason=""):
        res = MeasurePitch.Result()
        res.job_id = req.job_id
        res.string_no = req.string_no
        res.measure_id = req.measure_id
        if analysis is not None and analysis.ok:
            res.success = True
            res.measured_hz = analysis.hz
            res.fail_reason = ""
        else:
            res.success = False
            res.measured_hz = float("nan")
            res.fail_reason = fail_reason or (
                analysis.fail_reason if analysis is not None else "알 수 없음")
        return res

    def on_measure(self, goal_handle):
        req = goal_handle.request
        tag = f"job {req.job_id} / {req.string_no}번 / measure {req.measure_id}"
        self._busy = True
        t0 = time.monotonic()
        timeout = float(self.get_parameter("response_timeout_sec").value)

        rec_sec = self._record_sec(req.string_no)
        skip = float(self.get_parameter("skip_head_sec").value)
        replay = str(self.get_parameter("replay_wav").value or "")

        try:
            if replay:
                # 마이크 대신 WAV. 준비 Feedback은 형식을 맞추기 위해 그대로 보낸다.
                path = replay.replace("{string_no}", str(req.string_no))
                fb = MeasurePitch.Feedback()
                fb.ready = True
                goal_handle.publish_feedback(fb)
                try:
                    raw, sr = read_wav(path)
                except Exception as exc:
                    self.get_logger().error(f"[{tag}] WAV를 읽지 못했다 {path}: {exc}")
                    return self._result(req, fail_reason=pitch.FAIL_NO_DEVICE)
                raw = raw[:int(rec_sec * sr)]
                self.get_logger().info(
                    f"[{tag}] WAV 재생 모드 — {path} ({sr} Hz, {len(raw) / sr:.2f}초)")
            else:
                # 1) 녹음 준비. 여기서 스트림이 살아 있어야 ready를 보낼 수 있다.
                try:
                    sr = self.recorder.open()
                except RecorderError as exc:
                    self.get_logger().error(f"[{tag}] {exc}")
                    return self._result(req, fail_reason=pitch.FAIL_NO_DEVICE)

                self.recorder.flush()
                fb = MeasurePitch.Feedback()
                fb.ready = True
                goal_handle.publish_feedback(fb)

                # 2) 녹음
                self.get_logger().info(f"[{tag}] 녹음 {rec_sec}초 시작 ({sr} Hz)")
                try:
                    raw = self.recorder.read(
                        rec_sec,
                        abort=lambda: goal_handle.is_cancel_requested,
                        timeout=max(0.1, timeout - (time.monotonic() - t0)),
                    )
                except TimeoutError as exc:
                    self.get_logger().error(f"[{tag}] {exc}")
                    return self._result(req, fail_reason=pitch.FAIL_TIMEOUT)

                if raw is None or goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info(f"[{tag}] 취소됨")
                    return self._result(req, fail_reason=pitch.FAIL_CANCELED)

            # 3) 과도음 구간 제외 후 분석
            sig = raw[int(skip * sr):]
            analysis = pitch.analyze(
                sig, sr, req.string_no,
                min_db=float(self.get_parameter("min_db").value),
                semitones=float(self.get_parameter("search_semitones").value),
                min_prominence_db=float(self.get_parameter("min_prominence_db").value),
                max_agreement_cents=float(
                    self.get_parameter("max_agreement_cents").value),
            )

            elapsed = time.monotonic() - t0
            if elapsed > timeout:
                self.get_logger().error(f"[{tag}] 응답 제한 {timeout}초 초과 ({elapsed:.2f}초)")
                goal_handle.abort()
                return self._result(req, fail_reason=pitch.FAIL_TIMEOUT)

            level = f"{elapsed:.2f}초"
            if elapsed > RESPONSE_GOAL_SEC:
                self.get_logger().warn(
                    f"[{tag}] {analysis.summary()} — 응답 {level} (목표 {RESPONSE_GOAL_SEC}초 초과)")
            else:
                self.get_logger().info(f"[{tag}] {analysis.summary()} — 응답 {level}")

            if analysis.ok:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return self._result(req, analysis)

        except Exception as exc:                      # 어떤 경우에도 Result는 돌려준다
            self.get_logger().error(f"[{tag}] 예외: {exc!r}")
            if goal_handle.is_active:
                goal_handle.abort()
            return self._result(req, fail_reason=f"내부 오류: {exc}")
        finally:
            self._busy = False

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
