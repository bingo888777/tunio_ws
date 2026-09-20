"""
mock_state — 로봇·마이크 없이 관제 웹 화면만 확인하기 위한 모의 발행기. 담당: 정동휘.

기획 11.1 의 "UI는 모의 측정값으로 개발"에 해당한다. 조율 로직이 아니라 화면 검증용이다.
판정식도 수렴 규칙도 진짜가 아니다 — 화면에 나타날 수 있는 경우를 순서대로 보여주는 것이 목적이다.

⚠ task_manager 와 같이 띄우면 안 된다. /tuning/state 를 둘이 발행하고
/tuning/start 등 같은 이름의 Service 를 둘이 제공해 어느 쪽이 응답할지 알 수 없다.
result_store 와 같이 띄울 때는 serve_history:=false 로 /tuning/get_history 를 비켜 준다.

  ros2 run tunio_server mock_state
  ros2 run tunio_server web_node        # 다른 터미널

보여주는 경우:
  - 정상 수렴 (6번, 5번)
  - 측정 불가 한 번 겪고 재측정으로 회복 (2번)
  - 최종 검증에서 틀어져 실패로 재기록 (2번)
  - 조절 횟수를 다 쓰고 실패 (1번) -> 전체 판정이 '일부 성공'이 된다
  - 최종 검증 패스 -> 화음 -> 복귀 -> 완료
  - 일시정지 / 재개 / 취소
"""
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

from tunio_interfaces.msg import JobSummary, StringResult, TuningSettings, TuningState
from tunio_interfaces.srv import CommandTuning, GetHistory, GetSettings, SetSettings

STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# 대상 줄과 목표 주파수 (CLAUDE.md 4절)
TARGETS = ((6, 82.4069), (5, 110.0000), (2, 246.9417), (1, 329.6276))

# 줄마다 어떤 시나리오를 보여줄지.
#   offset        : 첫 측정의 목표 대비 오차(센트)
#   invalid_times : 첫 측정 전에 '측정 불가'를 몇 번 겪을지
#   always_fail   : 조절해도 수렴하지 않게 해서 '조절 횟수 소진'을 보여줄지
#   verify_drift  : 최종 검증에서 측정될 오차(센트). 허용 센트를 넘으면 '검증 시 틀어짐'
# 네 줄에 성공 / 측정 불가 / 검증 틀어짐 / 조절 소진이 한 번씩 나오게 배치했다.
SCENARIO = {
    6: (-38.0, 0, False, -2.0),    # 정상 수렴
    5: (+25.0, 0, False, +3.5),    # 정상 수렴
    2: (-30.0, 1, False, -14.0),   # 측정 불가 한 번 -> 수렴 -> 검증에서 틀어짐
    1: (+45.0, 0, True, +28.0),    # 조절 횟수 소진
}


class Cancelled(Exception):
    pass


class MockState(Node):

    def __init__(self):
        super().__init__('mock_state')
        self.declare_parameter('speed', 1.0)          # 1.0 = 기본 속도, 크게 하면 빨라진다
        self.declare_parameter('serve_history', True)  # result_store 와 같이 띄우면 false

        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(TuningState, '/tuning/state', STATE_QOS)

        for name, handler in (('start', self.on_start), ('pause', self.on_pause),
                              ('resume', self.on_resume), ('cancel', self.on_cancel)):
            self.create_service(CommandTuning, '/tuning/%s' % name, handler, callback_group=cb)
        self.create_service(GetSettings, '/tuning/get_settings', self.on_get_settings,
                            callback_group=cb)
        self.create_service(SetSettings, '/tuning/set_settings', self.on_set_settings,
                            callback_group=cb)
        if bool(self.get_parameter('serve_history').value):
            self.create_service(GetHistory, '/tuning/get_history', self.on_history,
                                callback_group=cb)

        self.settings = TuningSettings()
        self.settings.tolerance_cents = 10.0
        self.settings.max_adjust_count = 5
        self.settings.max_measure_retry = 3
        self.settings.measure_timeout_sec = 10.0
        self.settings.onset_timeout_sec = 10.0
        self.active_settings = TuningSettings()

        self.job_id = ''
        self.phase = TuningState.PHASE_IDLE
        self.current_string = 0
        self.target_hz = 0.0
        self.measured_hz = 0.0
        self.measurement_valid = False
        self.error_hz = 0.0
        self.adjust_count = 0
        self.results = []
        self.overall = TuningState.OVERALL_PENDING

        self._worker = None
        self._pause = threading.Event()
        self._cancel = threading.Event()
        self._history = []          # 모의 이력. 작업이 끝나면 쌓인다

        self.get_logger().warn(
            'mock_state 는 화면 확인용이다. task_manager 와 동시에 띄우지 말 것')
        self._publish()

    # ------------------------------------------------------------------ 상태 발행

    def _publish(self):
        msg = TuningState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.job_id = self.job_id
        msg.phase = self.phase
        msg.current_string = self.current_string
        msg.target_hz = self.target_hz
        msg.measured_hz = self.measured_hz
        msg.measurement_valid = self.measurement_valid
        msg.error_hz = self.error_hz
        msg.adjust_count = self.adjust_count
        msg.results = self.results
        msg.overall_result = self.overall
        msg.active_settings = self.active_settings if self.job_id else self.settings
        self.pub.publish(msg)

    def _sleep(self, sec):
        """진행 중 취소·일시정지를 반영하며 기다린다."""
        speed = max(0.05, float(self.get_parameter('speed').value))
        deadline = time.time() + sec / speed
        while time.time() < deadline:
            if self._cancel.is_set():
                raise Cancelled()
            if self._pause.is_set():
                keep, self.phase = self.phase, TuningState.PHASE_PAUSED
                self._publish()
                while self._pause.is_set():
                    if self._cancel.is_set():
                        raise Cancelled()
                    time.sleep(0.1)
                self.phase = keep
                self._publish()
                deadline = time.time() + sec / speed
            time.sleep(0.05)

    def _set_phase(self, phase, string_number=None):
        self.phase = phase
        if string_number is not None:
            self.current_string = string_number
        self._publish()

    # ------------------------------------------------------------------ 명령

    def _running(self):
        return self._worker is not None and self._worker.is_alive()

    def on_start(self, req, res):
        if self._running():
            res.accepted = False
            res.message = '이미 진행 중이다'
            return res
        self._pause.clear()
        self._cancel.clear()
        self.job_id = time.strftime('%Y%m%d_%H%M%S')
        self.active_settings = TuningSettings()
        for f in ('tolerance_cents', 'max_adjust_count', 'max_measure_retry',
                  'measure_timeout_sec', 'onset_timeout_sec'):
            setattr(self.active_settings, f, getattr(self.settings, f))
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        res.accepted = True
        res.message = '모의 작업 시작 (%s)' % self.job_id
        return res

    def on_pause(self, req, res):
        if not self._running():
            res.accepted = False
            res.message = '진행 중인 작업이 없다'
            return res
        self._pause.set()
        res.accepted = True
        res.message = '일시정지'
        return res

    def on_resume(self, req, res):
        if not self._running() or not self._pause.is_set():
            res.accepted = False
            res.message = '일시정지 상태가 아니다'
            return res
        self._pause.clear()
        res.accepted = True
        res.message = '재개'
        return res

    def on_cancel(self, req, res):
        if not self._running():
            res.accepted = False
            res.message = '진행 중인 작업이 없다'
            return res
        self._cancel.set()
        self._pause.clear()
        res.accepted = True
        res.message = '취소 요청'
        return res

    def on_get_settings(self, req, res):
        res.settings = self.settings
        return res

    def on_set_settings(self, req, res):
        if self._running():
            res.accepted = False
            res.message = '작업 진행 중에는 설정을 바꿀 수 없다'
            return res
        if req.settings.tolerance_cents <= 0.0:
            res.accepted = False
            res.message = '입력 범위 초과: tolerance_cents 는 0보다 커야 한다'
            return res
        self.settings = req.settings
        self._publish()
        res.accepted = True
        res.message = '설정 갱신 (모의)'
        return res

    def on_history(self, req, res):
        if req.job_id:
            for job, strings, settings in self._history:
                if job.job_id == req.job_id:
                    res.found = True
                    res.strings = strings
                    res.settings = settings
                    return res
            res.found = False
            return res
        limit = req.limit or 20
        res.found = bool(self._history)
        res.jobs = [j for j, _s, _t in reversed(self._history)][:limit]
        return res

    # ------------------------------------------------------------------ 모의 진행

    @staticmethod
    def _hz_from_cents(target, cents):
        return target * math.pow(2.0, cents / 1200.0)

    def _new_result(self, n, target):
        r = StringResult()
        r.string_number = n
        r.target_hz = target
        r.result = StringResult.RESULT_PENDING
        r.failure_reason = StringResult.FAIL_NONE
        return r

    def _show_measure(self, target, cents=None, valid=True):
        """측정 한 번을 화면에 보여준다. cents=None 이면 '측정 불가'."""
        self._set_phase(TuningState.PHASE_MEASURE_READY)
        self._sleep(0.4)
        self._set_phase(TuningState.PHASE_PLUCK)
        self._sleep(0.5)
        self._set_phase(TuningState.PHASE_ANALYZE)
        self._sleep(0.8)
        if valid:
            self.measured_hz = self._hz_from_cents(target, cents)
            self.measurement_valid = True
            self.error_hz = self.measured_hz - target
        else:
            self.measured_hz = 0.0
            self.measurement_valid = False
            self.error_hz = 0.0
        self._publish()
        self._sleep(0.5)

    def _tune_string(self, n, target, result):
        offset, invalid_times, always_fail, _drift = SCENARIO[n]
        self.current_string = n
        self.target_hz = target
        self.adjust_count = 0

        for _ in range(invalid_times):
            self._show_measure(target, valid=False)      # 소리 부족 -> 재측정

        self._show_measure(target, cents=offset)
        remaining = offset

        while abs(remaining) > self.active_settings.tolerance_cents:
            if self.adjust_count >= self.active_settings.max_adjust_count:
                result.result = StringResult.RESULT_FAILED
                result.failure_reason = StringResult.FAIL_ADJUST_EXHAUSTED
                break
            self._set_phase(TuningState.PHASE_ADJUST_PEG)
            self._sleep(1.0)
            self.adjust_count += 1
            # 실패 시나리오는 계속 근처에서 맴돌게 둔다(조절 횟수 소진을 보여주기 위함)
            remaining = remaining * (0.9 if always_fail else 0.1)
            self._show_measure(target, cents=remaining)
        else:
            result.result = StringResult.RESULT_SUCCESS

        result.tuned_hz = self.measured_hz
        result.has_tuned_hz = self.measurement_valid
        result.final_error_cents = remaining
        result.adjust_count = self.adjust_count
        self._publish()

    def _verify_pass(self):
        """조절 없이 측정만 다시 한다(기획 2.5). 넥이 당겨져 조금씩 틀어진 값을 보여준다."""
        self._set_phase(TuningState.PHASE_VERIFY)
        for r in self.results:
            self.current_string = r.string_number
            self.target_hz = r.target_hz
            drift = SCENARIO[r.string_number][3]
            self._show_measure(r.target_hz, cents=drift)
            self._set_phase(TuningState.PHASE_VERIFY)
            r.verified_hz = self.measured_hz
            r.has_verified_hz = True
            r.final_error_cents = drift
            if abs(drift) > self.active_settings.tolerance_cents \
                    and r.result == StringResult.RESULT_SUCCESS:
                r.result = StringResult.RESULT_FAILED
                r.failure_reason = StringResult.FAIL_DETUNED_ON_VERIFY
            self._publish()

    def _finish(self):
        ok = sum(1 for r in self.results if r.result == StringResult.RESULT_SUCCESS)
        if ok == len(self.results):
            self.overall = TuningState.OVERALL_SUCCESS
        elif ok == 0:
            self.overall = TuningState.OVERALL_FAILED
        else:
            self.overall = TuningState.OVERALL_PARTIAL

        job = JobSummary()
        job.job_id = self.job_id
        job.finished_at = self.get_clock().now().to_msg()
        job.overall_result = self.overall
        snapshot = TuningSettings()
        for f in ('tolerance_cents', 'max_adjust_count', 'max_measure_retry',
                  'measure_timeout_sec', 'onset_timeout_sec'):
            setattr(snapshot, f, getattr(self.active_settings, f))
        self._history.append((job, list(self.results), snapshot))

    def _run(self):
        # 대상 4줄을 처음부터 RESULT_PENDING 으로 채워 둔다. 끝난 것만 덧붙이면
        # 아직 차례가 오지 않은 줄이 배열에 없어서, 화면이 목표 주파수조차 못 그린다.
        # task_manager 도 같은 방식이다 (항상 4개를 다 보낸다).
        self.results = [self._new_result(n, t) for n, t in TARGETS]
        self.overall = TuningState.OVERALL_PENDING
        self.measurement_valid = False
        self.measured_hz = 0.0
        self._publish()
        try:
            for idx, (n, target) in enumerate(TARGETS):
                r = self.results[idx]
                self._publish()
                self._tune_string(n, target, r)

            self._verify_pass()

            self._set_phase(TuningState.PHASE_STRUM)
            self._sleep(1.5)
            self._set_phase(TuningState.PHASE_RETURN)
            self._sleep(1.5)
            self._finish()
            self.current_string = 0
            self._set_phase(TuningState.PHASE_DONE)
            self.get_logger().info('모의 작업 완료: %s' % self.job_id)
        except Cancelled:
            self.current_string = 0
            self._set_phase(TuningState.PHASE_CANCELED)
            self.get_logger().info('모의 작업 취소: %s (저장하지 않는다)' % self.job_id)


def main():
    rclpy.init()
    node = MockState()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
