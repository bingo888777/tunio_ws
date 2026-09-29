"""
task_manager — 작업 순서, 센트 판정, 재시도, 최종 검증, 상태 발행.

구조: MultiThreadedExecutor + 작업 스레드 1개. 상태기계를 순차 코드로 쓴다.
Action 응답은 executor 스레드가 처리하고, 작업 스레드는 Future 를 Event 로 기다린다.
robot_controller 도 같은 구조라 결이 맞고, "지금 어느 단계냐"가 코드 흐름 그대로 읽힌다.

미확정 항목은 이 파일 상단의 정책 상수에 모아 두고 시작 시 로그로 알린다.
값이 없으면 안 되는 물리 상수(페그 회전량)는 임시값을 쓰지 않고 그 자리에서 멈춘다.
근거: CLAUDE.md 11절, docs/task_manager_구현_브리핑.md
"""
from datetime import datetime
import threading
import time

from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

from tunio_interfaces.action import (AdjustPeg, MeasurePitch, PlayString, PreparePluck,
                                     RegisterPeg, ReturnHome, StrumAll)
from tunio_interfaces.msg import StringResult, TuningSettings, TuningState
from tunio_interfaces.srv import CommandTuning, GetSettings, SaveJob, SetSettings

from tunio_server import peg_calibration as PC
from tunio_server import tuning_logic as L

# 명세 5절: /tuning/state 는 reliable, depth 1, transient local.
# 늦게 켠 UI도 마지막 상태를 즉시 받는다. web_node 쪽 값과 같아야 매칭된다.
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ---------------------------------------------------------------------------
# 미확정 정책 — 사용자 결정 대기 중 (docs/task_manager_구현_브리핑.md B·C절)
# 문서에 답이 없어 임시로 한쪽을 택한 것들이다. 확정값이 아니다.
# 시작할 때 로그로 찍어, 이 가정 위에서 돈다는 걸 눈에 보이게 한다.
# ---------------------------------------------------------------------------

# B1-1. 페그를 돌린 뒤 측정 재시도 횟수를 0으로 되돌릴지.
# 기획 2.4 "측정 재시도와 페그 조절 횟수는 별도로 관리한다" 를 따라 리셋으로 뒀다.
MEASURE_RETRY_RESET_AFTER_ADJUST = True

# 2026-09-22 사용자 결정: 로봇 실패는 측정 재시도로 넘기지 않고 작업 중단.
PLAY_FAILURE_COUNTS_AS_MEASURE_RETRY = False

# 2026-09-22 사용자 결정: 성공 Result 없이 다음 줄의 로봇 명령을 보내지 않는다.
ADJUST_FAILURE_SKIPS_STRING_ONLY = False

# C2-1. 검증 패스에서 '측정 자체가' 실패했을 때 (허용 센트를 벗어난 게 아니라).
# True 면 Phase 1 판정을 유지하고 has_verified_hz=false 로만 남긴다.
# 기획 2.5 는 "허용 센트를 벗어난 줄"만 실패 재기록 대상으로 적고 있다.
KEEP_TUNED_RESULT_ON_VERIFY_MEASURE_FAILURE = True

# C5-1 은 미확정이 아니었다. CLAUDE.md 13절 521~522행이 확정 목록에 적고 있다 —
# "완료 결과 저장 ... 4줄을 완주했으면 PARTIAL·FAILED 도 저장한다(중간에 끊긴 작업만 제외)".
# '중간에 끊긴 작업만 제외'가 곧 기본값 False 다. 그래서 아래 상수 목록에서 뺐다.
#
# 다만 실측 회차에서는 끊긴 작업의 측정값도 남겨야 원인을 본다. 그래서 상수가 아니라
# ROS 파라미터 save_on_error_exit 로 뒀다 (__init__ 에서 선언, 기본 False).
# 실측 때만 런치 인자로 켠다.
#
# ⚠ 기본값을 True 로 올리려면 이슈 #47 을 먼저 볼 것. 두 가지가 걸려 있다.
#   - _finish_cancelled() 에 _finish_error() 로 빠지는 샛길이 있어, True 면
#     '로봇이 취소를 안 받은 취소 작업'이 저장된다. 취소 미저장은 확정 사항이다
#   - 중간에 끊긴 작업은 처리 못 한 줄이 RESULT_PENDING 이라 overall_result() 가
#     OVERALL_PENDING 을 낸다. 실패가 아니라 '판정 전'으로 저장된다

_POLICY_NOTES = (
    ('B1-1 조절 후 측정 재시도 리셋', MEASURE_RETRY_RESET_AFTER_ADJUST),
    ('B2-1 발현 실패를 측정 재시도로 계산', PLAY_FAILURE_COUNTS_AS_MEASURE_RETRY),
    ('B3-1 페그 실패 시 그 줄만 포기', ADJUST_FAILURE_SKIPS_STRING_ONLY),
    ('C2-1 검증 측정 실패 시 조율 판정 유지', KEEP_TUNED_RESULT_ON_VERIFY_MEASURE_FAILURE),
)


class Cancelled(Exception):
    """작업 취소. 단계 경계에서 던져 상태기계를 빠져나온다."""


class Paused(Exception):
    """
    일시정지로 진행 중인 로봇 동작을 세웠다 (2026-09-29 사용자 결정).

    로봇 쪽은 취소와 같은 경로다 — Soft Stop 후 스스로 복구해 홈으로 간다.
    서버는 PAUSED 로 기다렸다가 재개되면 **그 줄을 처음부터** 다시 한다. 멈춘 자리에서
    이어 가는 방식(#106 비동기 이동)은 9/29 실기에서 "목표 미도달" 로 막혀 쓰지 않는다.
    what — 세운 Action 이름. adjust_peg/register_peg 였으면 재개 전에 그 줄을 강제
    재등록한다 (손잡이가 반쯤 돌아간 채라 등록각이 틀렸을 수 있다 — CLAUDE.md 5절).
    """

    def __init__(self, what=''):
        super().__init__(what)
        self.what = what


class Aborted(Exception):
    """더 진행할 수 없는 오류. PHASE_ERROR 로 끝낸다."""


class PickRefillRequired(Aborted):
    """로봇이 빈손 재집기를 소진하고 홈 복귀를 완료했다."""


class Measurement:
    """측정 1회의 결과. MeasurePitch Result 를 서버 관점으로 옮겨 담은 것."""

    def __init__(self, ok, hz=0.0, reason=StringResult.FAIL_NONE):
        self.ok = ok
        self.hz = hz
        self.reason = reason


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')

        # --- 파라미터 ---
        share = self._share_dir()
        self.declare_parameter('peg_calibration_file',
                               str(share / 'config' / 'peg_calibration.yaml'))
        self.declare_parameter('node_wait_sec', 5.0)        # 시작 시 Action 서버 대기
        self.declare_parameter('goal_response_sec', 5.0)    # goal 수락 응답 대기
        self.declare_parameter('ready_wait_sec', 10.0)      # 마이크 ready feedback 대기
        self.declare_parameter('register_timeout_sec', 180.0)  # 줄별 임시값, 실측 후 조정
        self.declare_parameter('prepare_timeout_sec', 30.0)  # 준비 이동 임시 상한, 실측 후 조정
        self.declare_parameter('play_timeout_sec', 30.0)    # 임시값. F3 로봇 실측 후 조정
        self.declare_parameter('adjust_timeout_sec', 120.0)  # 통합 시험용 임시값, 요청 1회 기준
        self.declare_parameter('robot_completion_wait_sec', 120.0)  # 초과 후 종료 확인 임시값
        self.declare_parameter('return_timeout_sec', 120.0)  # 피크 반환·복귀 임시값
        self.declare_parameter('strum_timeout_sec', 60.0)   # 임시값
        self.declare_parameter('measure_slack_sec', 5.0)    # 통신 왕복 여유

        # 줄 하나가 끝난 뒤 다음 줄로 넘어가기 전에 쉬는 시간.
        # 기본 0 — 실제 동작에 없는 대기를 넣지 않는다. 화면 확인용 대역(fake_hardware)
        # 으로 돌릴 때만 값을 준다. 로봇이 실제로 움직이면 줄 사이 이동 시간이 있어
        # 결과를 볼 여유가 생기지만, 대역에는 그 시간이 없어 판정이 순식간에 지나간다.
        self.declare_parameter('string_done_hold_sec', 0.0)
        self.declare_parameter('cancel_wait_sec', 5.0)      # 취소 요청 수용 대기
        # 중간에 끊긴 작업(PHASE_ERROR)을 저장할지. 기본 False 는 확정 사항이다
        # (CLAUDE.md 13절 521~522행). 실측 회차에서만 켠다. 위 C5-1 주석과 이슈 #47 참조
        self.declare_parameter('save_on_error_exit', False)
        self.declare_parameter(
            'strum_string_numbers', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                description='화음 대상 줄. 비우면 로봇의 교시된 전체'))

        # --- 페그 캘리브레이션 (실측 전이면 조절 단계에서 멈춘다) ---
        self.calibration = self._load_calibration()

        # --- 설정 (작업 중에는 변경 거부) ---
        self.settings = TuningSettings()
        self.settings.tolerance_cents = 10.0     # 임시값. 실측 후 확정 (CLAUDE.md 4절)
        self.settings.max_adjust_count = 5
        self.settings.max_measure_retry = 3
        self.settings.measure_timeout_sec = 10.0
        self.settings.onset_timeout_sec = 10.0
        self.active_settings = TuningSettings()

        # --- 작업 상태 ---
        self._lock = threading.Lock()
        self._worker = None
        self._starting = False
        self._pause = threading.Event()
        self._resumed_since_play = threading.Event()
        self._cancel = threading.Event()
        self._active_goals = []
        self._registered_pegs = set()  # 이번 작업에서 성공 Result를 확인한 줄
        self.job_id = ''
        self.error_message = ''
        self._registration_ready = False  # 등록·복귀 성공으로 발급하는 조율 1회 허가
        self.measurement_id = 0
        self.phase = TuningState.PHASE_IDLE
        self._phase_before_pause = TuningState.PHASE_IDLE
        self.current_string = 0
        self.measured_hz = 0.0
        self.measurement_valid = False
        self.adjust_count = 0
        self.results = {}

        # --- 통신 ---
        cb = ReentrantCallbackGroup()
        self.cli_measure = ActionClient(self, MeasurePitch, '/audio/measure_pitch',
                                        callback_group=cb)
        self.cli_home = ActionClient(
            self, ReturnHome, '/robot/return_home', callback_group=cb)
        self.cli_register = ActionClient(
            self, RegisterPeg, '/robot/register_peg', callback_group=cb)
        self.cli_prepare = ActionClient(
            self, PreparePluck, '/robot/prepare_pluck', callback_group=cb)
        self.cli_play = ActionClient(self, PlayString, '/robot/play_string', callback_group=cb)
        self.cli_adjust = ActionClient(self, AdjustPeg, '/robot/adjust_peg', callback_group=cb)
        self.cli_strum = ActionClient(self, StrumAll, '/robot/strum_all', callback_group=cb)
        self.cli_save = self.create_client(SaveJob, '/tuning/save_job', callback_group=cb)

        self.pub_state = self.create_publisher(TuningState, '/tuning/state', STATE_QOS)

        # 명령 서비스. 시작·재등록·일시정지·재개·취소가 CommandTuning 타입을 공유하며
        # 어느 이름으로 불렸는지가 곧 명령이다 (명세 6절).
        #
        # 서비스마다 MutuallyExclusiveCallbackGroup 을 따로 준다. Reentrant 로 묶으면
        # MultiThreadedExecutor 가 같은 요청을 여러 스레드에 물려 콜백이 중복 실행될 수 있다
        # (실제로 이 환경에서 재현됐다). start 가 두 번 돌면 작업 스레드가 둘 생긴다.
        # 그룹을 따로 두면 같은 서비스끼리는 직렬화되고, 서로 다른 서비스는 계속 병행된다
        # (start 가 서버를 기다리는 동안에도 cancel 이 들어와야 한다).
        for srv_type, name, handler in (
                (CommandTuning, '/tuning/start', self.on_start),
                (CommandTuning, '/tuning/reregister_pegs', self.on_reregister_pegs),
                (CommandTuning, '/tuning/pause', self.on_pause),
                (CommandTuning, '/tuning/resume', self.on_resume),
                (CommandTuning, '/tuning/cancel', self.on_cancel),
                (GetSettings, '/tuning/get_settings', self.on_get_settings),
                (SetSettings, '/tuning/set_settings', self.on_set_settings)):
            self.create_service(srv_type, name, handler,
                                callback_group=MutuallyExclusiveCallbackGroup())

        self._reset_results()
        self._publish_state()
        self._log_startup()

    # ------------------------------------------------------------------ 시작 로그

    def _share_dir(self):
        from pathlib import Path
        try:
            from ament_index_python.packages import get_package_share_directory
            return Path(get_package_share_directory('tunio_server'))
        except Exception:
            return Path(__file__).resolve().parent.parent

    def _p(self, name):
        return self.get_parameter(name).value

    def _load_calibration(self):
        path = self.get_parameter('peg_calibration_file').value
        try:
            cal = PC.load(path)
        except PC.CalibrationMissing as exc:
            self.get_logger().warning('페그 캘리브레이션을 읽지 못했다: %s' % exc)
            return PC.unmeasured()
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error('캘리브레이션 파일 오류 (%s): %s' % (path, exc))
            return PC.unmeasured()
        return cal

    def _log_startup(self):
        log = self.get_logger()
        log.info('task_manager 시작. 줄 순서: %s' % L.STRING_ORDER)
        if self.calibration.measured:
            log.info('페그 캘리브레이션 로드 (실측일 %s)' % self.calibration.measured_at)
            for s in L.STRING_ORDER:
                cal = self.calibration.get(s)
                step = L.min_step_cents(cal)
                if step > self.settings.tolerance_cents:
                    log.warning(
                        '%d번 줄 최소 조절 폭 %.1f센트가 허용 센트 %.1f보다 크다. '
                        '원리상 수렴하지 않는다' % (s, step, self.settings.tolerance_cents))
        else:
            log.warning('페그 캘리브레이션 미실측 — 페그 조절이 필요해지면 그 자리에서 멈춘다')
            log.warning('  %s' % self.calibration.missing_reason())
        save_on_error = bool(self._p('save_on_error_exit'))
        if save_on_error:
            log.warning('save_on_error_exit = True — 중간에 끊긴 작업도 저장한다. '
                        '실측 회차용이다. 시연에서는 꺼라 (CLAUDE.md 13절)')
        else:
            log.info('save_on_error_exit = False (확정 기본값). 중간에 끊긴 작업은 저장하지 않는다')
        log.warning('미확정 정책 위에서 동작 중 (확정되면 task_manager.py 상단만 고치면 된다):')
        for name, value in _POLICY_NOTES:
            log.warning('  %s = %s' % (name, value))

    # ------------------------------------------------------------------ 상태 발행

    def _publish_state(self):
        """상태 발행은 전부 여기를 지난다. 주기 발행이 필요해지면 여기만 타이머로 감싼다.

        지금은 변화 시 발행만 한다 (브리핑 D3, 관제 웹과 맞출 항목).
        """
        msg = TuningState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.job_id = self.job_id
        msg.phase = self.phase
        msg.error_message = self.error_message
        msg.current_string = self.current_string
        msg.target_hz = L.TARGETS[self.current_string][1] if self.current_string else 0.0
        msg.measured_hz = self.measured_hz
        msg.measurement_valid = self.measurement_valid
        msg.error_hz = (self.measured_hz - msg.target_hz) if self.measurement_valid else 0.0
        msg.adjust_count = self.adjust_count
        msg.results = [self.results[s] for s in L.STRING_ORDER]
        msg.overall_result = L.overall_result([r.result for r in msg.results])
        msg.active_settings = self.active_settings if self.job_id else self.settings
        msg.robot_notice = self.robot_notice
        self.pub_state.publish(msg)

    def _set_robot_notice(self, message):
        """로봇의 판정을 그대로 관제에 전달한다. 서버에서 폭을 재판정하지 않는다."""
        self.robot_notice = message
        self._publish_state()

    def _set_phase(self, phase, string_number=None):
        self.phase = phase
        changed_string = string_number is not None and string_number != self.current_string
        if changed_string or phase in (TuningState.PHASE_MEASURE_READY, TuningState.PHASE_VERIFY,
                                       TuningState.PHASE_PREPARE_PLUCK):
            # 새 줄/새 측정에 이전 측정값을 붙여 발행하지 않는다.
            # 줄별 확정 결과는 self.results에 그대로 보존한다.
            self.measured_hz = 0.0
            self.measurement_valid = False
        if string_number is not None:
            self.current_string = string_number
        if changed_string:
            rec = self.results.get(string_number)
            self.adjust_count = rec.adjust_count if rec is not None else 0
        self._publish_state()

    def _reset_results(self):
        self.robot_notice = ''
        self.results = {}
        for s in L.STRING_ORDER:
            r = StringResult()
            r.string_number = s
            r.target_hz = L.TARGETS[s][1]
            r.result = StringResult.RESULT_PENDING
            r.failure_reason = StringResult.FAIL_NONE
            self.results[s] = r

    # ------------------------------------------------------------------ 명령 서비스

    def _running(self):
        return self._worker is not None and self._worker.is_alive()

    def on_reregister_pegs(self, req, res):
        return self.on_start(req, res, reregister=True)

    def on_start(self, req, res, reregister=False):
        # 검사와 스레드 생성 사이에 다른 start 가 끼어들면 작업 스레드가 둘 생긴다
        with self._lock:
            if self._running() or self._starting:
                res.accepted = False
                res.message = '이미 진행 중인 작업이 있다'
                return res
            self._starting = True
        try:
            return self._do_start(res, reregister=reregister)
        finally:
            self._starting = False

    def _do_start(self, res, reregister=False):
        if not reregister and not self._registration_ready:
            res.accepted = False
            res.message = '페그 재등록을 먼저 실행하세요. 조율 작업마다 새 등록이 필요합니다.'
            return res
        missing = self._servers_not_ready(registration_only=reregister)
        if missing:
            res.accepted = False
            res.message = '노드 준비 안 됨: %s' % ', '.join(missing)
            return res
        # 조율 접수 시 소비한다. 성공·오류·취소 모두 다음 시작은 재등록이 필요하다.
        # 노드 준비 부족으로 거부된 요청은 위에서 반환하므로 허가를 소비하지 않는다.
        self._registration_ready = False
        self._pause.clear()
        self._resumed_since_play.clear()
        self._cancel.clear()
        self.error_message = ''
        self._registered_pegs.clear()
        self._reset_results()
        self.job_id = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.measurement_id = 0
        self.adjust_count = 0
        self.measured_hz = 0.0
        self.measurement_valid = False
        self.active_settings = TuningSettings()
        for f in ('tolerance_cents', 'max_adjust_count', 'max_measure_retry',
                  'measure_timeout_sec', 'onset_timeout_sec'):
            setattr(self.active_settings, f, getattr(self.settings, f))
        self._set_phase(TuningState.PHASE_REGISTER_PEG if reregister
                        else TuningState.PHASE_PREPARE_PLUCK, L.STRING_ORDER[0])
        target = self._run_reregister if reregister else self._run_job
        self._worker = threading.Thread(target=target, name='tuning', daemon=True)
        self._worker.start()
        res.accepted = True
        res.message = ('페그 재등록 접수 %s — 조율 없이 등록·복귀만 실행' if reregister
                       else '작업 시작 %s — 이번에 등록한 값으로 조율') % self.job_id
        return res

    def _servers_not_ready(self, registration_only=False):
        """서버가 떠 있는지 확인한다. 브리핑 B6 — 존재 확인까지만 한다."""
        wait = float(self._p('node_wait_sec'))
        missing = []
        for name, cli in (('robot/return_home', self.cli_home),
                          ('robot/register_peg', self.cli_register),
                          ('robot/prepare_pluck', self.cli_prepare),
                          ('audio/measure_pitch', self.cli_measure),
                          ('robot/play_string', self.cli_play),
                          ('robot/adjust_peg', self.cli_adjust)):
            if not registration_only and cli is self.cli_register:
                continue
            if registration_only and cli not in (self.cli_home, self.cli_register):
                continue
            if not cli.wait_for_server(timeout_sec=wait):
                missing.append(name)
        return missing

    def on_pause(self, req, res):
        if not self._running():
            res.accepted = False
            res.message = '진행 중인 작업이 없다'
            return res
        self._pause.set()
        # 취소와 같은 경로로 로봇을 즉시 세운다. 로봇은 스스로 복구해 홈으로 간다
        self._request_goal_cancel()
        res.accepted = True
        res.message = '일시정지 — 로봇을 그 자리에서 세운다(접촉이면 살짝 든다). 재개하면 이 줄을 처음부터'
        return res

    def on_resume(self, req, res):
        if not self._running():
            res.accepted = False
            res.message = '진행 중인 작업이 없다'
            return res
        if not self._pause.is_set() or self.phase != TuningState.PHASE_PAUSED:
            res.accepted = False
            res.message = '현재 단계가 끝나 일시정지된 뒤 재개하라'
            return res
        self._resumed_since_play.set()
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
        self._request_goal_cancel()
        res.accepted = True
        res.message = '취소를 요청했습니다. 현재 동작의 종료 응답을 기다립니다.'
        return res

    def on_get_settings(self, req, res):
        res.settings = self.settings
        return res

    def on_set_settings(self, req, res):
        if self._running():
            res.accepted = False
            res.message = '작업 진행 중에는 설정을 바꿀 수 없다'
            return res
        bad = self._settings_violations(req.settings)
        if bad:
            res.accepted = False
            res.message = '입력 범위 초과: %s' % '; '.join(bad)
            return res
        self.settings = req.settings
        self._publish_state()
        res.accepted = True
        res.message = '설정 갱신'
        return res

    def _settings_violations(self, s):
        """
        입력 검증.

        ⚠ 필드별 허용 범위는 미확정이다 (브리핑 D1). 숫자를 지어내지 않기 위해,
        지금은 '이 값이면 작업이 성립하지 않는다'는 것만 막는다.
        상·하한이 정해지면 여기에 추가한다. 관제 웹의 폼 검증과 값을 맞춰야 한다.
        """
        bad = []
        if s.tolerance_cents <= 0.0:
            bad.append('tolerance_cents 는 0보다 커야 한다 (0 이하면 성공이 성립하지 않는다)')
        if s.measure_timeout_sec <= 0.0:
            bad.append('measure_timeout_sec 는 0보다 커야 한다')
        if s.onset_timeout_sec <= 0.0:
            bad.append('onset_timeout_sec 는 0보다 커야 한다')
        # max_adjust_count / max_measure_retry 는 0도 뜻이 있다(조절 없음 / 재시도 없음).
        # uint8 이라 음수는 들어오지 않는다. 상한은 미확정이라 막지 않는다.
        return bad

    # ------------------------------------------------------------------ Action 헬퍼

    def _wait(self, future, timeout, what):
        """작업 스레드에서 Future 를 기다린다. 시간 초과면 None."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self.get_logger().error('%s 응답 없음 (%.1f초)' % (what, timeout))
            return None
        return future.result()

    def _track(self, handle):
        with self._lock:
            self._active_goals.append(handle)

    def _untrack(self, handle):
        with self._lock:
            if handle in self._active_goals:
                self._active_goals.remove(handle)

    def _request_goal_cancel(self):
        """진행 중인 goal 에 취소를 건다 (ICD 6.4).

        robot_controller 에 cancel_callback 이 없으면 로봇이 거부한다.
        거부/무응답은 여기서 로그만 남기고, 작업 스레드가 PHASE_ERROR 로 끝낸다.
        """
        with self._lock:
            handles = list(self._active_goals)
        for h in handles:
            try:
                h.cancel_goal_async()
            except Exception as exc:                              # noqa: BLE001
                self.get_logger().warning('취소 요청 실패: %s' % exc)

    def _run_action(self, client, goal, timeout, what, feedback_cb=None):
        """성공 Result를 확인해야 반환한다. 실패/거부/종료 불명은 작업 중단."""
        send = client.send_goal_async(goal, feedback_callback=feedback_cb)
        handle = self._wait(send, float(self._p('goal_response_sec')), '%s 수락' % what)
        if handle is None:
            raise Aborted('%s 수락 여부 불명 — 후속 동작 중단' % what)
        if not handle.accepted:
            raise Aborted('%s goal 거부 — 후속 동작 중단' % what)
        self._track(handle)
        try:
            # 수락 응답을 기다리는 사이 취소·일시정지가 들어온 경우도 전달한다.
            if self._cancel.is_set() or self._pause.is_set():
                handle.cancel_goal_async()
            future = handle.get_result_async()
            wrapped = self._wait(future, timeout, '%s 결과' % what)
            if wrapped is None:
                # 취소 수락만으로 실제 팔 정지가 보장되지 않는다. 결과를 확인해야
                # 다음 동작을 요청할 수 있다. 추가 대기 중 받은 성공도 그대로 반영한다.
                self.get_logger().warning('%s 시간 초과 — 로봇 종료 응답을 기다린다' % what)
                wrapped = self._wait(future, float(self._p('robot_completion_wait_sec')),
                                     '%s 종료 확인' % what)
                if wrapped is None:
                    raise Aborted('%s 동작 종료 불명 — 후속 동작 중단' % what)
        finally:
            self._untrack(handle)
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            if self._cancel.is_set():
                raise Cancelled()
            if self._pause.is_set():
                raise Paused(what)
        # 취소가 아닌 ABORTED 중에서 피크 보충 대기를 가린다 (#88). CANCELED 와
        # 상태가 갈라져 서로 배타적이고, 사용자 취소가 언제나 우선이다.
        if (what in ('prepare_pluck', 'play_string')
                and wrapped.status == GoalStatus.STATUS_ABORTED
                and not wrapped.result.success
                and getattr(wrapped.result, 'pick_refill_required', False)):
            raise PickRefillRequired(wrapped.result.message)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
            raise Aborted('%s 실패(status=%s): %s — 후속 동작 중단'
                          % (what, wrapped.status, wrapped.result.message))
        if self._cancel.is_set():
            # 현재 동작은 끝났지만 사용자가 요청한 뒤에는 다음 동작을 시작하지 않는다.
            raise Cancelled()
        self.get_logger().info('%s 성공 Result 확인' % what)
        return wrapped.result

    # ------------------------------------------------------------------ 단계 경계

    def _gate(self):
        """
        일시정지·취소를 확인한다. 단계 경계에서만 부른다.

        브리핑 B4-1 의 '단계 경계 일시정지' 다. 2026-09-29 부터 on_pause 가 진행 중인
        로봇 goal 에 취소도 걸어서, 동작 중이면 로봇이 즉시 서고(복구·홈) _run_action 이
        Paused 를 던진다. 여기는 그 뒤(또는 동작이 없을 때) 실제로 멈춰 기다리는 자리다.
        """
        if self._cancel.is_set():
            raise Cancelled()
        if not self._pause.is_set():
            return
        resume_to = self.phase
        self._set_phase(TuningState.PHASE_PAUSED)
        while self._pause.is_set() and not self._cancel.is_set():
            time.sleep(0.05)
        if self._cancel.is_set():
            raise Cancelled()
        self._set_phase(resume_to)

    def _sleep_gated(self, seconds):
        """
        취소를 존중하며 기다린다.

        통째로 time.sleep 하면 그동안 취소 요청이 들어와도 반응하지 않는다.
        잘게 나눠 자면서 확인한다.
        """
        end = time.monotonic() + float(seconds)
        while time.monotonic() < end:
            if self._cancel.is_set():
                raise Cancelled()
            time.sleep(min(0.1, end - time.monotonic()))

    # ------------------------------------------------------------------ 측정 1회

    def _measure_once(self, string_number):
        """보충 대기는 측정 실패 횟수에 넣지 않고, 재개 후 새 측정을 준비한다."""
        while True:
            try:
                return self._measure_attempt(string_number)
            except PickRefillRequired as exc:
                self.measurement_valid = False
                self.measured_hz = 0.0
                self._set_robot_notice(str(exc))
                self._pause.set()
                self._gate()  # 기존 웹 재개/취소 서비스를 그대로 사용한다
                self._set_robot_notice('')

    def _measure_attempt(self, string_number):
        """
        ICD 6.1 한 사이클. ready feedback 을 받은 뒤에 발현을 시킨다.

        PlayString Result 와 MeasurePitch Result 의 도착 순서는 보장되지 않는다.
        둘 다 받은 뒤에 판정한다.
        """
        # #89: 피크 집기·이동을 완료한 뒤 마이크 타이머를 시작한다.
        self._gate()
        self._set_phase(TuningState.PHASE_PREPARE_PLUCK, string_number)
        prepare_goal = PreparePluck.Goal(
            string_number=string_number, resumed=self._resumed_since_play.is_set())
        self._resumed_since_play.clear()
        try:
            prepared = self._run_action(
                self.cli_prepare, prepare_goal,
                float(self._p('prepare_timeout_sec')), 'prepare_pluck',
                feedback_cb=lambda msg: self._set_robot_notice(msg.feedback.warning))
        except Aborted as exc:
            self._set_robot_notice(str(exc))
            raise
        if prepared is None or not prepared.success or not prepared.preparation_id:
            raise Aborted('발현 준비 성공 식별자가 없다 — 측정·발현 요청 차단')
        if prepared.message:
            self._set_robot_notice(prepared.message)
        self._gate()  # 준비 중 취소/일시정지: 마이크와 발현을 시작하지 않는다
        target_hz = L.TARGETS[string_number][1]
        self.measurement_id += 1
        mid = self.measurement_id

        goal = MeasurePitch.Goal()
        goal.job_id = self.job_id
        goal.string_number = string_number
        goal.measurement_id = mid
        goal.target_hz = target_hz
        goal.timeout_sec = self.active_settings.measure_timeout_sec
        goal.onset_timeout_sec = self.active_settings.onset_timeout_sec

        ready = threading.Event()

        def on_feedback(msg):
            if msg.feedback.ready:
                ready.set()

        self._set_phase(TuningState.PHASE_MEASURE_READY, string_number)
        send = self.cli_measure.send_goal_async(goal, feedback_callback=on_feedback)
        handle = self._wait(send, float(self._p('goal_response_sec')), 'measure_pitch 수락')
        if handle is None or not handle.accepted:
            return Measurement(False, reason=StringResult.FAIL_TIMEOUT)
        self._track(handle)
        result_future = handle.get_result_async()

        try:
            if not ready.wait(float(self._p('ready_wait_sec'))):
                self.get_logger().error('마이크 ready feedback 이 오지 않았다')
                handle.cancel_goal_async()
                return Measurement(False, reason=StringResult.FAIL_TIMEOUT)

            if self._cancel.is_set():
                handle.cancel_goal_async()
                raise Cancelled()
            if self._pause.is_set():
                handle.cancel_goal_async()
                raise Paused('measure_pitch')

            # 발현. 마이크가 준비된 뒤에만 튕긴다 (명세 5.2)
            self._set_phase(TuningState.PHASE_PLUCK, string_number)
            # 서버는 재개 사실만 전달한다. 피크 판정·알람·복귀는 로봇 내부 책임이다.
            play_goal = PlayString.Goal(
                string_number=string_number, resumed=self._resumed_since_play.is_set(),
                preparation_id=prepared.preparation_id)
            self._resumed_since_play.clear()
            robot_warnings = set()

            def on_robot_feedback(msg):
                warning = msg.feedback.warning
                if warning and warning not in robot_warnings:
                    robot_warnings.add(warning)
                    self.get_logger().warning('로봇 알림: %s' % warning)
                    self._set_robot_notice(warning)

            try:
                play = self._run_action(self.cli_play, play_goal,
                                        float(self._p('play_timeout_sec')), 'play_string',
                                        feedback_cb=on_robot_feedback)
            except PickRefillRequired:
                # 이전 녹음이 끝났는지 확인한 뒤에만 보충 대기로 전환한다.
                handle.cancel_goal_async()
                finished = self._wait(result_future, float(self._p('cancel_wait_sec')),
                                      '피크 보충 전 녹음 종료')
                if finished is None:
                    raise Aborted('녹음 종료 불명 — 피크 보충 후 자동 재개 차단')
                raise
            except Paused:
                handle.cancel_goal_async()   # 발현이 섰으면 그 소리를 재면 안 된다
                raise
            except Aborted as exc:
                # #83은 실패 Result도 예외로 전달한다. 피드백 유실 시에도 사유를 표시하고
                # 아직 기다리는 마이크를 취소한 뒤 후속 로봇 명령 없이 종료한다.
                self._set_robot_notice(str(exc))
                handle.cancel_goal_async()
                raise
            if play.message and play.message not in robot_warnings:
                self._set_robot_notice(play.message)
                # 성공 결과의 부가 알림은 표시만 한다.
                self.get_logger().warning('로봇 동작 완료(경고): %s' % play.message)

            # 측정 결과 대기. 마이크가 onset 부터 timeout_sec 을 세므로
            # 서버는 onset 대기 + 측정 + 통신 여유만큼 기다린다
            self._set_phase(TuningState.PHASE_ANALYZE, string_number)
            budget = (self.active_settings.onset_timeout_sec
                      + self.active_settings.measure_timeout_sec
                      + float(self._p('measure_slack_sec')))
            wrapped = self._wait(result_future, budget, 'measure_pitch 결과')
        finally:
            self._untrack(handle)

        if wrapped is None:
            return Measurement(False, reason=StringResult.FAIL_TIMEOUT)

        res = wrapped.result
        # 늦게 도착한 이전 결과를 걸러낸다 (명세 7절)
        if (res.job_id != self.job_id or res.string_number != string_number
                or res.measurement_id != mid):
            self.get_logger().warning(
                '식별자가 다른 측정 결과를 버린다: %s/%d/%d (기대 %s/%d/%d)'
                % (res.job_id, res.string_number, res.measurement_id,
                   self.job_id, string_number, mid))
            return Measurement(False, reason=StringResult.FAIL_TIMEOUT)

        if not res.success:
            return Measurement(False, reason=res.failure_reason)
        return Measurement(True, res.frequency_hz)

    def _measure_with_retry(self, string_number, budget):
        """
        유효 측정 1건을 얻을 때까지 재측정한다. budget 은 남은 재시도 횟수.

        페그는 건드리지 않는다 (ICD 6.2). 반환: (Measurement, 쓴 횟수)
        """
        used = 0
        last = Measurement(False, reason=StringResult.FAIL_LOW_SIGNAL)
        while True:
            self._gate()
            m = self._measure_once(string_number)
            self.measured_hz = m.hz
            self.measurement_valid = m.ok
            self._publish_state()
            if m.ok:
                return m, used
            last = m
            if (m.reason == StringResult.FAIL_ROBOT_ERROR
                    and not PLAY_FAILURE_COUNTS_AS_MEASURE_RETRY):
                return m, used
            if used >= budget:
                return last, used
            used += 1
            self.get_logger().warning(
                '%d번 줄 재측정 %d/%d (사유 %d)' % (string_number, used, budget, m.reason))

    # ------------------------------------------------------------------ 줄 하나 조율

    def _tune_string(self, string_number):
        rec = self.results[string_number]
        target_hz = rec.target_hz
        self.adjust_count = 0
        # 현재 줄 전환은 _set_phase에서 측정값 초기화와 함께 수행한다.
        retry_budget = self.active_settings.max_measure_retry

        while True:
            self._gate()
            m, used = self._measure_with_retry(string_number, retry_budget)
            if not MEASURE_RETRY_RESET_AFTER_ADJUST:
                retry_budget = max(0, retry_budget - used)

            if not m.ok:
                rec.result = StringResult.RESULT_FAILED
                rec.failure_reason = m.reason
                rec.adjust_count = self.adjust_count
                self._publish_state()
                return

            rec.tuned_hz = m.hz
            rec.has_tuned_hz = True
            rec.final_error_cents = L.error_cents(m.hz, target_hz)
            rec.adjust_count = self.adjust_count

            if abs(rec.final_error_cents) <= self.active_settings.tolerance_cents:
                rec.result = StringResult.RESULT_SUCCESS
                rec.failure_reason = StringResult.FAIL_NONE
                self._publish_state()
                self.get_logger().info(
                    '%d번 줄 조율 성공 %.2fHz (%+.1f센트, 조절 %d회)'
                    % (string_number, m.hz, rec.final_error_cents, self.adjust_count))
                return

            if self.adjust_count >= self.active_settings.max_adjust_count:
                rec.result = StringResult.RESULT_FAILED
                rec.failure_reason = StringResult.FAIL_ADJUST_EXHAUSTED
                self._publish_state()
                return

            self._gate()
            if not self._adjust_peg(string_number, rec.final_error_cents):
                rec.result = StringResult.RESULT_FAILED
                rec.failure_reason = StringResult.FAIL_ROBOT_ERROR
                rec.adjust_count = self.adjust_count
                self._publish_state()
                if ADJUST_FAILURE_SKIPS_STRING_ONLY:
                    return
                raise Aborted('%d번 줄 페그 조절 실패' % string_number)

            self.adjust_count += 1
            rec.adjust_count = self.adjust_count
            # 조절이 끝난 직후에 알린다. 여기서 발행하지 않으면 다음 측정이 끝날 때까지
            # 화면의 조절 횟수가 그대로라, 측정값과 조절 횟수가 한꺼번에 바뀌는 것처럼
            # 보이고 페그를 돌리는 동안은 화면이 멈춘 것처럼 보인다.
            self._publish_state()
            if MEASURE_RETRY_RESET_AFTER_ADJUST:
                retry_budget = self.active_settings.max_measure_retry

    def _adjust_peg(self, string_number, err_cents):
        """
        페그 조절 1회. 백래시 마무리 때문에 goal 이 2번 나갈 수 있다 (CLAUDE.md 3.2).

        캘리브레이션 실측 전이면 여기서 Aborted 를 던진다. 임시 계수로 로봇을 돌리지 않는다.
        """
        try:
            cal = self.calibration.get(string_number)
        except PC.CalibrationMissing as exc:
            raise Aborted('페그 회전량을 계산할 수 없다: %s' % exc)

        self._set_phase(TuningState.PHASE_ADJUST_PEG, string_number)
        for direction, angle_deg in L.adjust_plan(err_cents, cal):
            self._gate()
            goal = AdjustPeg.Goal()
            goal.string_number = string_number
            goal.direction = int(direction)
            goal.angle_deg = float(angle_deg)
            self.get_logger().info(
                '%d번 줄 페그 %s %.2f도'
                % (string_number, '조임' if direction > 0 else '풀림', angle_deg))
            res = self._run_action(self.cli_adjust, goal,
                                   float(self._p('adjust_timeout_sec')), 'adjust_peg')
            if res is None or not res.success:
                why = res.message if res is not None else '응답 없음'
                self.get_logger().error('페그 조절 실패: %s' % why)
                return False
        return True

    # ------------------------------------------------------------------ Phase 2

    def _verify_pass(self, numbers=None):
        """
        최종 검증 패스. 페그를 건드리지 않고 다시 측정만 한다 (기획 2.5).

        한 줄을 조이면 넥이 당겨져 먼저 맞춘 줄이 틀어진다. 전체 판정은 이 결과가 기준이다.
        Phase 1 성공 줄만 검증한다. 실패 줄은 최초 실패 사유와 측정값을 보존한다.
        """
        for s in (L.STRING_ORDER if numbers is None else numbers):
            self._gate()
            rec = self.results[s]
            if rec.result != StringResult.RESULT_SUCCESS:
                self.get_logger().info(
                    '%d번 줄 검증 제외 — 1차 성공 아님 (사유 %d)' % (s, rec.failure_reason))
                continue
            self._set_phase(TuningState.PHASE_VERIFY, s)
            m, _ = self._measure_with_retry(s, self.active_settings.max_measure_retry)

            if not m.ok:
                if KEEP_TUNED_RESULT_ON_VERIFY_MEASURE_FAILURE and rec.has_tuned_hz:
                    self.get_logger().warning(
                        '%d번 줄 검증 측정 실패(사유 %d). 조율 판정을 유지한다' % (s, m.reason))
                else:
                    rec.result = StringResult.RESULT_FAILED
                    rec.failure_reason = m.reason
                self._publish_state()
                continue

            rec.verified_hz = m.hz
            rec.has_verified_hz = True
            rec.final_error_cents = L.error_cents(m.hz, rec.target_hz)
            if abs(rec.final_error_cents) <= self.active_settings.tolerance_cents:
                rec.result = StringResult.RESULT_SUCCESS
                rec.failure_reason = StringResult.FAIL_NONE
            else:
                rec.result = StringResult.RESULT_FAILED
                rec.failure_reason = StringResult.FAIL_DETUNED_ON_VERIFY
                self.get_logger().warning(
                    '%d번 줄 검증에서 틀어짐 %.2fHz (%+.1f센트)'
                    % (s, m.hz, rec.final_error_cents))
            self._publish_state()

    def _strum(self):
        """완료 알림용 화음. 측정하지 않는다 (기획 2.6).

        대상 줄은 파라미터로 받는다. 비우면 로봇의 교시된 전체 줄이다 (브리핑 D4, 미확정).
        """
        self._gate()
        self._set_phase(TuningState.PHASE_STRUM, 0)
        goal = StrumAll.Goal()
        goal.string_numbers = [int(n) for n in (self._p('strum_string_numbers') or [])]
        self._run_action(self.cli_strum, goal,
                         float(self._p('strum_timeout_sec')), 'strum_all')

    def _return_home(self):
        """피크 반환과 복귀는 미구현 화음의 성공 여부와 분리한다."""
        self._gate()
        self._set_phase(TuningState.PHASE_RETURN, 0)
        res = self._run_action(self.cli_home, ReturnHome.Goal(),
                               float(self._p('return_timeout_sec')), 'return_home')
        if res is None or not res.success:
            raise Aborted('피크 반환·대기 복귀 실패')

    # ------------------------------------------------------------------ 저장

    def _save(self):
        codes = [self.results[s].result for s in L.STRING_ORDER]
        overall = L.overall_result(codes)
        if not self.cli_save.wait_for_service(timeout_sec=float(self._p('node_wait_sec'))):
            self.get_logger().error('result_store 가 없어 저장하지 못했다')
            return
        req = SaveJob.Request()
        req.job_id = self.job_id
        req.finished_at = self.get_clock().now().to_msg()
        req.overall_result = overall
        req.strings = [self.results[s] for s in L.STRING_ORDER]
        req.settings = self.active_settings
        res = self._wait(self.cli_save.call_async(req),
                         float(self._p('node_wait_sec')), 'save_job')
        if res is None or not res.saved:
            why = res.message if res is not None else '응답 없음'
            self.get_logger().error('저장 실패: %s' % why)
        else:
            self.get_logger().info('작업 %s 저장 완료' % self.job_id)

    # ------------------------------------------------------------------ 상태기계 본체

    def _register_pegs(self, numbers=None, force=False):
        numbers = tuple(L.STRING_ORDER if numbers is None else numbers)
        if len(numbers) == len(L.STRING_ORDER):
            self._registered_pegs = set()
        for no in numbers:
            f = force
            while True:
                self._gate()
                self._set_phase(TuningState.PHASE_REGISTER_PEG, no)
                self.get_logger().info('%d번 페그 각 등록 요청 — 완료 전 발현하지 않는다' % no)
                try:
                    result = self._run_action(
                        self.cli_register, RegisterPeg.Goal(string_number=no, force=f),
                        float(self._p('register_timeout_sec')), 'register_peg')
                    break
                except Paused:
                    # 잡다 말았으니 그 줄은 처음부터, 그리고 이전 각도를 믿지 않고 강제로
                    self.get_logger().warning(
                        '%d번 페그 등록 중 일시정지 — 로봇이 그 자리에서 선다. 재개하면 이 줄부터 다시' % no)
                    self._gate()
                    f = True
            if result is None or not result.success:
                why = result.message if result is not None else '응답 없음'
                raise Aborted(f'{no}번 페그 초기 등록 실패: {why}')
            self.get_logger().info('%d번 페그 각 등록 확인 완료' % no)
        self._gate()  # 등록 중 취소됐다면 측정을 시작하지 않는다
        self._registered_pegs.update(numbers)  # 전부 성공한 뒤에만 후속 명령을 허용한다

    def _run_reregister(self):
        """재등록만 실행한다. 성공·실패 모두 조율 결과로 저장하지 않는다."""
        try:
            self._register_pegs(force=True)
            self._return_home()
            self._gate()  # 복귀 중 취소 요청까지 확인한 뒤에만 다음 조율을 허용한다
            self._registration_ready = True
            self.job_id = ''
            self._set_phase(TuningState.PHASE_IDLE, 0)
            self.get_logger().info('페그 재등록·복귀 완료')
        except Cancelled:
            self._home_after_stop()
            self._set_phase(TuningState.PHASE_CANCELED, 0)
        except Exception as exc:                          # noqa: BLE001
            self.error_message = str(exc)
            self.get_logger().error('페그 재등록 중단: %s' % exc)
            self._set_phase(TuningState.PHASE_ERROR, 0)

    def _run_job(self):
        self.get_logger().info('작업 %s 시작' % self.job_id)
        try:
            self.get_logger().info('기존 페그 등록값으로 조율 시작 — 자동 등록 없음')
            hold = float(self._p('string_done_hold_sec'))
            for s in L.STRING_ORDER:                      # Phase 1
                while True:
                    self._gate()
                    try:
                        self._tune_string(s)
                        break
                    except Paused as p:
                        self._hold_after_pause(s, p.what)
                # 판정 결과를 화면이 보여 줄 여유. 기본 0 이라 실제 동작에는 영향이 없다.
                if hold > 0:
                    self._sleep_gated(hold)

            while True:                                   # Phase 2
                try:
                    self._verify_pass()
                    break
                except Paused as p:
                    self._hold_after_pause(0, p.what)     # 검증은 측정만이라 통째로 다시
            # #84: 화음은 미구현이라 실행 제외. 구현·실기 검증 후 아래 호출 복구.
            # self._strum()
            while True:
                try:
                    self._return_home()
                    break
                except Paused as p:
                    self._hold_after_pause(0, p.what)

            self._set_phase(TuningState.PHASE_DONE, 0)
            self._save()

        except Cancelled:
            self._finish_cancelled()
        except Aborted as exc:
            self.error_message = str(exc)
            self.get_logger().error('작업 중단: %s' % exc)
            self._finish_error()
        except Exception as exc:                          # noqa: BLE001
            self.error_message = str(exc) or type(exc).__name__
            self.get_logger().error('예상치 못한 오류로 중단: %r' % exc)
            self._finish_error()

    def _hold_after_pause(self, string_number, what):
        """
        일시정지로 로봇을 세운 뒤 PAUSED 로 기다린다.

        재개되면 호출자가 그 단계를 처음부터 다시 돈다. 취소면 _gate 가 Cancelled 를
        던진다. adjust_peg / register_peg 를 세웠으면 그 줄을 강제 재등록한다 — 손잡이를 물고
        돌리다 멈춰서 등록각이 실제와 어긋났을 수 있고, 그대로 물면 물리면서 페그를
        교시 각으로 되돌려 버린다(조용히 틀린다, CLAUDE.md 5절). 발현 쪽(prepare_pluck ·
        play_string · measure_pitch)은 페그를 안 건드려 재등록이 필요 없다.
        """
        where = ('%d번 줄 ' % string_number) if string_number else ''
        self.get_logger().warning(
            '%s%s 중 일시정지 — 로봇이 그 자리에서 선다. 재개하면 처음부터 다시' % (where, what))
        self._gate()
        if what in ('adjust_peg', 'register_peg') and string_number:
            self.get_logger().info('%d번 페그를 물고 있다 멈췄다 — 강제 재등록 후 재개' % string_number)
            self._register_pegs([string_number], force=True)

    def _finish_cancelled(self):
        """취소 종료. DB 에 저장하지 않고 자동 복귀도 하지 않는다 (기획 2.7)."""
        with self._lock:
            pending = list(self._active_goals)
        if pending:
            # 로봇이 취소를 받지 않으면 goal 이 남는다. 허브 지시대로 오류로 끝낸다
            deadline = time.time() + float(self._p('cancel_wait_sec'))
            while time.time() < deadline:
                with self._lock:
                    if not self._active_goals:
                        break
                time.sleep(0.05)
            with self._lock:
                stuck = list(self._active_goals)
            if stuck:
                self.error_message = '취소 후에도 로봇 동작 종료를 확인하지 못했습니다.'
                self.get_logger().error(
                    '취소했는데 goal %d개가 남아 있다. 로봇이 취소를 받지 않는 것으로 보인다'
                    % len(stuck))
                self._finish_error()
                return
        for rec in self.results.values():
            if rec.result == StringResult.RESULT_PENDING:
                rec.result = StringResult.RESULT_FAILED
                rec.failure_reason = StringResult.FAIL_CANCELED
        self._home_after_stop()
        self._set_phase(TuningState.PHASE_CANCELED, 0)
        self.get_logger().info('작업 취소. 저장하지 않는다')

    def _home_after_stop(self):
        """
        취소로 로봇을 세운 뒤 홈으로 보낸다 (2026-09-29 사용자 결정).

        로봇은 정지·접촉 풀기·상승까지만 하고 그 자리에서 기다린다 — 일시정지는 거기서
        재개하므로 로봇이 홈에 가면 안 되고, 취소만 서버가 ReturnHome 으로 보낸다.
        ReturnHome 은 피크를 쥐고 있으면 거치대에 반환하고 대기 자세로 간다.

        _cancel 을 여기서 내린다. 작업은 이미 끝났고 이 goal 만은 취소 대상이 아니다.
        보내는 중에 또 취소를 누르면 그 goal 이 세워지고(로봇은 다시 그 자리 대기)
        여기서 사유만 남기고 CANCELED 로 끝낸다 — 다시 홈을 시도하지 않는다.
        """
        self._cancel.clear()
        self._pause.clear()
        try:
            self._return_home()
        except (Cancelled, Aborted) as exc:
            self.error_message = '취소 후 복귀 실패: %s' % (exc or '사용자가 복귀도 취소')
            self.get_logger().error(self.error_message)
        except Exception as exc:                          # noqa: BLE001
            self.error_message = '취소 후 복귀 실패: %r' % exc
            self.get_logger().error(self.error_message)

    def _finish_error(self):
        self._set_phase(TuningState.PHASE_ERROR, 0)
        if bool(self._p('save_on_error_exit')):
            self.get_logger().warning(
                '오류 종료 — save_on_error_exit 가 켜져 있어 저장한다. '
                '처리 못 한 줄이 있으면 전체 판정은 OVERALL_PENDING 으로 들어간다')
            self._save()
        else:
            self.get_logger().warning(
                '오류 종료 — 저장하지 않는다 (중간에 끊긴 작업. CLAUDE.md 13절). '
                '이 회차의 측정값을 남기려면 save_on_error_exit:=true')


def main():
    rclpy.init()
    node = TaskManager()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
