"""
fake_hardware — 로봇·마이크 없이 서버를 통째로 돌려 보기 위한 하드웨어 대역. 담당: 정동휘.

robot_controller 와 audio_analyzer 를 대신해 일곱 Action 을 제공한다. 그 위에서
**진짜 task_manager 와 진짜 result_store 가 돈다.** 조율 판정·재시도·백래시 결정·
검증 패스·저장이 전부 실제 코드로 실행된다. 기획 11.1 의 "모의 결과로 서버 흐름 테스트".

  ros2 run tunio_server fake_hardware
  ros2 run tunio_server result_store      # 다른 터미널
  ros2 run tunio_server task_manager      # 또 다른 터미널
  ros2 service call /tuning/start tunio_interfaces/srv/CommandTuning

⚠ mock_state 와 혼동하지 말 것. 방향이 반대다.
  - mock_state    : task_manager 를 대체한다. 위에서 상태를 흉내 낸다. 화면 확인용
  - fake_hardware : 로봇·마이크를 대체한다. 아래를 받친다. 서버 로직 검증용
  둘 다 대역이지만 겹치지 않는다. 하나로 합칠 수 없다.

⚠ 진짜 robot_controller / audio_analyzer 와 같이 띄우면 안 된다.
  같은 이름의 Action 을 둘이 제공하면 어느 쪽이 goal 을 받을지 알 수 없다.

⚠ 기본 모드(on_target / detuned / fail)에서는 페그를 돌려도 음정이 변하지 않는다.
  각도-센트 실측값이 없어서, 수렴하는 척하려면 가짜 상수를 넣어야 하기 때문이다.
  그러면 나중에 진짜로 수렴하지 않을 때 원인이 코드인지 상수인지 구분되지 않는다.

mode=sim — 화면 확인 전용 수렴 시뮬레이션
  조율 과정(조절 횟수가 늘고 오차가 줄어드는 것)을 화면에서 봐야 할 때만 쓴다.
  줄마다 이탈값을 들고 있다가 adjust_peg 를 받으면 그만큼 움직이고, 한 줄을 조이면
  다른 줄이 조금씩 끌려간다(넥 장력 — 기획 2.5 가 최종 검증 패스를 두는 이유).

  ⚠ 여기 쓰는 각도-센트 상수는 실측값이 아니라 **지어낸 숫자**다.
    이 모드에서 수렴하는 것은 조율 성능의 근거가 되지 않는다. 화면이 제대로
    그려지는지만 본다. 실측이 끝나면 sim_cents_per_deg 를 실제 값으로 바꾼다.

  task_manager 가 페그 각도를 계산하려면 캘리브레이션이 measured: true 여야 하므로
  전용 파일을 같이 쓴다. 이것도 지어낸 값이다.

    ros2 run tunio_server fake_hardware --ros-args -p mode:=sim
    ros2 run tunio_server task_manager --ros-args \
        -p peg_calibration_file:=<share>/config/peg_calibration.sim.yaml
"""
import threading
import uuid

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tunio_interfaces.action import (AdjustPeg, MeasurePitch, PlayString, PreparePluck,
                                     RegisterPeg, ReturnHome, WipeStrings)
from tunio_interfaces.srv import SaveJob

# mode 파라미터 값
MODE_ON_TARGET = 'on_target'   # 항상 목표 주파수를 낸다 -> 4줄 모두 조절 없이 성공
MODE_DETUNED = 'detuned'       # 항상 detune_cents 만큼 벗어난다 -> 페그 조절 경로로 간다
MODE_FAIL = 'fail'             # 항상 측정 실패 -> 재측정 소진 경로로 간다
MODE_SIM = 'sim'               # 페그를 반영해 수렴한다. 화면 확인 전용 (상단 주석 참조)
MODES = (MODE_ON_TARGET, MODE_DETUNED, MODE_FAIL, MODE_SIM)


class FakeHardware(Node):

    def __init__(self):
        super().__init__('fake_hardware')

        self.declare_parameter('mode', MODE_ON_TARGET)
        self.declare_parameter('detune_cents', -30.0)      # mode=detuned 일 때의 오차
        self.declare_parameter('fail_reason', 1)           # mode=fail 일 때. 1=소리 부족
        # 지연 기본값은 실제 동작에 가깝게 잡는다. 0.1초대로 두면 화면이 순식간에
        # 지나가 단계 전환과 화면 갱신을 눈으로 확인할 수 없다. 빠르게 돌려 결과만
        # 볼 때는 전부 0.05 정도로 낮춰 주면 된다.
        #
        # measure_delay_sec 2.2 만 근거가 있는 값이다 (기획 2.3 — 정상 응답 목표.
        # 저음현 녹음 2.0초 + 장치·분석 시간). 나머지 셋은 로봇 실측 전 추정값이라
        # 교시가 끝나면 실제 시간으로 고쳐야 한다.
        self.declare_parameter('ready_delay_sec', 0.3)     # 마이크가 귀를 여는 시간
        self.declare_parameter('measure_delay_sec', 2.2)   # 녹음+분석 (기획 2.3)
        self.declare_parameter('prepare_delay_sec', 0.0)  # 준비 시간 재현용
        self._prepared_pluck = None
        self.declare_parameter('play_delay_sec', 3.0)      # 피크 집기+접근+발현+후퇴 (추정)
        self.declare_parameter('adjust_delay_sec', 4.0)    # 피크 반환+페그 파지+회전+해제 (추정)
        self.declare_parameter('wipe_delay_sec', 3.0)       # 넥 닦기+대기 자세 복귀 (추정, 옛 strum_delay_sec)
        # result_store 를 같이 띄우면 그쪽이 /tuning/save_job 을 제공하므로 꺼 둔다.
        # 저장까지 가짜로 때우고 싶을 때만 켠다
        self.declare_parameter('serve_save_job', False)

        # 어느 쪽을 대신할지 고른다. 진짜 노드와 같은 Action 을 둘이 제공하면
        # 어느 쪽이 goal 을 받을지 알 수 없으므로, 진짜를 띄우는 쪽은 꺼야 한다.
        #   serve_measure=false + 진짜 audio_analyzer  -> 마이크만 실물로 검증
        #   serve_robot=false   + 진짜 robot_controller -> 로봇만 실물로 검증
        self.declare_parameter('serve_measure', True)   # /audio/measure_pitch
        self.declare_parameter('serve_robot', True)     # /robot/* 여섯 개

        # mode=sim 전용. 전부 지어낸 값이다 (상단 주석 참조).
        self.declare_parameter('sim_start_cents', -38.0)      # 시작할 때 벗어난 정도
        self.declare_parameter('sim_cents_per_deg', 0.15)     # 페그 1도당 센트
        self.declare_parameter('sim_slip', 0.18)              # 계산 대비 실제 오차 비율
        self.declare_parameter('sim_neck_drift', 0.06)        # 옆 줄이 끌려가는 비율

        # 줄별 현재 이탈(센트). sim 모드에서만 쓴다.
        self._sim_lock = threading.Lock()
        self._sim_cents = {}
        self._sim_seed = 12345      # 난수 대신 고정 수열. 돌릴 때마다 같은 흐름이 나온다

        mode = self.get_parameter('mode').value
        if mode not in MODES:
            raise ValueError('mode 는 %s 중 하나여야 한다 (받은 값: %r)' % (list(MODES), mode))

        # 측정과 발현은 겹쳐서 진행된다. task_manager 가 measure goal 을 보내고
        # ready 를 받은 뒤 measure 가 아직 실행 중일 때 play goal 을 보내기 때문이다.
        # 콜백 그룹을 나누지 않으면 play 가 measure 뒤에 막혀 교착한다.
        cb = ReentrantCallbackGroup()
        wanted = []
        if bool(self.get_parameter('serve_measure').value):
            wanted.append((MeasurePitch, '/audio/measure_pitch', self.on_measure))
        if bool(self.get_parameter('serve_robot').value):
            wanted += [(PreparePluck, '/robot/prepare_pluck', self.on_prepare_pluck),
                       (ReturnHome, '/robot/return_home', self.on_return_home),
                       (RegisterPeg, '/robot/register_peg', self.on_register),
                       (PlayString, '/robot/play_string', self.on_play),
                       (AdjustPeg, '/robot/adjust_peg', self.on_adjust),
                       (WipeStrings, '/robot/wipe_strings', self.on_wipe)]
        if not wanted:
            raise RuntimeError(
                'serve_measure 와 serve_robot 이 둘 다 false 다. 대신할 게 없으면 띄울 이유가 없다')
        for action_type, name, handler in wanted:
            ActionServer(self, action_type, name, handler,
                         callback_group=cb, cancel_callback=self.on_cancel)

        if self.get_parameter('serve_save_job').value:
            # 서비스는 별도 그룹에 둔다. MultiThreadedExecutor + Reentrant 조합에서
            # 서비스 콜백이 중복 실행되는 것을 통합 테스트에서 확인했다
            self.create_service(SaveJob, '/tuning/save_job', self.on_save_job,
                                callback_group=MutuallyExclusiveCallbackGroup())

        log = self.get_logger()
        serving = []
        if bool(self.get_parameter('serve_measure').value):
            serving.append('마이크')
        if bool(self.get_parameter('serve_robot').value):
            serving.append('로봇')
        log.warning('fake_hardware 기동 — %s 를 대신한다. 같은 자리의 진짜 노드와 같이 띄우지 마라'
                    % '·'.join(serving))
        log.info('  mode=%s' % mode)
        if mode == MODE_DETUNED:
            log.info('  항상 목표에서 %.1f센트 벗어난 값을 낸다. 페그를 돌려도 변하지 않는다'
                     % float(self.get_parameter('detune_cents').value))
        elif mode == MODE_FAIL:
            log.info('  항상 측정 실패(사유 %d)를 낸다'
                     % int(self.get_parameter('fail_reason').value))
        elif mode == MODE_SIM:
            log.warning('  수렴 시뮬레이션 — 각도-센트 상수가 실측값이 아니라 지어낸 값이다.')
            log.warning('  여기서 수렴한다고 실제로 수렴한다는 뜻이 아니다. 화면 확인 전용.')
            log.info('  시작 이탈 %.1f센트 / %.3f센트per도 / 오차 %.0f%% / 넥 끌림 %.0f%%'
                     % (float(self.get_parameter('sim_start_cents').value),
                        float(self.get_parameter('sim_cents_per_deg').value),
                        float(self.get_parameter('sim_slip').value) * 100,
                        float(self.get_parameter('sim_neck_drift').value) * 100))

    def _p(self, name):
        return self.get_parameter(name).value

    # ---------------------------------------------------------- sim 모드 상태
    # 실제 기타를 흉내 내는 최소한의 모형이다. 줄마다 '목표에서 몇 센트 벗어났나'만
    # 들고 있다가, 페그를 돌리면 그만큼 움직인다. 상수는 전부 지어낸 값이다.

    def _noise(self):
        """난수 대신 고정 수열. 돌릴 때마다 같은 흐름이 나와야 화면을 비교할 수 있다."""
        self._sim_seed = (self._sim_seed * 1103515245 + 12345) & 0x7FFFFFFF
        return (self._sim_seed / 0x7FFFFFFF) * 2.0 - 1.0      # -1.0 ~ +1.0

    def _sim_get(self, string_number):
        """줄의 현재 이탈(센트). 처음 물으면 시작값을 정해 둔다."""
        with self._sim_lock:
            if string_number not in self._sim_cents:
                # 줄마다 조금씩 다르게 시작한다. 전부 같으면 화면이 단조롭다.
                base = float(self._p('sim_start_cents'))
                self._sim_cents[string_number] = base * (1.0 + 0.35 * self._noise())
            return self._sim_cents[string_number]

    def _sim_adjust(self, string_number, direction, angle_deg):
        """
        페그를 돌린 결과를 반영한다.

        계산한 각도와 실제 결과가 정확히 맞으면 한 번에 수렴해 버린다. 실제로는
        파지 미끄러짐·백래시·축 어긋남으로 어긋나므로 sim_slip 만큼 흔든다.
        그래서 화면에서 조절 횟수가 여러 번으로 늘어난다.

        그리고 한 줄을 조이면 다른 줄이 조금 끌려간다. 6줄 합계 약 715N 이 넥을
        당기기 때문인데, 기획 2.5 가 최종 검증 패스를 두는 이유가 이것이다.
        """
        per_deg = float(self._p('sim_cents_per_deg'))
        slip = float(self._p('sim_slip'))
        drift = float(self._p('sim_neck_drift'))

        moved = direction * angle_deg * per_deg * (1.0 + slip * self._noise())
        with self._sim_lock:
            cur = self._sim_cents.get(string_number, 0.0)
            self._sim_cents[string_number] = cur + moved
            # 옆 줄이 끌려간다. 조인 쪽과 반대 방향으로 아주 조금.
            for n in list(self._sim_cents):
                if n != string_number:
                    self._sim_cents[n] -= moved * drift
        return moved

    def on_cancel(self, goal):
        """취소를 수락한다. 진행 중인 동작은 다음 확인 지점에서 빠져나온다."""
        self.get_logger().info('취소 요청 수락')
        return CancelResponse.ACCEPT

    def _sleep(self, goal, seconds) -> bool:
        """
        동작 소요 시간을 흉내 낸다. 취소되면 False.

        한 번에 자지 않고 잘게 나눠 자면서 취소를 확인한다. 진짜 로봇도 이래야
        cancel 이 동작하고(robot_controller 의 TODO 참조), 서버의 취소 경로를
        여기서 미리 검증할 수 있다.
        """
        remaining = float(seconds)
        step = 0.05
        while remaining > 0.0:
            if goal.is_cancel_requested:
                return False
            threading.Event().wait(min(step, remaining))
            remaining -= step
        return not goal.is_cancel_requested

    # ------------------------------------------------------------------ 마이크

    def on_measure(self, goal):
        req = goal.request
        res = MeasurePitch.Result()
        # 요청 식별자를 그대로 되돌려준다. task_manager 가 늦게 온 결과를 거르는 기준이다
        res.job_id = req.job_id
        res.string_number = req.string_number
        res.measurement_id = req.measurement_id

        if not self._sleep(goal, self._p('ready_delay_sec')):
            goal.canceled()
            res.success = False
            res.failure_reason = MeasurePitch.Result.FAIL_CANCELED
            return res

        # 녹음 준비 완료. 서버는 이걸 받은 뒤에 발현을 시킨다 (명세 5.2)
        fb = MeasurePitch.Feedback()
        fb.ready = True
        goal.publish_feedback(fb)

        if not self._sleep(goal, self._p('measure_delay_sec')):
            goal.canceled()
            res.success = False
            res.failure_reason = MeasurePitch.Result.FAIL_CANCELED
            return res

        mode = self._p('mode')
        if mode == MODE_FAIL:
            goal.succeed()
            res.success = False
            res.failure_reason = int(self._p('fail_reason'))
            return res

        if mode == MODE_SIM:
            cents = self._sim_get(req.string_number)
        elif mode == MODE_ON_TARGET:
            cents = 0.0
        else:
            cents = float(self._p('detune_cents'))
        goal.succeed()
        res.success = True
        res.frequency_hz = req.target_hz * (2.0 ** (cents / 1200.0))
        res.failure_reason = MeasurePitch.Result.FAIL_NONE
        return res

    # ------------------------------------------------------------------ 로봇

    def _simple(self, goal, result_type, delay_param, what):
        """발현·페그·화음처럼 '시간만 쓰고 성공하는' 동작의 공통부."""
        if not self._sleep(goal, self._p(delay_param)):
            goal.canceled()
            return result_type(success=False, message='취소됨')
        goal.succeed()
        self.get_logger().info(what)
        return result_type(success=True, message='')

    def on_return_home(self, goal):
        self._prepared_pluck = None
        goal.succeed()
        return ReturnHome.Result(success=True, message='')

    def on_register(self, goal):
        self._prepared_pluck = None
        goal.succeed()
        return RegisterPeg.Result(success=True, message='')

    def on_prepare_pluck(self, goal):
        self._prepared_pluck = None
        if not self._sleep(goal, self._p('prepare_delay_sec')):
            goal.canceled()
            return PreparePluck.Result(success=False, message='준비 취소')
        token = uuid.uuid4().hex
        self._prepared_pluck = (goal.request.string_number, token)
        goal.succeed()
        return PreparePluck.Result(success=True, preparation_id=token)

    def on_play(self, goal):
        token = goal.request.preparation_id
        prepared = self._prepared_pluck
        self._prepared_pluck = None
        if token and prepared != (goal.request.string_number, token):
            goal.abort()
            return PlayString.Result(success=False, message='준비 식별자 불일치')
        return self._simple(goal, PlayString.Result, 'play_delay_sec',
                            '%d번 줄 발현' % goal.request.string_number)

    def on_adjust(self, goal):
        self._prepared_pluck = None
        req = goal.request
        way = '조임' if req.direction > 0 else '풀림'

        if self._p('mode') != MODE_SIM:
            # 기본 모드에서는 음정을 바꾸지 않는다. 파일 상단 주석 참조
            return self._simple(
                goal, AdjustPeg.Result, 'adjust_delay_sec',
                '%d번 줄 페그 %s %.2f도 (음정은 변하지 않는다)'
                % (req.string_number, way, req.angle_deg))

        if not self._sleep(goal, self._p('adjust_delay_sec')):
            goal.canceled()
            return AdjustPeg.Result(success=False, message='취소됨')

        moved = self._sim_adjust(req.string_number, req.direction, req.angle_deg)
        goal.succeed()
        self.get_logger().info(
            '%d번 줄 페그 %s %.2f도 -> %+.1f센트 이동, 현재 %+.1f센트'
            % (req.string_number, way, req.angle_deg, moved,
               self._sim_get(req.string_number)))
        return AdjustPeg.Result(success=True, message='')

    def on_wipe(self, goal):
        self._prepared_pluck = None
        return self._simple(goal, WipeStrings.Result, 'wipe_delay_sec',
                            '넥 닦기 + 대기 자세 복귀')

    # ------------------------------------------------------------------ 저장 대역

    def on_save_job(self, req, res):
        self.get_logger().info('save_job 수신 (가짜): %s, 전체판정 %d, 줄 %d개'
                               % (req.job_id, req.overall_result, len(req.strings)))
        res.saved = True
        res.message = ''
        return res


def main():
    rclpy.init()
    node = FakeHardware()
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
