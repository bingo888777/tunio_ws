"""
fake_hardware — 로봇·마이크 없이 서버를 통째로 돌려 보기 위한 하드웨어 대역. 담당: 정동휘.

robot_controller 와 audio_analyzer 를 대신해 네 Action 을 제공한다. 그 위에서
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

⚠ 페그를 돌려도 음정이 변하지 않는다. 일부러 그렇게 두었다.
  각도-센트 실측값이 아직 없어서, 수렴하는 척하려면 가짜 상수를 넣어야 한다.
  그러면 나중에 진짜로 수렴하지 않을 때 원인이 코드인지 상수인지 구분되지 않는다.
  실측이 끝나면 그때 반영한다 (CLAUDE.md 13절 미확정 항목).
"""
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tunio_interfaces.action import AdjustPeg, MeasurePitch, PlayString, StrumAll
from tunio_interfaces.srv import SaveJob

# mode 파라미터 값
MODE_ON_TARGET = 'on_target'   # 항상 목표 주파수를 낸다 -> 4줄 모두 조절 없이 성공
MODE_DETUNED = 'detuned'       # 항상 detune_cents 만큼 벗어난다 -> 페그 조절 경로로 간다
MODE_FAIL = 'fail'             # 항상 측정 실패 -> 재측정 소진 경로로 간다
MODES = (MODE_ON_TARGET, MODE_DETUNED, MODE_FAIL)


class FakeHardware(Node):

    def __init__(self):
        super().__init__('fake_hardware')

        self.declare_parameter('mode', MODE_ON_TARGET)
        self.declare_parameter('detune_cents', -30.0)      # mode=detuned 일 때의 오차
        self.declare_parameter('fail_reason', 1)           # mode=fail 일 때. 1=소리 부족
        self.declare_parameter('ready_delay_sec', 0.05)    # 마이크 준비 흉내
        self.declare_parameter('measure_delay_sec', 0.3)   # 녹음+분석 흉내
        self.declare_parameter('play_delay_sec', 0.1)      # 접근+발현+후퇴 흉내
        self.declare_parameter('adjust_delay_sec', 0.2)    # 파지+회전 흉내
        self.declare_parameter('strum_delay_sec', 0.3)     # 화음+복귀 흉내
        # result_store 를 같이 띄우면 그쪽이 /tuning/save_job 을 제공하므로 꺼 둔다.
        # 저장까지 가짜로 때우고 싶을 때만 켠다
        self.declare_parameter('serve_save_job', False)

        mode = self.get_parameter('mode').value
        if mode not in MODES:
            raise ValueError('mode 는 %s 중 하나여야 한다 (받은 값: %r)' % (list(MODES), mode))

        # 측정과 발현은 겹쳐서 진행된다. task_manager 가 measure goal 을 보내고
        # ready 를 받은 뒤 measure 가 아직 실행 중일 때 play goal 을 보내기 때문이다.
        # 콜백 그룹을 나누지 않으면 play 가 measure 뒤에 막혀 교착한다.
        cb = ReentrantCallbackGroup()
        for action_type, name, handler in (
                (MeasurePitch, '/audio/measure_pitch', self.on_measure),
                (PlayString, '/robot/play_string', self.on_play),
                (AdjustPeg, '/robot/adjust_peg', self.on_adjust),
                (StrumAll, '/robot/strum_all', self.on_strum)):
            ActionServer(self, action_type, name, handler,
                         callback_group=cb, cancel_callback=self.on_cancel)

        if self.get_parameter('serve_save_job').value:
            # 서비스는 별도 그룹에 둔다. MultiThreadedExecutor + Reentrant 조합에서
            # 서비스 콜백이 중복 실행되는 것을 통합 테스트에서 확인했다
            self.create_service(SaveJob, '/tuning/save_job', self.on_save_job,
                                callback_group=MutuallyExclusiveCallbackGroup())

        log = self.get_logger()
        log.warning('fake_hardware 기동 — 가짜 로봇·마이크다. 진짜 노드와 같이 띄우지 마라')
        log.info('  mode=%s' % mode)
        if mode == MODE_DETUNED:
            log.info('  항상 목표에서 %.1f센트 벗어난 값을 낸다. 페그를 돌려도 변하지 않는다'
                     % float(self.get_parameter('detune_cents').value))
        elif mode == MODE_FAIL:
            log.info('  항상 측정 실패(사유 %d)를 낸다'
                     % int(self.get_parameter('fail_reason').value))

    def _p(self, name):
        return self.get_parameter(name).value

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

        cents = 0.0 if mode == MODE_ON_TARGET else float(self._p('detune_cents'))
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

    def on_play(self, goal):
        return self._simple(goal, PlayString.Result, 'play_delay_sec',
                            '%d번 줄 발현' % goal.request.string_number)

    def on_adjust(self, goal):
        req = goal.request
        # 실제로 음정을 바꾸지는 않는다. 파일 상단 주석 참조
        return self._simple(
            goal, AdjustPeg.Result, 'adjust_delay_sec',
            '%d번 줄 페그 %s %.2f도 (음정은 변하지 않는다)'
            % (req.string_number, '조임' if req.direction > 0 else '풀림', req.angle_deg))

    def on_strum(self, goal):
        nums = list(goal.request.string_numbers)
        return self._simple(goal, StrumAll.Result, 'strum_delay_sec',
                            '화음 %s + 대기 자세 복귀' % (nums if nums else '전체'))

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
