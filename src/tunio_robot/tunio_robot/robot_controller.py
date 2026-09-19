"""
robot_controller — 발현·페그 조절·화음 Action 서버. 두산 API 연동.
기획서. 세 Action이 충돌 없이 순차 처리되도록 관리.

TODO(정묵·성현):
  - play_string: YAML 교시 좌표로 이동 -> 부착 피크로 타현 -> 5mm 후퇴
  - adjust_peg: 페그 파지 -> 회전(최종 접근은 조이는 방향으로 통일)
  - strum_all: 전체 줄 훑어 화음 (측정 없는 완료 연출)
  - 두산 API의 정지/재개, 파지 유지 검증
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from tunio_interfaces.action import AdjustPeg, PlayString, StrumAll
from tunio_robot import motion_primitives as MP


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        cb = ReentrantCallbackGroup()
        self.play_srv = ActionServer(
            self, PlayString, "/robot/play_string",
            self.on_play, callback_group=cb,
            cancel_callback=self.on_cancel)
        self.adjust_srv = ActionServer(
            self, AdjustPeg, "/robot/adjust_peg",
            self.on_adjust, callback_group=cb,
            cancel_callback=self.on_cancel)
        self.strum_srv = ActionServer(
            self, StrumAll, "/robot/strum_all",
            self.on_strum_all, callback_group=cb,
            cancel_callback=self.on_cancel)
        # TODO: 두산 API 연결 초기화, teach_points.yaml 로드
        self.get_logger().info("robot_controller 시작")

    def on_cancel(self, goal):
        """
        취소 요청 수락.

        cancel_callback 을 지정하지 않으면 rclpy 기본값이 REJECT 다. 그 상태로 두면
        task_manager 가 /tuning/cancel 을 받아 진행 중인 goal 에 취소를 걸어도
        로봇이 거부하고 하던 동작을 끝까지 수행한다. 기획 5.1 의 pause/cancel 과
        명세 6.4 취소 시퀀스가 성립하지 않는다.

        TODO(정묵·성현): 수락만으로는 움직이는 로봇이 멈추지 않는다. 두 가지가 더 필요하다.
          - 여기서 두산 API 의 정지를 호출한다
          - 각 on_* 의 동작이 goal.is_cancel_requested 를 주기적으로 확인하고
            goal.canceled() 로 빠져나간다. 지금은 MP 함수가 한 번에 블로킹이라
            확인 지점이 없다.
        """
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    def on_play(self, goal):
        MP.pluck_string(goal.request.string_number)   # TODO
        # Feedback 없음. 측정 타임아웃은 이 Result 와 무관하다 —
        # 마이크가 오디오 onset(무음 -> 소리)을 스스로 검출한 시점부터 센다.
        # 로봇 접근·파지 시간이 측정 제한 시간에 먹히지 않게 하려는 것이다. (기획 2.4)
        goal.succeed()
        res = PlayString.Result(); res.success = True; res.message = ""
        return res

    def on_adjust(self, goal):
        r = goal.request
        # direction: +1 조임(음 높이기) / -1 풀림
        MP.adjust_peg(r.string_number, r.direction, r.angle_deg)   # TODO
        goal.succeed()
        res = AdjustPeg.Result(); res.success = True; res.message = ""
        return res

    def on_strum_all(self, goal):
        MP.strum_all(list(goal.request.string_numbers))   # TODO
        # 화음 연출 직후 대기 자세로 복귀한다.
        # TuningState.PHASE_RETURN 을 호출할 전용 Action 이 인터페이스 13개에 없어서
        # 이 Action 이 복귀까지 책임진다. 인터페이스를 다시 열지 않으려는 선택이며,
        # 기획 Phase 2 에서 화음 연출 직후가 곧 복귀라 순서상으로도 이어진다.
        # (2026-09-19 결정. 정상 완료 경로에만 있으므로, 취소·오류로 끝난 경우의
        #  복귀는 별도 수단이 없다 — 지금은 사람이 회수한다.)
        MP.return_home()   # TODO
        goal.succeed()
        res = StrumAll.Result(); res.success = True; res.message = ""
        return res


def main():
    rclpy.init()
    from rclpy.executors import MultiThreadedExecutor
    node = RobotController()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
