"""
robot_controller — 이송·발현·페그 조절 Action 서버. 두산 API 연동.
기획서. 네 Action이 충돌 없이 순차 처리되도록 관리.

TODO(정묵·성현):
  - play_string: YAML 교시 좌표로 이동 -> 부착 피크로 타현 -> 5mm 후퇴
  - adjust_peg: 페그 파지 -> 회전(최종 접근은 조이는 방향으로 통일)
  - press_pin: 브릿지핀을 힘 제어로 눌러 안착 확인
  - strum_all: 전체 줄 훑어 화음 (측정 없는 완료 연출)
  - 두산 API의 정지/재개, 파지 유지 검증
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from tunio_interfaces.action import PlayString, AdjustPeg, PressPin, StrumAll
from tunio_robot import motion_primitives as MP


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        cb = ReentrantCallbackGroup()
        self.play_srv = ActionServer(
            self, PlayString, "/robot/play_string",
            self.on_play, callback_group=cb)
        self.adjust_srv = ActionServer(
            self, AdjustPeg, "/robot/adjust_peg",
            self.on_adjust, callback_group=cb)
        self.press_srv = ActionServer(
            self, PressPin, "/robot/press_pin",
            self.on_press_pin, callback_group=cb)
        self.strum_srv = ActionServer(
            self, StrumAll, "/robot/strum_all",
            self.on_strum_all, callback_group=cb)
        # TODO: 두산 API 연결 초기화, teach_points.yaml 로드
        self.get_logger().info("robot_controller 시작")

    def on_play(self, goal):
        MP.pluck_string(goal.request.string_no)   # TODO
        goal.succeed()
        res = PlayString.Result(); res.done = True; res.message = "ok"
        return res

    def on_adjust(self, goal):
        r = goal.request
        MP.adjust_peg(r.string_no, r.raise_pitch, r.angle_deg)   # TODO
        goal.succeed()
        res = AdjustPeg.Result(); res.done = True; res.message = "ok"
        return res

    def on_press_pin(self, goal):
        seated = MP.press_pin(goal.request.string_no)   # TODO
        goal.succeed()
        res = PressPin.Result(); res.done = True; res.seated = bool(seated)
        res.message = "ok"
        return res

    def on_strum_all(self, goal):
        MP.strum_all(list(goal.request.strings))   # TODO
        goal.succeed()
        res = StrumAll.Result(); res.done = True; res.message = "ok"
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
