"""
robot_controller — 발현·페그 조절·화음 Action 서버. 두산 API 연동.

세 Action(play_string / adjust_peg / strum_all)은 같은 팔을 쓰므로
동시에 실행되면 안 된다. 하나가 도는 동안 들어온 goal 은 REJECT 한다
(audio_analyzer 의 동시 요청 거부와 같은 방식).

dry_run 파라미터
  true  — 두산 API를 부르지 않고 goal 검증·좌표 조회까지만 한다.
          로봇 없이 서버와 붙여볼 때 쓴다.
  false — 실제로 움직인다.
기본값은 true 다. 실수로 켜자마자 움직이는 것보다 안 움직이는 쪽이 안전하다.

TODO(정묵·성현):
  - motion_primitives 의 실제 동작 채우기 (그리퍼 방식 확정 후)
  - 두산 API의 정지/재개, 파지 유지 검증
"""
import os
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from tunio_interfaces.action import AdjustPeg, PlayString, StrumAll
from tunio_robot import motion_primitives as MP
from tunio_robot import teach_points as TP


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        default_yaml = os.path.join(
            get_package_share_directory("tunio_robot"), "config", "teach_points.yaml")
        self.declare_parameter("teach_points", default_yaml)
        self.declare_parameter("dry_run", True)

        path = self.get_parameter("teach_points").value
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.points = TP.load(path)
        self.get_logger().info(f"교시점 로드: {path}")
        if self.dry_run:
            self.get_logger().warn("dry_run=true — 로봇을 움직이지 않는다")
        else:
            MP.connect(self.points.robot, logger=self.get_logger())

        # 팔은 하나뿐이라 세 Action 이 같은 락을 쓴다
        self._lock = threading.Lock()
        self._running = None

        cb = ReentrantCallbackGroup()
        self.play_srv = ActionServer(
            self, PlayString, "/robot/play_string",
            self.on_play, goal_callback=self.on_goal, callback_group=cb)
        self.adjust_srv = ActionServer(
            self, AdjustPeg, "/robot/adjust_peg",
            self.on_adjust, goal_callback=self.on_goal, callback_group=cb)
        self.strum_srv = ActionServer(
            self, StrumAll, "/robot/strum_all",
            self.on_strum_all, goal_callback=self.on_goal, callback_group=cb)
        self.get_logger().info("robot_controller 시작")

    # ------------------------------------------------------ 공통
    def on_goal(self, goal_request):
        """이미 다른 동작이 돌고 있으면 받지 않는다. 팔이 하나라 겹칠 수 없다."""
        with self._lock:
            if self._running is not None:
                self.get_logger().warn(f"'{self._running}' 실행 중이라 새 goal 을 거부한다")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _run(self, name, goal, result_type, work):
        """
        하나의 Action 을 락 안에서 실행하고 Result 를 만든다.

        work() 가 예외 없이 끝나면 success=True, 예외면 success=False + 사유.
        서버가 message 를 화면·DB에 그대로 싣기 때문에 사유를 사람 말로 남긴다.
        """
        with self._lock:
            if self._running is not None:
                goal.abort()
                res = result_type()
                res.success = False
                res.message = f"{self._running} 실행 중"
                return res
            self._running = name

        res = result_type()
        try:
            work()
            res.success = True
            res.message = ""
            goal.succeed()
        except Exception as e:                      # noqa: B902 — 사유를 Result 로 넘긴다
            res.success = False
            res.message = f"{type(e).__name__}: {e}"
            self.get_logger().error(f"{name} 실패 — {res.message}")
            goal.abort()
        finally:
            with self._lock:
                self._running = None
        return res

    # ------------------------------------------------------ Action
    def on_play(self, goal):
        no = goal.request.string_number

        def work():
            pose = self.points.pluck(no)            # 미교시면 여기서 걸린다
            if self.dry_run:
                self.get_logger().info(f"[dry_run] {no}번 줄 발현 {pose.joints}")
                return
            MP.pluck_string(pose, self.points.motion)

        return self._run("play_string", goal, PlayString.Result, work)

    def on_adjust(self, goal):
        r = goal.request

        def work():
            # direction(+1 조임 / -1 풀림)과 줄별 turn_sign 을 관절 부호로 바꾼다.
            # 백래시 대응(풀었다 조이기)은 task_manager 가 결정한다. 여기서 덧붙이지 않는다.
            joint, delta = self.points.joint_delta(r.string_number, r.direction, r.angle_deg)
            pose = self.points.peg(r.string_number)
            if self.dry_run:
                self.get_logger().info(
                    f"[dry_run] {r.string_number}번 페그 J{joint} {delta:+.1f}도")
                return
            MP.adjust_peg(pose, joint, delta, self.points.motion)

        return self._run("adjust_peg", goal, AdjustPeg.Result, work)

    def on_strum_all(self, goal):
        numbers = list(goal.request.string_numbers) or list(TP.STRINGS)

        def work():
            if self.dry_run:
                self.get_logger().info(f"[dry_run] 화음 훑기 {numbers}")
                return
            MP.strum_all(numbers, self.points)

        return self._run("strum_all", goal, StrumAll.Result, work)


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
