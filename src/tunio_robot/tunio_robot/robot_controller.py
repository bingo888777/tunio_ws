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
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
            MP.connect(self.points.robot, self.points.gripper,
                       logger=self.get_logger())

        # 팔은 하나뿐이라 세 Action 이 같은 락을 쓴다
        self._lock = threading.Lock()
        self._running = None
        self._side = None          # 마지막으로 다녀온 헤드스톡 쪽

        cb = ReentrantCallbackGroup()
        self.play_srv = ActionServer(
            self, PlayString, "/robot/play_string",
            self.on_play, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=cb)
        self.adjust_srv = ActionServer(
            self, AdjustPeg, "/robot/adjust_peg",
            self.on_adjust, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=cb)
        self.strum_srv = ActionServer(
            self, StrumAll, "/robot/strum_all",
            self.on_strum_all, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=cb)
        self.get_logger().info("robot_controller 시작")

    # ------------------------------------------------------ 공통
    def on_goal(self, goal_request):
        """이미 다른 동작이 돌고 있으면 받지 않는다. 팔이 하나라 겹칠 수 없다."""
        with self._lock:
            if self._running is not None:
                self.get_logger().warn(f"'{self._running}' 실행 중이라 새 goal 을 거부한다")
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        """
        취소 요청 수락.

        cancel_callback 을 지정하지 않으면 rclpy 기본값이 REJECT 다. 그 상태로 두면
        task_manager 가 /tuning/cancel 을 받아 진행 중인 goal 에 취소를 걸어도
        로봇이 거부하고 하던 동작을 끝까지 수행한다. 기획 5.1 의 pause/cancel 과
        명세 6.4 취소 시퀀스가 성립하지 않는다. 페그를 조이는 중에 멈출 수 없다는
        안전 문제이기도 하다.

        TODO(정묵·성현): 수락만으로는 움직이는 로봇이 멈추지 않는다. 두 가지가 더 필요하다.
          - 여기서 두산 API 의 정지를 호출한다
          - _run() 의 work() 가 goal.is_cancel_requested 를 주기적으로 확인하고
            goal.canceled() 로 빠져나간다. 지금은 MP 함수가 한 번에 블로킹이라
            확인 지점이 없다. tunio_server 의 fake_hardware.py 가 잘게 나눠 자면서
            확인하는 형태로 구현돼 있으니 참고할 수 있다.
        """
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

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

    def _cross_to(self, side):
        """
        헤드스톡 반대쪽으로 넘어갈 때만 경유지를 거친다.

        같은 쪽 페그끼리는 37mm 밖에 안 떨어져 있어 바로 간다.
        반대쪽은 손목이 뒤집혀야 하고, TCP 가 208mm 라 그 뒤집힘이
        기타 근처에서 일어나면 손끝이 큰 호를 그리며 부딪힌다.
        """
        if self._side == side:
            return
        route = self.points.route()
        if route is not None:
            self.get_logger().info(f"경유지를 거쳐 {side} 쪽으로")
            MP.goto_route(route, self.points.motion)

    # ------------------------------------------------------ Action
    def on_play(self, goal):
        # Feedback 없음. 측정 타임아웃은 이 Result 와 무관하다 —
        # 마이크가 오디오 onset(무음 -> 소리)을 스스로 검출한 시점부터 센다.
        # 로봇 접근·피크 집기·파지 시간이 측정 제한 시간에 먹히지 않게 하려는 것이다.
        # (기획 2.4)
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
            side = TP.side_of(r.string_number)
            if self.dry_run:
                via = "" if side == self._side else f" (경유: {side})"
                self.get_logger().info(
                    f"[dry_run] {r.string_number}번 페그 J{joint} {delta:+.1f}도{via}")
                self._side = side
                return
            self._cross_to(side)
            MP.adjust_peg(pose, joint, delta, self.points.motion)
            self._side = side

        return self._run("adjust_peg", goal, AdjustPeg.Result, work)

    def on_strum_all(self, goal):
        numbers = list(goal.request.string_numbers) or list(TP.STRINGS)

        def work():
            # 화음 연출 직후 대기 자세로 복귀한다. PHASE_RETURN 을 호출할 전용
            # Action 이 인터페이스 13개에 없어서 이 Action 이 복귀까지 책임진다
            # (2026-09-19 결정). 정상 완료 경로에만 있다.
            standby = self.points.frame("standby")
            if self.dry_run:
                self.get_logger().info(
                    f"[dry_run] 화음 훑기 {numbers} -> 대기 자세 복귀 {standby.joints}")
                return
            MP.strum_all(numbers, self.points)
            MP.return_home(standby, self.points.motion)

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
