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
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP
from tunio_robot import peg_state as PS
from tunio_robot import teach_points as TP


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        default_yaml = os.path.join(
            get_package_share_directory("tunio_robot"), "config", "teach_points.yaml")
        self.declare_parameter("teach_points", default_yaml)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("peg_state_file", PS.DEFAULT_PATH)
        self.declare_parameter("peg_state_max_age_sec", PS.DEFAULT_MAX_AGE_SEC)
        # Phase 0 등록. 미등록 페그를 만나면 그 자리에서 돈다 (아래 _ensure_registered)
        self.declare_parameter("peg_register_force", True)
        self.declare_parameter("gripper_host", GW.DEFAULT_HOST)
        self.declare_parameter("gripper_port", GW.DEFAULT_PORT)
        self.declare_parameter("gripper_unit", GW.DEFAULT_UNIT)
        self.declare_parameter("gripper_width_reg", GW.WIDTH_REG)

        path = self.get_parameter("teach_points").value
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self._read_width = None
        self.points = TP.load(path)
        self.get_logger().info(f"교시점 로드: {path}")

        # 페그 손잡이 각. 조절할 때마다 바뀌므로 기억해 둬야 다음에 제대로 문다 (#31).
        # 파일로 남겨 노드가 죽었다 살아나도 Phase 0 를 다시 안 돌게 한다. 단 오래된
        # 값은 사람이 손으로 돌려놨을 수 있어 믿지 않는다 (peg_state 주석 참조)
        self.pegs = PS.PegState(
            path=self.get_parameter("peg_state_file").value,
            max_age_sec=float(self.get_parameter("peg_state_max_age_sec").value)).load()
        known = self.pegs.registered()
        self.get_logger().info(
            f"페그 손잡이 각 등록: {known if known else '없음 — Phase 0 필요'}")
        if self.dry_run:
            self.get_logger().warn("dry_run=true — 로봇을 움직이지 않는다")
        else:
            MP.connect(self.points.robot, self.points.gripper,
                       logger=self.get_logger())

        # 팔은 하나뿐이라 세 Action 이 같은 락을 쓴다
        self._lock = threading.Lock()
        self._running = None
        self._side = None          # 마지막으로 다녀온 헤드스톡 쪽
        # 지금 피크를 쥐고 있는지. 집기·반환 시점은 로봇이 스스로 판단하고
        # 서버는 지시하지 않는다 (CLAUDE.md 2절). 페그를 잡으려면 손가락이
        # 필요해 발현↔페그 조절 사이마다 반환·집기가 한 번씩 낀다.
        # 노드 시작 시 피크는 거치대에 있다고 가정한다 — Phase 0 은 페그
        # 손잡이 각도만 다루고 피크는 다루지 않는다
        self._holding_pick = False

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

        self._side 가 None 이면 **어디 있는지 모른다**는 뜻이라 무조건 경유지를 탄다.
        피크를 반환한 직후가 그렇다 — 거치대는 좌우 어느 쪽도 아니다 (#34).
        """
        if self._side == side:
            return
        route = self.points.route()
        if route is not None:
            self.get_logger().info(f"경유지를 거쳐 {side} 쪽으로")
            MP.goto_route(route, self.points.motion)

    def _ensure_registered(self, string_number):
        """
        페그 손잡이 각이 등록돼 있는지 보고, 없으면 **그 자리에서 Phase 0 를 돈다**.

        왜 여기서 하나
          서버는 "작업 시작" 을 로봇에 알리지 않는다. 로봇이 아는 건 Action 세 개뿐이다.
          피크 집기·반환 시점을 로봇이 스스로 판단하는 것과 같은 방식으로(CLAUDE.md 2절),
          등록도 **필요해지는 순간** 한다. 인터페이스가 안 바뀐다.

        왜 등록이 꼭 있어야 하나
          손잡이가 돌아가 있는데 교시 자세로 들어가면, 물리는 순간 페그가 그 각으로
          비틀려 **직전에 돌린 것이 통째로 풀린다.** 파지에 성공하고 동작도 성공으로
          보고하면서 조용히 틀린다 (#31). 그래서 모르면 돌리지 않는다.

        등록에 실패하면 예외가 그대로 올라가 이 goal 이 실패한다. 사람이 손잡이를
        눕혀야 하는 경우가 대부분이라, 메시지에 어느 줄인지가 들어 있다.
        """
        roll = self.pegs.roll_of(string_number)
        if roll is not None:
            return roll

        self.get_logger().info(
            f"{string_number}번 페그가 미등록이다. 지금 Phase 0 를 돈다 (#31)")
        roll = MP.register_peg(
            self.points.peg(string_number), self.points.motion,
            self.points.peg_handle(), self._width_reader(),
            logger=self.get_logger(),
            use_force=bool(self.get_parameter("peg_register_force").value))
        self.pegs.register(string_number, roll)
        self.pegs.save()
        self.get_logger().info(f"{string_number}번 손잡이 각 {roll:+.1f}도 등록")
        return roll

    def _width_reader(self):
        """그리퍼 폭을 읽는 함수. 한 번 만들어 두고 다시 쓴다."""
        if getattr(self, "_read_width", None) is None:
            self._read_width = GW.make_reader(
                self.get_parameter("gripper_host").value,
                int(self.get_parameter("gripper_port").value),
                int(self.get_parameter("gripper_unit").value),
                int(self.get_parameter("gripper_width_reg").value))
        return self._read_width

    def _ensure_pick(self):
        """
        피크가 필요한 동작(발현·화음) 전에 부른다. 이미 쥐고 있으면 아무 일도 안 한다.

        집는 시점을 여기서 판단하는 이유 — 서버는 play_string 이 언제 오는지만
        알지, 로봇이 지금 손이 비어 있는지는 모른다. 인터페이스를 안 늘리려면
        로봇이 스스로 판단해야 한다 (CLAUDE.md 2절).
        """
        if self._holding_pick:
            return
        holder = self.points.pick_holder()           # 미교시면 여기서 걸린다
        standby = self.points.frame("standby")
        if self.dry_run:
            self.get_logger().info(f"[dry_run] 피크 집기 {holder.task}")
        else:
            MP.pick_up(holder, self.points.motion, standby=standby)
        self._holding_pick = True

    def _release_pick(self):
        """페그를 잡아야 하는 동작(조절) 전에 부른다. 안 쥐고 있으면 아무 일도 안 한다."""
        if not self._holding_pick:
            return
        holder = self.points.pick_holder()
        if self.dry_run:
            self.get_logger().info(f"[dry_run] 피크 반환 {holder.task}")
        else:
            MP.pick_return(holder, self.points.motion)
        self._holding_pick = False
        # 반환이 끝나면 팔이 **거치대 위**에 있다. 페그도 경유지도 아니다.
        # _cross_to 는 "직전에 있던 쪽" 으로 판단하는데, 거치대는 어느 쪽도 아니므로
        # 그 기억을 지운다. 안 지우면 직전 페그와 같은 쪽일 때 경유지를 건너뛰고
        # **거치대에서 페그 접근점으로 관절 보간으로 바로 간다.** 그 경로는 아무도
        # 본 적이 없다 — 충돌 검증(2026-09-19)은 페그↔페그와 경유지만 봤다 (#34)
        self._side = None

    # ------------------------------------------------------ Action
    def on_play(self, goal):
        # Feedback 없음. 측정 타임아웃은 이 Result 와 무관하다 —
        # 마이크가 오디오 onset(무음 -> 소리)을 스스로 검출한 시점부터 센다.
        # 로봇 접근·피크 집기·파지 시간이 측정 제한 시간에 먹히지 않게 하려는 것이다.
        # (기획 2.4)
        no = goal.request.string_number

        def work():
            pose = self.points.pluck(no)            # 미교시면 여기서 걸린다
            self._ensure_pick()
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
            # 페그를 잡으려면 손가락이 필요하다. 피크를 쥐고 있으면 먼저 반환한다
            self._release_pick()
            if self.dry_run:
                via = "" if side == self._side else f" (경유: {side})"
                self.get_logger().info(
                    f"[dry_run] {r.string_number}번 페그 J{joint} {delta:+.1f}도{via}")
                self._side = side
                return
            self._cross_to(side)
            # 손잡이 각을 모르면 돌리지 않는다. 미등록이면 여기서 Phase 0 를 돈다 (#31)
            roll = self._ensure_registered(r.string_number)
            MP.adjust_peg(pose, joint, delta, self.points.motion, roll_deg=roll)
            # 돌린 만큼 손잡이도 돌아갔다. **성공한 뒤에만** 적는다 —
            # 중간에 실패하면 얼마나 돌았는지 모르므로 누적하면 더 틀어진다
            # J6 일 때만 누적한다. 관절 회전량이 곧 손목 롤(posx 의 C)인 건 J6 뿐이라,
            # 다른 관절 값을 각으로 쌓으면 다음 진입이 틀린 방향으로 돈다 (#31)
            if joint == 6:
                self.pegs.add_turn(r.string_number, delta)
                self.pegs.save()
            else:
                self.get_logger().warn(
                    f"peg_joint 가 J{joint} 라 손잡이 각을 누적하지 않는다. "
                    "각 추적은 J6 전용이다 (#31)")
            self._side = side

        return self._run("adjust_peg", goal, AdjustPeg.Result, work)

    def on_strum_all(self, goal):
        numbers = list(goal.request.string_numbers) or list(TP.STRINGS)

        def work():
            # 화음 연출 직후 대기 자세로 복귀한다. PHASE_RETURN 을 호출할 전용
            # Action 이 인터페이스 13개에 없어서 이 Action 이 복귀까지 책임진다
            # (2026-09-19 결정). 정상 완료 경로에만 있다.
            standby = self.points.frame("standby")
            # 화음도 발현이라 피크가 필요하다. 작업 마지막이니 훑은 뒤 거치대에
            # 돌려놓고 빈손으로 복귀한다 — 다음 작업 시작도 "거치대에 있다"는
            # 가정(Phase 0)이 그대로 성립해야 한다
            self._ensure_pick()
            if self.dry_run:
                self.get_logger().info(
                    f"[dry_run] 화음 훑기 {numbers} -> 대기 자세 복귀 {standby.joints}")
                return
            MP.strum_all(numbers, self.points)
            self._release_pick()
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
