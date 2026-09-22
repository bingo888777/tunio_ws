"""
robot_controller — 발현·페그 조절·화음 Action 서버. 두산 API 연동.

다섯 Action(register_peg / play_string / adjust_peg / strum_all / return_home)은 같은 팔을 쓰므로
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
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from tunio_interfaces.action import AdjustPeg, PlayString, RegisterPeg, ReturnHome, StrumAll
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP
from tunio_robot import peg_state as PS
from tunio_robot import runlog as RL
from tunio_robot import teach_points as TP


def _plain(msg):
    """
    ROS 메시지를 json 이 삼킬 수 있는 형태로 바꾼다.

    string_numbers 같은 칸이 array.array 로 와서 그대로 넣으면 json.dump 가
    TypeError 를 낸다. runlog.write 는 OSError 만 잡으므로 여기서 걸러야 한다.
    """
    out = {}
    for f in getattr(msg, "get_fields_and_field_types", lambda: {})():
        v = getattr(msg, f, None)
        if isinstance(v, (bool, int, float, str)) or v is None:
            out[f] = v
        else:
            try:
                out[f] = [float(x) for x in v]
            except (TypeError, ValueError):
                out[f] = repr(v)
    return out


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
        #
        # **힘 제어는 통합 뒤로 미뤘다** (2026-09-21 결정, #51 · #31).
        #   기본값이 False 라 진입은 위치 제어 + 순응제어로만 한다. 힘 제어를 켜려면
        #   런치에서 peg_register_force:=true 를 준다.
        #
        #   미룬 이유 — 아직 실물 검증이 안 끝났다. 9/21 회차가 4N 으로 20초에
        #   +7.5mm 밖에 못 갔고, 스프링을 이기고 갈 수 있는 거리도 자세마다 달라진다.
        #   통합 실기에서 봐야 할 것이 많은데 여기서 한 번 더 막히면 그 시간이 통째로
        #   날아간다. 값(search_mm · insert_force_n · insert_timeout_sec)은 계산이
        #   맞게 고쳐 뒀으므로 통합이 끝나면 이 기본값만 다시 올리면 된다.
        #
        #   순응제어는 이 값과 무관하게 켜지지만 **진입이 끝난 뒤다.** 기본값이면
        #   stop_mm 이 0 이라 페그까지 위치 제어로 가고, 순응은 파지·후퇴만 지킨다
        #   (motion_primitives.register_peg). PR #37 리뷰의 "미는 힘이 강성 × 남은
        #   거리로 묶인다" 는 **진입 구간에는 해당하지 않는다** — 그때는 순응을 켠 채
        #   들어가는 구조였다. 지금 진입의 안전망은 사람의 사전 정렬(CLAUDE.md 5절)과
        #   로봇 보호정지뿐이고, 보호정지 문턱은 아직 확인 안 됐다 (이슈 #53).
        self.declare_parameter("peg_register_force", False)
        # 이동 속도 배율. 0 이면 teach_points.yaml 의 motion.speed_scale 을 그대로 쓴다.
        # 실기에서 값을 바꿔 가며 보려고 런치 인자로 뺐다 — 빌드 없이 바꾼다 (#58).
        #   ros2 run ... --ros-args -p speed_scale:=1.0   (1.0 이면 예전 속도)
        # **이동·접근에만 걸린다.** 페그 회전·발현 스트로크·접촉 직전 구간은
        # 배율을 안 탄다 (motion_primitives._fast 의 주석).
        self.declare_parameter("speed_scale", 0.0)
        # 실행 기록. 도구(peg_register 등)는 docs/robot_data/ 에 원자료를 남기는데
        # **노드 경로는 아무것도 안 남겼다.** 2026-09-21 통합 실기가 끝난 뒤 볼 수
        # 있는 게 peg_state.json 의 각도 두 줄뿐이었다 — goal 이 몇 초 걸렸는지,
        # 어디서 실패했는지가 통째로 없다. 서버 타임아웃을 정하려면 그 시간이 필요하다.
        # 빈 문자열이면 안 남긴다.
        #
        # 기본값이 **홈 밑**이다. 도구들은 저장소의 docs/robot_data/ 에 남기지만
        # 그건 사람이 저장소 안에서 부르는 도구라 그렇다. 노드는 런치가 어디서
        # 뜨는지 모르고, cwd 기준으로 잡으면 실행한 자리마다 docs/ 가 생긴다
        # (실제로 테스트를 돌린 디렉터리에 하나 생겼다). peg_state 와 같은 자리를 쓴다.
        # 남길 만한 회차는 사람이 docs/robot_data/ 로 옮긴다.
        self.declare_parameter(
            "runlog_dir", os.path.expanduser("~/.tunio/robot_data"))
        self.declare_parameter("gripper_host", GW.DEFAULT_HOST)
        self.declare_parameter("gripper_port", GW.DEFAULT_PORT)
        self.declare_parameter("gripper_unit", GW.DEFAULT_UNIT)
        self.declare_parameter("gripper_width_reg", GW.WIDTH_REG)

        path = self.get_parameter("teach_points").value
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self._read_width = None
        self.points = TP.load(path)
        self.get_logger().info(f"교시점 로드: {path}")

        log_dir = str(self.get_parameter("runlog_dir").value or "")
        self._runlog_path = RL.session_path("node_session", log_dir) if log_dir else None
        self._runlog = {"tool": "robot_controller", "dry_run": self.dry_run, "goals": []}
        if self._runlog_path:
            self.get_logger().info(f"실행 기록: {self._runlog_path}")

        override = float(self.get_parameter("speed_scale").value or 0.0)
        if override > 0:
            self.points.motion["speed_scale"] = override
        scale = float(self.points.motion.get("speed_scale", 1.0))
        if scale != 1.0:
            self.get_logger().info(
                f"이동 속도 배율 {scale:g}배 "
                f"(관절 {MP.SPEED_CAP_JOINT:g}deg/s · 직선 {MP.SPEED_CAP_LINEAR:g}mm/s 에서 자른다). "
                "페그 회전·발현 스트로크·접촉 직전은 배율을 안 탄다")

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

        # 팔은 하나뿐이라 다섯 Action 이 같은 락을 쓴다
        self._lock = threading.Lock()
        self._running = None
        self._side = None          # 마지막으로 다녀온 헤드스톡 쪽
        # 지금 피크를 쥐고 있는지. 집기·반환 시점은 로봇이 스스로 판단하고
        # 서버는 지시하지 않는다 (CLAUDE.md 2절). 페그를 잡으려면 손가락이
        # 필요해 발현↔페그 조절 사이마다 반환·집기가 한 번씩 낀다.
        # 노드 시작 시 피크는 거치대에 있다고 가정한다 — Phase 0 은 페그
        # 손잡이 각도만 다루고 피크는 다루지 않는다
        #
        # True/False 둘로는 부족하다 — None(모름)이 있다. pick_up() 은
        # 그리퍼를 닫은 **뒤에도** 들어 올리기·대기 자세 이동이 남아 있는데,
        # 그 사이 예외가 나면 물리적으로는 피크를 쥔 채 함수만 실패한다. 그때
        # False 로 남으면 다음 _ensure_pick() 이 pick_up() 을 다시 불러
        # **그 첫 동작인 _release() 가 지금 팔이 있는 자리에서 피크를 놓아버린다**
        # (거치대 위가 아닐 수 있다). None 이면 그 대신 멈추고 사람을 부른다
        # (이슈 #34-1).
        self._holding_pick = False

        cb = ReentrantCallbackGroup()
        self.home_srv = ActionServer(
            self, ReturnHome, "/robot/return_home", self.on_return_home,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel, callback_group=cb)
        self.register_srv = ActionServer(
            self, RegisterPeg, "/robot/register_peg",
            self.on_register, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, callback_group=cb)
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
        t0 = time.time()
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
        took = time.time() - t0
        # 걸린 시간을 **항상** 남긴다. 서버의 play/adjust 타임아웃이 전부 임시값이고,
        # 특히 줄마다 첫 adjust_peg 는 안에서 Phase 0 를 돌아 훨씬 오래 걸린다.
        # 그 시간을 아무도 재 본 적이 없다 (CLAUDE.md 13절)
        self.get_logger().info(
            f"{name} {'성공' if res.success else '실패'} — {took:.1f}초")
        self._record(name, goal, res, took)
        return res

    def _record(self, name, goal, res, took):
        """Goal 하나를 실행 기록에 남기고 파일을 덮어쓴다. 실패해도 조용히 넘어간다."""
        if self._runlog_path is None:
            return
        req = goal.request
        self._runlog["goals"].append({
            "action": name,
            "at": time.strftime("%H:%M:%S"),
            "elapsed_sec": round(took, 2),
            "success": bool(res.success),
            "message": res.message,
            "request": _plain(req),
        })
        RL.write(self._runlog_path, self._runlog)

    def _cross_to(self, side):
        """실제 J1을 읽어 페그 쪽 접근에 필요한 경유를 결정한다."""
        MP.approach_peg_side(side, self.points.route(), self.points.motion,
                             logger=self.get_logger())

    def _ensure_registered(self, string_number):
        """
        페그 손잡이 각이 등록돼 있는지 보고, 없으면 **그 자리에서 Phase 0 를 돈다**.

        RegisterPeg 전용이다. AdjustPeg는 등록을 수행하지 않는다.

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

    def _ensure_pick(self, side=None):
        """
        피크가 필요한 동작(발현·화음) 전에 부른다. 이미 쥐고 있으면 아무 일도 안 한다.

        side — 이 다음에 갈 페그가 헤드스톡 어느 쪽인지("left"/"right").
               pick.holder.sol_by_side 에 그 쪽 해가 적혀 있으면 그 팔 모양으로
               집는다 (#58). 안 적혀 있으면(기본 null) 예전 그대로다.

        집는 시점을 여기서 판단하는 이유 — 서버는 play_string 이 언제 오는지만
        알지, 로봇이 지금 손이 비어 있는지는 모른다. 인터페이스를 안 늘리려면
        로봇이 스스로 판단해야 한다 (CLAUDE.md 2절).

        _holding_pick 이 None(모름)이면 거부한다 — 이전 집기가 중간에 실패해
        실제로는 쥐고 있을 수 있다. 그 상태로 또 pick_up() 을 부르면 첫 동작인
        _release() 가 지금 팔 위치에서 피크를 놓아버린다 (이슈 #34-1).
        """
        if self._holding_pick is None:
            raise MP.MotionError(
                "피크를 쥐고 있는지 모른다 (이전 집기 중 예외). "
                "그리퍼를 직접 확인한 뒤 노드를 재시작하라")
        if self._holding_pick:
            return
        holder = self.points.pick_holder()           # 미교시면 여기서 걸린다
        standby = self.points.frame("standby")
        if self.dry_run:
            self.get_logger().info(f"[dry_run] 피크 집기 {holder.task} (경유 {standby.joints})")
            self._holding_pick = True
            return
        self._holding_pick = None                     # 시도 중 — 끝나기 전엔 모른다
        sol = holder.sol_by_side.get(side) if side else None
        # via — 거치대로 내려가기 전에 관절 자세를 한 번 거친다. 피크를 가지러 갈 때
        # 팔은 직전 동작이 남긴 자리에 있고, 페그 조절 직후면 헤드스톡 쪽이다.
        # 거기서 거치대 위로 바로 movel 하면 기타 넥을 가로지른다. pick_return 이
        # 반대 방향에 같은 경유를 쓴다(성현, PR #38) — 이쪽만 비어 있었다 (#34 후속)
        MP.pick_up(holder, self.points.motion, standby=standby, via=standby, sol=sol,
                   route=self.points.route(), logger=self.get_logger())
        self._holding_pick = True
        # 경유지를 지나며 팔 모양을 그 쪽으로 맞췄다면 거기 서 있는 셈이다
        if sol is not None:
            self._side = side

    def _release_pick(self, side=None):
        """
        페그를 잡아야 하는 동작(조절) 전에 부른다. 안 쥐고 있으면 아무 일도 안 한다.

        반환 뒤 페그 접근 경유 여부는 _cross_to가 실제 J1으로 다시 판단한다.
        """
        if self._holding_pick is None:
            raise MP.MotionError(
                "피크를 쥐고 있는지 모른다 (이전 집기 중 예외). "
                "그리퍼를 직접 확인한 뒤 노드를 재시작하라")
        if not self._holding_pick:
            return
        holder = self.points.pick_holder()
        via = self.points.frame("standby")
        sol = holder.sol_by_side.get(side) if side else None
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] 피크 반환 {holder.task}"
                + (f" (팔 모양 sol {sol} — 경유지 생략)" if sol is not None else ""))
        else:
            MP.pick_return(holder, self.points.motion, via=via, sol=sol,
                           route=self.points.route(), logger=self.get_logger())
        self._holding_pick = False
        # 마지막 팔 모양 기록. 페그 경유 판단에는 실제 J1을 사용한다.
        self._side = side if sol is not None else None

    # ------------------------------------------------------ Action
    def on_return_home(self, goal):
        def work():
            self._release_pick()
            if not self.dry_run:
                MP.return_home(self.points.frame("standby"), self.points.motion)
                self._side = None

        return self._run("return_home", goal, ReturnHome.Result, work)

    def on_register(self, goal):
        no = goal.request.string_number

        def work():
            self.points.peg(no)  # 줄 번호와 교시 여부 검증
            if self.pegs.roll_of(no) is not None:
                return
            if self.dry_run:
                self.get_logger().info(f"[dry_run] {no}번 페그 초기 등록")
                return  # 가짜 등록값을 실기 파일에 저장하지 않는다
            side = TP.side_of(no)
            self._release_pick(side)
            self._cross_to(side)
            self._ensure_registered(no)
            self._side = side

        return self._run("register_peg", goal, RegisterPeg.Result, work)

    def on_play(self, goal):
        # Feedback 없음. 측정 타임아웃은 이 Result 와 무관하다 —
        # 마이크가 오디오 onset(무음 -> 소리)을 스스로 검출한 시점부터 센다.
        # 로봇 접근·피크 집기·파지 시간이 측정 제한 시간에 먹히지 않게 하려는 것이다.
        # (기획 2.4)
        no = goal.request.string_number

        def work():
            pose = self.points.pluck(no)            # 미교시면 여기서 걸린다
            # 이 줄의 페그 쪽으로 집는다 — 발현 다음은 거의 항상 이 줄의 조절이다 (#58)
            self._ensure_pick(TP.side_of(no))
            standby = self.points.frame("pluck_standby")
            if self.dry_run:
                self.get_logger().info(f"[dry_run] {no}번 줄 발현 {pose.task}")
                return
            MP.pluck_string(pose, self.points.motion, standby=standby)

        return self._run("play_string", goal, PlayString.Result, work)

    def on_adjust(self, goal):
        r = goal.request

        def work():
            # direction(+1 조임 / -1 풀림)과 줄별 turn_sign 을 관절 부호로 바꾼다.
            # 백래시 대응(풀었다 조이기)은 task_manager 가 결정한다. 여기서 덧붙이지 않는다.
            joint, delta = self.points.joint_delta(r.string_number, r.direction, r.angle_deg)
            pose = self.points.peg(r.string_number)
            side = TP.side_of(r.string_number)
            roll = self.pegs.roll_of(r.string_number)
            if not self.dry_run and roll is None:
                raise MP.MotionError(
                    f"{r.string_number}번 페그 미등록: register_peg를 먼저 완료하라. "
                    "adjust_peg는 초기 등록을 실행하지 않는다")
            # 페그를 잡으려면 손가락이 필요하다. 피크를 쥐고 있으면 먼저 반환한다.
            # 갈 쪽을 알려 줘서 그 쪽 팔 모양으로 반환하게 한다 (#58)
            self._release_pick(side)
            if self.dry_run:
                via = "" if side == self._side else f" (경유: {side})"
                self.get_logger().info(
                    f"[dry_run] {r.string_number}번 페그 J{joint} {delta:+.1f}도{via}")
                self._side = side
                return
            self._cross_to(side)
            # 회전/후퇴 중 실패하면 예전 각을 다시 믿지 않는다. 되감기는 페그를
            # 안 돌리므로 누적량에 넣지 않고, 요청한 delta만 성공 후 확정한다 (#70).
            with self.pegs.turning(r.string_number, delta if joint == 6 else None):
                MP.adjust_peg(pose, joint, delta, self.points.motion, roll_deg=roll)
            if joint != 6:
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
            # TODO(성현, 이슈 #34-3): strum_all 이 구현되면 여기 _ensure_pick() 을
            # 다시 넣는다 — 화음도 발현이라 피크가 필요하다. 지금 넣어두면
            # motion_primitives.strum_all() 이 NotImplementedError 를 던지기
            # *전에* 실제로 피크를 집어버려서, dry_run 이 아닌 실기에서 로봇이
            # 피크를 쥔 채 반환도 못 하고 멈춘다. 미구현 상태에선 빼 둔다.
            # (반환 쪽은 그대로 둔다 — play_string 이 앞서 쥐어 놨을 수 있어서다)
            if self.dry_run:
                self.get_logger().info(
                    f"[dry_run] 화음 훑기 {numbers} -> 대기 자세 복귀 {standby.joints}")
                return
            # strum_all() 이 아직 NotImplementedError 를 던지므로, 반환을 그 뒤에
            # 두면 절대 안 탄다 (PR #38 리뷰 ③). play_string 이 앞서 쥐어 놨을
            # 피크를 반환하려면 여기서 먼저 해야 한다 — 미구현 상태라 앞에 둬도
            # 잃는 게 없다.
            self._release_pick()
            # strum_all() 은 아직 NotImplementedError 를 던진다. 그대로 두면 **복귀가
            # 통째로 안 돈다** — 바로 다음 줄인데 예외가 여기서 함수를 끝내기 때문이다.
            # 서버는 화음 실패를 "연출이라 결과에 영향 없다" 며 경고로만 넘기므로
            # 화면에는 '조율 완료' 가 뜨는데, 실물은 팔이 마지막 페그 앞에 남는다.
            # 아무도 그 사실을 모른다.
            #
            # 복귀는 화음과 달리 연출이 아니다. 무슨 일이 있어도 돌아가야 한다.
            try:
                MP.strum_all(numbers, self.points)
            except Exception as exc:               # noqa: B902 — 복귀가 먼저다
                self.get_logger().warn(f"화음 훑기 실패 — {exc}. 복귀는 그대로 한다")
                MP.return_home(standby, self.points.motion)
                raise
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
