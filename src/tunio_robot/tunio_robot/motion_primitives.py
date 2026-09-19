"""
두산 API를 감싸는 저수준 동작 함수. robot_controller 가 호출한다.

왜 전용 노드를 따로 만드는가
  DSR_ROBOT2 의 함수는 내부에서 rclpy.spin_until_future_complete(g_node, ...) 를 부른다
  (설치본 DSR_ROBOT2.py 635행 등). 그런데 robot_controller 노드는 이미
  MultiThreadedExecutor 가 spin 하고 있어서, 같은 노드로 또 spin 하면 충돌한다.
  그래서 DR_init.__dsr__node 에는 executor 에 넣지 않은 별도 노드를 물린다.
  이 노드는 robot_controller 의 락 안에서만 쓰이므로 동시 접근은 없다.

왜 네임스페이스가 필요한가
  DSR_ROBOT2 의 서비스 이름 접두사는 'dsr_controller2/' 로 **상대 경로**다.
  노드가 dsr01 네임스페이스에 있어야 /dsr01/dsr_controller2/... 로 풀린다.
  (우리 Action 이름은 '/robot/...' 로 절대 경로라 네임스페이스 영향을 받지 않는다)

그리퍼는 OnRobot RG2 이고, Compute Box 의 WebLogic 을 디지털 I/O 로 부른다
(2026-09-19 확정). 폭·힘은 웹에서 정하고 코드는 프리셋만 고른다. gripper.py 참조.

TODO(정묵·성현):
  - pluck_string: 피크를 집어 힘 제어로 타현 + 발현 직후 5mm 후퇴 + 피크 반환
  - strum_all: 전체 줄 훑기
  - 파지 확인 — RG2 의 grip detected 를 이 ROS 2 패키지가 토픽으로 안 내보낸다
    (gripper.py 주석 참조). 확인 수단을 정해야 한다
  - set_tcp — 피크를 든 상태와 페그를 잡은 상태의 TCP 가 다르다.
    티칭 펜던트에 각각 등록해야 한다
"""

from tunio_robot import teach_points as TP

# 두산 API 핸들. connect() 가 채운다. dry_run 이면 None 그대로다
_api = None
_node = None
_gripper = None


class MotionError(Exception):
    """동작 실패. 메시지가 Action Result 의 message 로 그대로 나간다."""


def connect(robot_cfg, gripper_cfg=None, logger=None):
    """
    두산 API와 그리퍼를 초기화한다.

    robot_cfg   — teach_points.yaml 의 robot 절 (id, model, tool, tcp)
    gripper_cfg — 같은 파일의 gripper 절 (presets, settle_sec)
    executor 에 넣지 않은 전용 노드를 만들어 DR_init 에 물린다.
    """
    global _api, _node, _gripper
    if _api is not None:
        return _api

    import rclpy

    import DR_init
    DR_init.__dsr__id = robot_cfg["id"]
    DR_init.__dsr__model = robot_cfg["model"]

    # executor 에 add_node 하지 않는다. DSR_ROBOT2 가 이 노드로 직접 spin 한다
    _node = rclpy.create_node("tunio_dsr", namespace=robot_cfg["id"])
    DR_init.__dsr__node = _node

    try:
        import DSR_ROBOT2 as api
    except ImportError as e:
        raise MotionError(f"두산 API(DSR_ROBOT2) import 실패: {e}")

    _api = api

    if gripper_cfg:
        from tunio_robot.gripper import RG2, parse_presets
        _gripper = RG2(api, parse_presets(gripper_cfg.get("presets")),
                       settle_sec=gripper_cfg.get("settle_sec", 0.3))

    if robot_cfg.get("tool"):
        api.set_tool(robot_cfg["tool"])
    if robot_cfg.get("tcp"):
        api.set_tcp(robot_cfg["tcp"])
    if logger:
        logger.info(f"두산 API 연결: {robot_cfg['id']} / {robot_cfg['model']}")
    return _api


def _require_api():
    if _api is None:
        raise MotionError("두산 API가 초기화되지 않았다. connect() 를 먼저 부르거나 dry_run 으로 돌려라")
    return _api


def _movej(joints, motion, relative=False):
    """관절 이동. relative=True 면 현재 자세 기준 상대 이동."""
    api = _require_api()
    from DR_common2 import posj
    mod = api.DR_MV_MOD_REL if relative else api.DR_MV_MOD_ABS
    return api.movej(posj(list(joints)),
                     vel=motion.get("vel", 30), acc=motion.get("acc", 30), mod=mod)


def _movel(task, motion):
    """작업 좌표로 직선 이동. BASE 기준 [X, Y, Z, A, B, C]."""
    api = _require_api()
    from DR_common2 import posx
    v = motion.get("travel_vel", [50.0, 30.0])
    a = motion.get("travel_acc", [100.0, 60.0])
    return api.movel(posx(list(task)), vel=v, acc=a)


def _movejx(task, motion, sol=None):
    """
    작업 좌표로 가되 **관절 보간**으로 간다. 경로는 직선이 아니다.

    특이점에서 출발하거나 특이점을 지나도 괜찮다. 직선 이동과 달리
    손끝 경로를 구속하지 않기 때문이다. 경유지에서 페그 접근점까지처럼
    "가기만 하면 되는" 구간에 쓴다.

    sol — 팔 자세(어느 역기구학 해를 쓸지). None 이면 지금 자세를 유지한다.
          엉뚱한 값을 주면 팔이 통째로 뒤집힌다.
    """
    api = _require_api()
    from DR_common2 import posx
    if sol is None:
        sol = api.get_current_solution_space()
    return api.movejx(posx(list(task)),
                      vel=motion.get("vel", 30), acc=motion.get("acc", 30), sol=sol)


def goto_route(route, motion):
    """
    경유지로 간다. null 인 관절은 지금 값을 그대로 둔다.

    관절 이동이라 특이점과 무관하다. 특이점이 문제가 되는 건 직선 이동뿐이다.
    """
    api = _require_api()
    current = list(api.get_current_posj())
    _movej(route.resolve(current), motion)


def _goto_pose(pose, motion):
    """
    교시 자세로 이동한다. 관절각이면 movej, 작업 좌표면 접근축을 따라 들어간다.

    작업 좌표는 물러난 지점까지 먼저 가고 거기서 직선으로 들어간다.
    바로 가면 경로가 기타를 쓸고 갈 수 있다.
    """
    if pose.kind == "joint":
        return _movej(pose.joints, motion)
    back = motion.get("approach_mm", 100.0)
    # 접근점까지는 관절 보간. 경유지가 특이점 자세라 직선으로 출발하면 안 된다
    _movejx(TP.backed_off(pose.task, back), motion, sol=getattr(pose, "sol", None))
    # 마지막 짧은 구간만 직선. 페그에 똑바로 들어가야 한다
    _movel(pose.task, motion)


def _retreat(pose, motion):
    """들어온 직선 그대로 물러난다."""
    if pose.kind == "task":
        _movel(TP.backed_off(pose.task, motion.get("approach_mm", 100.0)), motion)


def _grip(preset_name):
    """
    프리셋 이름으로 잡는다.

    폭·힘은 WebLogic 에 설정된 값이고 여기서 못 바꾼다. 'peg' 프리셋은
    페그 두께보다 1~2mm 여유를 준 폭이다. 꽉 물면 손목 회전축과 페그 축이
    조금만 어긋나도 옆힘이 걸려 많이 못 돌린다. 대신 그 여유(와 고무 골무의
    비틀림)가 그대로 헛돎이 되어 백래시에 더해진다.
    """
    if _gripper is None:
        raise MotionError("그리퍼가 초기화되지 않았다. connect() 를 먼저 불러라")
    return _gripper.select(preset_name)


def _release():
    """그리퍼를 연다."""
    return _grip("open")


def pluck_string(pose, motion):
    """줄 튕기기. 힘 제어로 일정한 세기 유지 후 즉시 5mm 이상 후퇴(버징 방지)."""
    raise NotImplementedError("성현 담당 — 발현 동작 교시 후 구현")


def adjust_peg(pose, joint, delta_deg, motion):
    """
    페그 조절. 받은 관절을 받은 각도만큼 돌린다.

    pose      — 페그 파지 자세 (teach_points.Pose)
    joint     — 돌릴 관절 번호 1~6
    delta_deg — 부호 붙은 회전량. 부호 변환은 teach_points.joint_delta 가 이미 했다

    백래시 대응(목표보다 높으면 풀었다 조여 올리기)은 task_manager 가 결정한다
    (명세 9.1 확정). 여기서 조이는 방향 마무리를 덧붙이면 서버가 계산한 각도와
    이중으로 겹쳐 결과가 어긋난다.
    """
    _require_api()
    _goto_pose(pose, motion)                        # 물러난 지점 -> 직선으로 페그 앞
    _grip("peg")                                    # 여유를 준 폭으로 파지
    try:
        delta = [0.0] * 6
        delta[joint - 1] = float(delta_deg)
        _movej(delta, motion, relative=True)        # 그 관절만 상대 회전
    finally:
        _release()                                  # 실패해도 페그는 놓는다
        _retreat(pose, motion)                      # 들어온 직선 그대로 나온다


def strum_all(string_numbers, points):
    """전체 줄을 훑어 화음. 측정 없는 완료 연출."""
    raise NotImplementedError("성현 담당 — 훑기 동작 교시 후 구현")
