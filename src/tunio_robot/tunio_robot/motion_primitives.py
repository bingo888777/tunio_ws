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

그리퍼는 OnRobot RG2 다 (2026-09-19 확정). 폭·힘을 수치로 준다. gripper.py 참조.

TODO(정묵·성현):
  - pluck_string: 피크를 집어 힘 제어로 타현 + 발현 직후 5mm 후퇴 + 피크 반환
  - strum_all: 전체 줄 훑기
  - 파지 확인 — RG2 의 grip detected 를 이 ROS 2 패키지가 토픽으로 안 내보낸다
    (gripper.py 주석 참조). 확인 수단을 정해야 한다
  - set_tcp — 피크를 든 상태와 페그를 잡은 상태의 TCP 가 다르다.
    티칭 펜던트에 각각 등록해야 한다
"""

# 두산 API 핸들. connect() 가 채운다. dry_run 이면 None 그대로다
_api = None
_node = None
_gripper = None


class MotionError(Exception):
    """동작 실패. 메시지가 Action Result 의 message 로 그대로 나간다."""


def connect(robot_cfg, logger=None):
    """
    두산 API를 초기화한다.

    robot_cfg — teach_points.yaml 의 robot 절 (id, model, tool, tcp)
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

    # 그리퍼. 서비스가 안 떠 있으면 여기서 걸리는 편이 동작 중에 걸리는 것보다 낫다
    from tunio_robot.gripper import RG2
    _gripper = RG2(_node)

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


def _grip(width_mm, force_n):
    """
    페그·피크를 잡는다.

    width_mm — 목표 폭. 페그 축을 정확히 안 잡아도 되게 여유를 준다.
      꽉 물면 손목 회전축과 페그 축이 조금만 어긋나도 페그에 옆힘이 걸려
      더 못 돌린다. 헐겁게 물면 그 어긋남을 흡수해 한 번에 많이 돌릴 수 있다.
      대신 그 여유가 그대로 헛돎(백래시)이 된다. 폭을 바꾸면 백래시도 바뀌므로
      **폭을 확정한 뒤에 백래시를 재야 한다** (tools/peg_cal.py --backlash).
    force_n — 쥐는 힘. RG2 범위 0~40N
    """
    if _gripper is None:
        raise MotionError("그리퍼가 초기화되지 않았다. connect() 를 먼저 불러라")
    if width_mm is None or force_n is None:
        raise MotionError("파지 폭·힘이 교시점 YAML 에 없다 (grip_width_mm / grip_force_n)")
    _gripper.grip(float(width_mm), float(force_n))


def _release():
    """그리퍼를 연다."""
    if _gripper is None:
        raise MotionError("그리퍼가 초기화되지 않았다. connect() 를 먼저 불러라")
    _gripper.release()


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
    _movej(pose.joints, motion)                     # 페그 앞으로
    _grip(pose.grip_width_mm, pose.grip_force_n)    # 여유를 준 폭으로 파지
    try:
        delta = [0.0] * 6
        delta[joint - 1] = float(delta_deg)
        _movej(delta, motion, relative=True)        # 그 관절만 상대 회전
    finally:
        _release()                                  # 실패해도 페그는 놓는다


def strum_all(string_numbers, points):
    """전체 줄을 훑어 화음. 측정 없는 완료 연출."""
    raise NotImplementedError("성현 담당 — 훑기 동작 교시 후 구현")
