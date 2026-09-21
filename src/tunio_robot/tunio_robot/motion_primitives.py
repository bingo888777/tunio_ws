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

tool 과 TCP 는 connect() 가 **항상 설정하고 읽어서 확인한다** (이슈 #26·#23·#21)
  배포용이라 펜던트 설정에 기대지 않는다. 설정이 안 된 로봇에 가져다 놓아도 같은
  코드가 돌아야 하므로, 이미 맞아 보여도 set 을 생략하지 않는다. 이름은
  move.py 에서 동작이 확인된 것과 같다 — tool "Tool Weight", tcp "GripperDA_v1".

  판정은 set 반환값이 아니라 get 결과로 한다. 펜던트가 켜 둔 경우 set 이 -1 로
  거부돼도 좌표는 맞기 때문이다(#23). 못 맞추면 움직이기 전에 멈춘다.

  교시 좌표는 **읽을 때 켜져 있던 TCP 와 짝**이다. 그 이름을 YAML 의 pose 마다
  tcp: 로 적어 두고, teach_points 가 robot.tcp 와 다르면 조회를 거부한다.
  섞인 채로 움직이면 208mm 어긋난다 (#21).

TODO(정묵·성현):
  - pluck_string: 힘 제어로 타현 (지금은 위치 제어만)
  - strum_all: 전체 줄 훑기
  - 파지 확인 — RG2 의 grip detected 를 이 ROS 2 패키지가 토픽으로 안 내보낸다
    (gripper.py 주석 참조). 확인 수단을 정해야 한다
"""
import time

from tunio_robot import teach_points as TP

# 두산 API 핸들. connect() 가 채운다. dry_run 이면 None 그대로다
_api = None
_node = None
_gripper = None
_robot_cfg = None       # connect() 가 받은 robot 절. ensure_tcp 가 기본값으로 쓴다


class MotionError(Exception):
    """동작 실패. 메시지가 Action Result 의 message 로 그대로 나간다."""


def connect(robot_cfg, gripper_cfg=None, logger=None):
    """
    두산 API와 그리퍼를 초기화한다.

    robot_cfg   — teach_points.yaml 의 robot 절 (id, model, tool, tcp)
    gripper_cfg — 같은 파일의 gripper 절 (presets, settle_sec)
    executor 에 넣지 않은 전용 노드를 만들어 DR_init 에 물린다.
    """
    global _api, _node, _gripper, _robot_cfg
    if _api is not None:
        return _api
    _robot_cfg = dict(robot_cfg)

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

    # 첫 호출 전에 서비스가 보이기를 기다린다. 안 기다리면 첫 호출이 영원히 멈춘다.
    # 디스커버리가 불안정해(#23) 기본값을 넉넉히 둔다. robot_cfg 로 덮을 수 있다
    _wait_services(_node, robot_cfg["id"],
                   timeout_sec=float(robot_cfg.get("service_wait_sec", 30.0)),
                   logger=logger)

    # 배포용이라 펜던트 설정에 기대지 않는다. 항상 설정하고, 읽어서 확인한다.
    # 순서는 tool 먼저다 — 무게 설정이 먼저 들어가야 TCP 보정이 제 값을 쓴다
    _ensure_named("tool", robot_cfg.get("tool"), logger=logger)
    _ensure_named("tcp", robot_cfg.get("tcp"), logger=logger)
    if logger:
        logger.info(f"두산 API 연결: {robot_cfg['id']} / {robot_cfg['model']} "
                    f"tool='{_read_named('tool')}' tcp='{_read_named('tcp')}'")
    return _api


# connect() 가 기다릴 서비스. DSR_ROBOT2 가 실제로 쓰는 것들 중 대표만 고른다
_WAIT_SERVICES = [
    ("GetCurrentTool", "dsr_controller2/tool/get_current_tool"),
    ("GetCurrentTcp", "dsr_controller2/tcp/get_current_tcp"),
    ("GetCurrentPosj", "dsr_controller2/aux_control/get_current_posj"),
    ("MoveJoint", "dsr_controller2/motion/move_joint"),
    ("MoveLine", "dsr_controller2/motion/move_line"),
    # 힘 제어 계열 (Phase 0 진입이 쓴다). 접두사가 force/ 라 위와 다르다.
    # 안 기다리면 task_compliance_ctrl 첫 호출에서 **영원히 멈춘다** —
    # spin_until_future_complete 에 타임아웃이 없다 (#31)
    ("TaskComplianceCtrl", "dsr_controller2/force/task_compliance_ctrl"),
    ("ReleaseComplianceCtrl", "dsr_controller2/force/release_compliance_ctrl"),
    ("SetDesiredForce", "dsr_controller2/force/set_desired_force"),
    ("ReleaseForce", "dsr_controller2/force/release_force"),
    ("CheckPositionCondition", "dsr_controller2/force/check_position_condition"),
    ("CheckForceCondition", "dsr_controller2/force/check_force_condition"),
]


def _wait_services(node, robot_id, timeout_sec=30.0, logger=None, report=print):
    """
    DSR_ROBOT2 의 첫 호출 전에 서비스 디스커버리를 기다린다 (이슈 #21·#23).

    DSR_ROBOT2 는 기다리지 않고 바로 부른다. 브링업 직후처럼 아직 디스커버리가
    안 끝난 시점에 부르면 spin_until_future_complete 에 타임아웃이 없어 **영원히
    멈춘다.** 2026-09-20 로봇 PC 에서 실제로 겪었다.

    반드시 **DSR 이 쓰는 그 노드**로 기다려야 한다. 다른 노드로 기다리면 그 노드만
    디스커버리가 끝나고 DSR 노드는 아직이라 똑같이 멈춘다.

    디스커버리가 불안정해서(#23) 조용히 수십 초 막혀 있으면 멈춘 건지 기다리는
    중인지 구분이 안 된다. 5초마다 지금 몇 개가 보이는지 알려준다.
    """
    import dsr_msgs2.srv as srv
    pending = [(getattr(srv, t), n) for t, n in _WAIT_SERVICES]
    t0 = time.monotonic()
    while pending:
        still = []
        for srv_type, name in pending:
            cli = node.create_client(srv_type, name)
            try:
                if not cli.wait_for_service(timeout_sec=0.5):
                    still.append((srv_type, name))
            finally:
                node.destroy_client(cli)
        pending = still
        if not pending:
            break
        elapsed = time.monotonic() - t0
        if elapsed >= timeout_sec:
            raise MotionError(
                f"서비스가 안 보인다 ({timeout_sec:.0f}초): "
                + ", ".join(f"/{robot_id}/{n}" for _, n in pending)
                + ". 브링업이 떠 있는지, ROS_DOMAIN_ID·RMW·ROS_DISCOVERY_SERVER 가"
                " 브링업 터미널과 같은지 확인하라. 디스커버리 자체가 불안정할 수도"
                " 있다 — tools/tcp_diag.py 로 더 길게 볼 수 있다 (이슈 #21, #23)")
        if report and int(elapsed) % 5 < 1:
            report(f"  서비스 대기 {elapsed:.0f}/{timeout_sec:.0f}초 — "
                   f"{len(_WAIT_SERVICES) - len(pending)}/{len(_WAIT_SERVICES)}개 보임, "
                   f"전체 {len(node.get_service_names_and_types())}개")
        time.sleep(1.0)
    if logger:
        logger.info(f"서비스 디스커버리 확인 ({len(_WAIT_SERVICES)}개)")
    time.sleep(1.0)   # DSR_ROBOT2 가 자기 클라이언트를 만들 여유


def _read_named(kind):
    """
    활성 tool/tcp 이름을 읽는다. 못 읽으면 None.

    두산 래퍼(`get_tool`/`get_tcp`)에 두 가지 함정이 있어서 그대로 못 쓴다 —

    1. **타입이 섞인다.** 정상이면 `result.info`(str)지만 `result == None` 이면
       `-1`(int)을 돌려준다. 문자열로 비교하는 쪽은 -1 을 만나면 조용히 "안 맞음"이
       된다. 그래서 str 이 아니면 None 으로 본다.
    2. **`ret` 이 unbound 될 수 있다.** 서비스 호출이 예외로 실패하면 except 절이
       로그만 찍고 `ret` 을 안 채워 `return ret` 에서 UnboundLocalError 가 난다
       (DSR_ROBOT2.py 6312·6362행). `-1` 이 아니라 파이썬 예외다. 그래서 감싼다.

    응답의 `success` 는 **보지 않는다.** 컨트롤러가 `res->success = true` 로
    하드코딩한다 (dsr_controller2.cpp 1940·1969행). 실패를 표현할 방법이 없으니
    판단은 `info` 문자열로만 한다.
    """
    api = _require_api()
    try:
        got = getattr(api, f"get_{kind}")()
    except Exception as e:                       # noqa: BLE001 — 래퍼가 뭘 던질지 모른다
        raise MotionError(f"get_{kind}() 호출이 실패했다: {e!r}")
    return got if isinstance(got, str) else None


def _ensure_named(kind, want, logger=None):
    """
    활성 tool/tcp 를 want 로 **반드시 설정하고**, 읽어서 확인한다 (이슈 #26·#23).

    배포용이라 펜던트 설정에 기대지 않는다. 설정이 안 된 로봇에 가져다 놓아도
    같은 코드가 돌아야 하므로, 이미 맞아 보여도 `set` 을 생략하지 않는다.
    기준 이름은 `move.py` 에서 동작이 확인된 것과 같다 —
    tool `"Tool Weight"`, tcp `"GripperDA_v1"`.

    판정은 **`set` 의 반환값이 아니라 `get` 결과로** 한다. 두 가지 이유다 —
      · 펜던트가 이미 켜 둔 경우 `set` 이 -1 로 거부돼도 좌표는 맞다 (이슈 #23).
        2026-09-19 페그 교시가 그 상태였다
      · `set` 성공만 믿으면 컨트롤러가 다른 이름을 켜 둔 경우를 못 잡는다

    못 맞추면 **움직이기 전에 멈춘다.** 페그 좌표가 GripperDA_v1(툴 Z 208mm)
    기준이라, TCP 가 안 맞으면 208mm 어긋난 곳으로 직선 진입한다. 헤드스톡 쪽이다.
    """
    if not want:
        return
    api = _require_api()
    before = _read_named(kind)

    ret, err = None, None
    try:
        ret = getattr(api, f"set_{kind}")(want)
    except Exception as e:                       # noqa: BLE001 — UnboundLocalError 포함
        err = e                                  # 실패해도 아래 확인으로 판정한다

    now = _read_named(kind)
    if now == want:
        if logger:
            logger.info(f"{kind}='{want}' 확인 (set 반환 {ret}, 직전 '{before}')")
        return

    raise MotionError(
        f"{kind} 를 '{want}' 로 못 맞췄다 (set 반환 {ret}"
        + (f", 예외 {err!r}" if err is not None else "")
        + f", 현재 '{now}'). 펜던트에 그 이름이 등록돼 있는지, 설정 변경 제어권이"
        " 로봇에 있는지 확인하라. 교시 좌표가 그 기준이라 다른 기준으로 움직이면"
        " 208mm 어긋난 곳으로 들어간다 (이슈 #23, #26)")


def ensure_tcp(name=None, logger=None):
    """
    움직이기 직전에 이 동작이 쓸 TCP 를 켠다 (이슈 #21).

    교시 좌표는 읽을 때 켜져 있던 TCP 와 짝이라, 페그와 발현이 서로 다른 기준을
    쓴다. 동작마다 자기 것을 켜면 순서가 섞여도 안전하다. 비용은 호출 한 번이다.
    name 을 안 주면 connect() 가 받은 robot.tcp 를 쓴다.
    """
    if name is None:
        name = (_robot_cfg or {}).get("tcp")
    _ensure_named("tcp", name, logger=logger)


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


def turn_motion(motion):
    """
    회전용 속도로 바꾼 설정 사본.

    페그를 돌리는 건 줄에 장력을 거는 동작이라 이동보다 느려야 한다.
    토크를 읽어 파지를 확인하는 것도 이 구간이다(CLAUDE.md 3.6).
    turn_vel/turn_acc 가 없으면 이동 속도를 그대로 쓴다.
    """
    out = dict(motion)
    out["vel"] = motion.get("turn_vel", motion.get("vel", 30))
    out["acc"] = motion.get("turn_acc", motion.get("acc", 30))
    return out


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


def pluck_string(pose, motion, standby=None):
    """
    줄 튕기기 1차 구현 (위치 제어만). 힘 제어·피크 집기는 2단계에서 붙인다.

    pose    — 발현 접촉 자세. 작업 좌표(**pose.task**, YAML 의 x: Base 절대 [X,Y,Z,A,B,C],
              교시 TCP 기준)여야 한다. 접촉점 하나만 교시하고 위 대기점은 여기서 계산한다.
              관절각(j)만 있으면 "위에서 접근"을 만들 수 없어 거부한다.
              pose.stroke_sign (+1/-1, 없으면 +1) — 이 줄의 스트로크 방향. 6·5번(저음 쪽)과
              2·1번(고음 쪽)은 빈 공간이 반대라 훑는 방향이 반대다 (2026-09-20 실기 확인).
    motion  — teach_points.yaml 의 motion 절. 아래 pluck_* 키가 없으면 기본값을 쓴다
              (기본값은 pluck_test.py 에서 검증한 값).
    standby — 기타 위 대기 자세(Pose, joints 또는 task). 주면 발현 전후에 들른다.
              joints 면 movej, task(x) 면 Base 직선(movel)으로 간다.

    순서 (성현 2026-09-19, 2026-09-21 대기 자세 좌표화):
      대기 자세 → 접촉점 위 pluck_up_mm → 접촉점(느리게) → 옆으로 스트로크
      → 즉시 후퇴(5mm 이상) → 접촉점 위 → 대기 자세
    """
    _require_api()
    ensure_tcp()                                        # 발현 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    x = getattr(pose, "task", None)
    if not x or len(x) != 6:
        raise MotionError(f"{pose.name}: 발현 자세는 x(Base posx 6개)로 적어야 한다. "
                          "접촉점 위로 접근하려면 작업 좌표가 필요하다")

    up_mm = float(motion.get("pluck_up_mm", 20.0))
    stroke = list(motion.get("pluck_stroke_tool", [-8.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    sign = getattr(pose, "stroke_sign", 1) or 1
    if sign not in (1, -1):
        raise MotionError(f"{pose.name}: stroke_sign 은 +1 또는 -1 이어야 한다 (지금 {sign!r})")
    stroke = [float(v) * sign for v in stroke[:3]] + [float(v) for v in stroke[3:]]
    retract = list(motion.get("pluck_retract_tool", [0.0, 0.0, -8.0, 0.0, 0.0, 0.0]))
    v_app, a_app = motion.get("pluck_vel_approach", 100), motion.get("pluck_acc_approach", 100)
    v_fine, a_fine = motion.get("pluck_vel_fine", 20), motion.get("pluck_acc_fine", 50)
    v_str, a_str = motion.get("pluck_vel_stroke", 1000), motion.get("pluck_acc_stroke", 2000)

    if _retract_depth_mm(retract) < 5.0:
        raise MotionError("후퇴 거리가 5mm 미만이다 (CLAUDE.md 3.4). pluck_retract_tool 을 확인하라")

    contact = list(x)
    above = list(x)
    above[2] += up_mm                                   # Base +Z

    _goto_standby(standby, motion, v_app, a_app)        # 기타 위 대기
    _movel_base(above, v_app, a_app)                    # 접촉점 위 2cm
    _movel_base(contact, v_fine, a_fine)                # 접촉점 (느리게)
    _movel_tool_rel(stroke, v_str, a_str, radius=3)     # 옆으로 살짝 → 발현
    _movel_tool_rel(retract, v_str, a_str)              # 즉시 후퇴 (버징 방지)
    _movel_base(above, v_app, a_app)                    # 다시 2cm 위
    _goto_standby(standby, motion, v_app, a_app)        # 기타 위 대기


def _goto_standby(standby, motion, vel, acc):
    """Standby 가 있으면 그리로 간다. joints 면 movej, task(x) 면 Base 직선(movel)."""
    if standby is None:
        return
    if standby.kind == "joint":
        _movej(standby.joints, motion)
    else:
        _movel_base(standby.task, vel, acc)


def _pick_points(holder, motion):
    """거치대 접촉점과 그 위 대기점(Base +Z pick_up_mm). holder 는 작업 좌표(x)여야 한다."""
    x = getattr(holder, "task", None)
    if not x or len(x) != 6:
        raise MotionError(f"{holder.name}: 거치대 자세는 x(Base posx 6개)로 적어야 한다. "
                          "위에서 내려가려면 작업 좌표가 필요하다")
    up_mm = float(motion.get("pick_up_mm", 50.0))
    contact = list(x)
    above = list(x)
    above[2] += up_mm
    return contact, above


def pick_up(holder, motion, standby=None, via=None):
    """
    거치대에서 피크를 집는다 (성현 2026-09-20).

    순서: (via 가 있으면) 경유 자세 → 그리퍼 열기 → 거치대 위 pick_up_mm
          → 거치대(느리게) → 그리퍼 닫기(프리셋) → 거치대 위
          → (standby 가 있으면) 기타 위 대기 자세

    via — 거치대로 내려가기 전에 먼저 들르는 안전 경유 관절 자세(Pose, joints).
          pick_return 의 via 와 같은 이유인데 **방향이 반대다.** 피크를 가지러
          갈 때 팔은 직전 동작이 남긴 자리에 있다 — 페그 조절 직후면 헤드스톡
          쪽 접근점이다. 거기서 거치대 위로 바로 movel 하면 기타 넥을 가로지른다.
          2026-09-19 충돌 검증은 페그↔페그와 경유지뿐이라 이 경로는 본 적이 없다.
          관절 자세를 한 번 거쳐 직선 구간을 거치대 근처로 좁힌다 (#34 후속)

    holder  — 거치대 자세 (teach_points.Pose, x). holder.grip_preset 이 있으면 그 프리셋으로,
              없으면 motion["pick_preset"], 그것도 없으면 "close" 로 잡는다.
              폭·힘은 WebLogic 에 박힌 값이라 코드가 못 정한다 (gripper.py).
    파지 확인은 하지 않는다 (CLAUDE.md 3.6, 잡았다고 가정).

    RG2.select() 의 settle_sec(기본 0.3초)만으로는 부족해 집자마자 들어 올리는 동작이
    물리적으로 안 끝난 그리퍼를 밀고 올라갔다 (2026-09-20 실기). 그래서 집은 뒤
    motion["pick_settle_sec"](기본 1.0초)만큼 한 번 더 멈췄다 움직인다.
    """
    _require_api()
    ensure_tcp()                                         # 거치대 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    contact, above = _pick_points(holder, motion)
    preset = getattr(holder, "grip_preset", None) or motion.get("pick_preset", "close")
    v_app, a_app = motion.get("pick_vel_approach", 100), motion.get("pick_acc_approach", 100)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)
    settle_sec = float(motion.get("pick_settle_sec", 1.0))

    if via is not None:
        _movej(via.joints, motion)                      # 경유 자세 (관절 보간)
    _release()                                          # 열고 간다
    _movel_base(above, v_app, a_app)                    # 거치대 위 5cm
    _movel_base(contact, v_fine, a_fine)                # 내려가서
    _grip(preset)                                       # 집는다
    time.sleep(settle_sec)                              # 그리퍼가 물리적으로 다 닫히기를 기다린다
    _movel_base(above, v_app, a_app)                    # 들어 올린다
    if standby is not None:
        _movej(standby.joints, motion)                  # 기타 위 대기


def pick_return(holder, motion, via=None, home=None):
    """
    피크를 거치대에 돌려놓는다.

    via — 거치대로 내려가기 전에 먼저 들르는 안전 경유 관절 자세(Pose, joints).
          발현 직후엔 팔이 기타 위 대기 자세(pluck_standby)에 있는데, 거기서
          거치대 위로 바로 movel 하는 경로는 검증된 적이 없다. frames.standby
          를 경유해 안전한 관절 자세를 한 번 거친 뒤 내려간다.

    순서: (via 가 있으면) 경유 자세 → 거치대 위 pick_up_mm → 거치대(느리게)
          → 그리퍼 열기 → 거치대 위 → (home 이 있으면) 홈 자세
    """
    _require_api()
    ensure_tcp()                                         # 거치대 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    contact, above = _pick_points(holder, motion)
    v_app, a_app = motion.get("pick_vel_approach", 100), motion.get("pick_acc_approach", 100)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)

    if via is not None:
        _movej(via.joints, motion)                      # 경유 자세
    _movel_base(above, v_app, a_app)                    # 거치대 위 5cm
    _movel_base(contact, v_fine, a_fine)                # 내려가서
    _release()                                          # 놓는다
    _movel_base(above, v_app, a_app)                    # 빈손으로 올라온다
    if home is not None:
        _movej(home.joints, motion)                     # 홈


def _retract_depth_mm(delta):
    """후퇴 벡터의 병진 크기(mm)."""
    return sum(float(v) ** 2 for v in delta[:3]) ** 0.5


def _movel_base(pos, vel, acc):
    """직선 이동 (Base 절대 posx). 속도를 직접 준다. _movel 은 motion 절의 travel_* 를 쓴다."""
    api = _require_api()
    from DR_common2 import posx
    return api.movel(posx(list(pos)), vel=vel, acc=acc, ref=api.DR_BASE)


def _movel_tool_rel(delta, vel, acc, radius=0):
    """
    Tool(TCP) 좌표 기준 상대 직선 이동. 스트로크·후퇴에 쓴다.

    피크 자세를 기준으로 움직이므로 기타가 조금 옮겨져도 방향이 유지된다.
    radius 를 주면 다음 모션과 블렌딩(멈춤 없이 이어짐).
    """
    api = _require_api()
    from DR_common2 import posx
    return api.movel(posx(list(delta)), vel=vel, acc=acc,
                     ref=api.DR_TOOL, mod=api.DR_MV_MOD_REL, radius=radius)


def read_force(ref=None):
    """
    툴에 걸리는 외력 [Fx,Fy,Fz,Mx,My,Mz]. ref 를 안 주면 Tool 기준.

    페그 진입 감시는 BASE 기준으로 본다 — 접근축이 base ±Y 라 그 축 성분만
    보면 되기 때문이다.
    """
    api = _require_api()
    return api.get_tool_force(api.DR_TOOL if ref is None else ref)


def read_posx():
    """현재 TCP 자세 (Base). 교시값 기록·로그용. 반환 (posx, sol)."""
    api = _require_api()
    return api.get_current_posx(api.DR_BASE)


def _rolled(pose, roll_deg):
    """
    손잡이 각만큼 손목을 돌린 파지 자세를 만든다 (이슈 #31).

    툴 Z 둘레 회전이라 손끝 위치도 접근축도 안 바뀐다. 손가락 방향만 돈다.
    관절각(j)으로 적힌 자세에는 쓸 수 없다 — 페그는 전부 작업 좌표(x)다.
    """
    if not roll_deg:
        return pose
    if pose.kind != "task":
        raise MotionError(
            f"{pose.name} 이 관절각으로 적혀 있어 손잡이 각을 못 반영한다. "
            "페그 자세는 작업 좌표(x)여야 한다 (#31)")
    from tunio_robot.kinematics import roll_about_tool_z
    turned = TP.Pose(pose.name, pose.taught,
                     task=roll_about_tool_z(pose.task, roll_deg), tcp=pose.tcp)
    turned.sol = getattr(pose, "sol", None)
    return turned


def compliance(motion, on=True):
    """
    순응제어를 켜고 끈다 (이슈 #31).

    **각도 정렬용이 아니다.** 손목이 페그보다 1400배 뻣뻣해서 정렬에는 기여하지
    못한다 — 닿으면 페그가 돌지 손목이 지지 않는다. 용도는 **삽입축 밀림 방지**다.
    네 페그의 접근축이 전부 base ±Y 라, Y 를 무르게 두면 걸렸을 때 로봇이 물러나
    기타를 안 민다.

    stx 는 [X, Y, Z, Rx, Ry, Rz] 강성이고 기준계는 BASE 다
    (DSR_ROBOT2 의 ref = _g_coord, 기본값 DR_BASE).
    """
    api = _require_api()
    if not on:
        return api.release_compliance_ctrl()
    stx = motion.get("compliance_stx")
    if not stx:
        return None
    return api.task_compliance_ctrl(list(stx))


def _cond_met(ret):
    """
    조건 검사 반환을 뜻으로 옮긴다 (진릿값으로 쓰면 뒤집힌다).

    두산 래퍼가 `ret = 0 if result.success else -1` 로 돌려준다
    (DSR_ROBOT2.py 5470·5556행 부근). 즉 **조건 충족이 0, 미충족이 -1** 이다.
    파이썬에서 0 은 거짓이고 -1 은 참이라, `if check_force_condition(...)` 로 쓰면
    정확히 반대로 동작한다. 2026-09-21 실물에서 진입이 곧바로 "도달" 로 끝나
    로봇이 페그로 들어가지도 않고 그리퍼만 여닫았다.

    -1 은 "조건 미충족" 과 "호출 실패" 를 겸한다. 둘을 못 가르므로 미충족으로 본다.
    """
    return ret == 0


def _insert_force_controlled(pose, motion, handle, logger=None):
    """
    페그 앞 짧은 구간을 **힘 상한을 정해 놓고** 더듬어 들어간다 (이슈 #31).

    먼 구간(접근점 ~ search_mm)은 위치 제어로 빠르게 간다. 힘 제어는 접촉
    탐색용이지 이동용이 아니다 — 3N 으로 100mm 를 가면 느리기만 하고, 손잡이와
    부딪힐 수 있는 건 페그 바로 앞뿐이다.

    왜 위치 제어로 나눠 들어가면 안 되나
      위치를 명령하면 로봇은 그 자리까지 **어떻게든 간다.** 막혀 있으면 밀어버린다.
      걸음을 잘게 쪼개고 힘을 확인해도 "밀고 나서 확인"이라 이미 늦다.
      2026-09-21 실물에서 기타가 밀렸다. 순응 강성 0.2 N/mm 에 걸음 12.5mm 라
      한 걸음에 2.5N 밖에 안 쌓여 8N 문턱에 영영 못 닿았고, 그대로 밀고 들어갔다.
      강성을 더 낮추는 건 답이 아니다 — 낮출수록 힘이 늦게 쌓여 감지가 더 늦어진다.

    그래서 힘을 명령한다
      set_desired_force 로 접근축 방향에 상한을 걸면 로봇이 **그 힘까지만 민다.**
      감지가 아니라 제어다. 기타가 몇 N 에서 밀리는지 몰라도 상한을 낮게 잡으면
      그보다 세게 미는 일이 없다. 나사로 고정된 게 아니라 마찰이 매번 다른
      상황에 맞는 방식이다.

    끝나는 조건은 둘
      · 목표 깊이 도달 — 손잡이를 무사히 지났다. 물면 된다
      · 힘이 상한 — 뭔가에 막혔다. 이미 안 밀고 버티는 중이라 기타는 안 밀린다

    왜 check_*_condition 을 안 쓰나
      이 드라이버의 조건 검사에 두 번 속았다 (2026-09-21).
        1. 반환이 `0 if success else -1` 이라 **진릿값으로 쓰면 뒤집힌다**
        2. `check_force_condition(max=2.0)` 은 "2N 을 넘었다" 가 아니라
           **"힘이 2N 이하다"** 는 뜻이다. 허공에서 늘 참이라, 로봇이 가만히
           서 있는 채로 "막혔다" 고 보고했다
      그래서 위치와 힘을 **직접 읽어 우리가 판단한다.** 줄 수는 비슷하고 무슨
      일이 일어나는지가 화면에 보인다. logger 를 주면 1초마다 진행과 힘을 찍는다.
    """
    api = _require_api()
    axis = TP.approach_axis(pose.task)          # 접근축(툴 Z)의 BASE 단위벡터
    push_n = float(handle.get("insert_force_n", 3.0))
    limit_n = float(handle.get("contact_force_n", 2.0))
    tol_mm = float(handle.get("arrive_tol_mm", 2.0))
    timeout = float(handle.get("insert_timeout_sec", 20.0))

    # 네 페그 모두 접근축이 base ±Y 다(실측 |Y| >= 0.998). 지배축을 골라 쓴다
    k = max(range(3), key=lambda i: abs(axis[i]))
    dir_vec = [0.0] * 6
    dir_vec[k] = 1.0
    fd = [0.0] * 6
    fd[k] = push_n * (1.0 if axis[k] > 0 else -1.0)

    # 먼 구간(접근점 ~ search_mm)은 **부르는 쪽이 순응을 켜기 전에** 끝내 놓는다.
    # 여기서 하면 안 된다 — 순응이 켜진 채 위치 명령을 주면 스프링이 흡수해서
    # 팔이 그 자리에 선다. 2026-09-21 실기에서 search_mm 을 8 로 줬는데도
    # 힘 제어가 99.8mm 를 가려 했다. 위치 이동이 통째로 안 먹힌 것이다 (#31).
    target = pose.task[k]
    start = read_posx()[0][k]
    need = abs(target - start)                  # 남은 거리(mm). 힘 제어로 갈 구간

    # ── 기준선을 뺀다 (2026-09-21 실측으로 드러난 것) ────────────────────────
    # 밀기 전에 이미 힘이 읽힌다. 6번 실기에서 **안 움직였는데 1.2N** 이었다.
    # 툴 무게·자세·순응제어가 얹히는 몫이라 자세마다 다르다. 절댓값으로 문턱을
    # 재면 그 몫이 예산을 통째로 먹는다 — 문턱 2.0N 에 여유가 0.8N 밖에 안 남았다.
    #
    # 그리고 순응 스프링이 **허공에서도** 반력을 만든다. Y 강성 200 N/m = 0.2 N/mm
    # 라 2mm 만 밀려도 0.4N 이 붙는다. 닿은 게 아닌데 닿았다고 읽힌다.
    # 그래서 판정은 **기준선 대비 증가분**으로 하고, 스프링 몫도 빼 준다.
    base_f = abs(read_force(api.DR_BASE)[k])
    stx = motion.get("compliance_stx") or []
    k_n_per_mm = (float(stx[k]) / 1000.0) if len(stx) > k else 0.0   # N/m -> N/mm
    if logger:
        logger.info(f"기준선 {'XYZ'[k]} 축 {base_f:.2f}N "
                    f"(순응 {k_n_per_mm * 1000:.0f} N/m = {k_n_per_mm:.2f} N/mm)")
        reach = (push_n - base_f) / k_n_per_mm if k_n_per_mm > 0 else float("inf")
        if reach < need:
            logger.warn(f"{push_n:.1f}N 으로는 스프링을 이기고 {reach:.0f}mm 까지만 간다 "
                        f"— {need:.0f}mm 가 필요하다. search_mm 을 줄이거나 "
                        f"insert_force_n 을 올려라")

    api.set_desired_force(fd, dir_vec, mod=api.DR_FC_MOD_REL)
    t0 = time.monotonic()
    said = 0.0
    try:
        while True:
            pos = read_posx()[0][k]
            raw = abs(read_force(api.DR_BASE)[k])
            gone = (pos - start) * (1.0 if axis[k] > 0 else -1.0)
            # 기준선과 스프링 몫을 뺀 나머지가 "뭔가에 닿아서 생긴 힘" 이다
            spring = k_n_per_mm * max(0.0, gone)
            f = max(0.0, raw - base_f - spring)

            if logger and time.monotonic() - said > 1.0:
                logger.info(f"진입 {gone:+.1f}/{need:.1f}mm, {'XYZ'[k]} 축 "
                            f"접촉 {f:.2f}N (읽은값 {raw:.2f} - 기준선 {base_f:.2f} "
                            f"- 스프링 {spring:.2f})")
                said = time.monotonic()

            if abs(pos - target) <= tol_mm or gone >= need:
                if logger:
                    logger.info(f"{pose.name} 진입 완료 ({gone:+.1f}mm, 접촉 {f:.2f}N)")
                return
            if f >= limit_n:
                raise MotionError(
                    f"{pose.name}: {gone:+.1f}mm 들어가다 막혔다 ({'XYZ'[k]} 축 접촉 "
                    f"{f:.2f}N, 문턱 {limit_n:.1f}N. 읽은값 {raw:.2f} - 기준선 "
                    f"{base_f:.2f} - 스프링 {spring:.2f}). 손잡이가 서 있어 그리퍼가 "
                    "못 들어간다. 그 페그를 손으로 눕힌 뒤 다시 시도하라 (#31)")
            if time.monotonic() - t0 > timeout:
                reach = ((push_n - base_f) / k_n_per_mm) if k_n_per_mm > 0 else 0.0
                raise MotionError(
                    f"{pose.name}: {timeout:.0f}초 동안 {gone:+.1f}mm 밖에 못 갔다 "
                    f"(접촉 {f:.2f}N). 미는 힘 {push_n:.1f}N 이 순응 스프링"
                    f"({k_n_per_mm:.2f} N/mm)을 이기고 갈 수 있는 거리는 "
                    f"{reach:.0f}mm 다. search_mm 을 줄이거나 insert_force_n 을 올려라")
            time.sleep(0.05)
    finally:
        api.release_force()                     # 힘 제어는 반드시 푼다


def register_peg(pose, motion, handle, read_width, logger=None,
                 use_force=True, assume_grip=False):
    """
    Phase 0 — 페그를 물어서 손잡이 각을 확정하고 그 각을 돌려준다 (#31).

    로봇은 손잡이가 지금 어느 각으로 서 있는지 모른다. 사람이 마지막으로 돌려놓은
    상태라 매번 다르다. 작업 전에 줄마다 한 번씩 잡아보고 각을 기록한다. 그 뒤로는
    adjust_peg 가 이 각으로 들어가므로 물리면서 되돌려지는 일이 없다.

    pose       — 페그 파지 자세
    handle     — teach_points 의 peg_handle 절
    read_width — 지금 손가락 폭(mm)을 돌려주는 함수 (grip_width.make_reader)
    use_force  — False 면 힘 제어 없이 그냥 직선으로 들어간다. **사람이 손잡이를
                 눕혀 놓고 지켜보는 측정 세션용**이다. 아직 실물 검증 전이기도 하다 (#31).
                 **순응제어는 이 값과 무관하게 항상 켠다** — 끄면 로봇이 뻣뻣해져
                 오히려 더 세게 민다 (PR #37 리뷰)
    assume_grip — True 면 폭을 안 읽고 물렸다고 본다. Modbus 가 안 붙는 PC 에서
                 측정만 할 때다. **빈손·사점을 못 가른다.** 눈으로 확인해야 한다

    ## 왜 꽉 무나 (probe 프리셋, 15mm)

    작업용 peg(21mm)는 손잡이 두께(19.6mm)보다 넓어 **닿지도 않는다.** 각이 안
    잡히고, 폭도 물림과 빈손이 똑같이 읽혀 구분이 안 된다. 꽉 물어야 손잡이가
    손가락과 평행하게 강제되고 그 각이 곧 답이 된다. 여기서는 돌리지 않으므로
    꽉 물어도 안전모드 조건이 아니다.

    ## 진입은 힘 제어로 한다

    위치를 명령하면 막혀 있어도 밀고 들어간다. 2026-09-21 실물에서 기타가 밀렸다.
    `set_desired_force` 로 상한을 걸면 **그 힘까지만 민다.** 감지가 아니라 제어라,
    기타가 몇 N 에서 밀리는지 몰라도 낮게 잡으면 그보다 세게 미는 일이 없다.

    ## 왜 사점에서 돌려 빠져나오지 않나

    **그럴 공간이 없다.** 열기 폭이 38mm 인데 손잡이가 59도쯤 서면 38.2mm 가
    필요하다. 폭을 키우면 옆 페그를 친다(41·42·45 전부 쳤다). 창이 겹치지 않는다.

    그래서 사람이 작업 전에 세로로 선 손잡이를 눕혀 둔다. 조율 전이라 음이
    틀어져도 상관없다. 빠뜨리면 여기서 접촉으로 알아채고 **어느 줄인지 알리며
    멈춘다.** 사람이 네 줄을 검사하는 게 아니라 로봇이 짚어 준다.
    """
    _require_api()
    ensure_tcp()
    probe = handle.get("probe_preset", "close")
    thresholds = handle["thresholds_mm"]
    from tunio_robot.grip_width import classify

    _grip("open")
    approach_mm = float(motion.get("approach_mm", 100.0))
    back = TP.backed_off(pose.task, approach_mm)
    _movejx(back, motion, sol=getattr(pose, "sol", None))

    # 먼 구간은 **순응을 켜기 전에** 위치 제어로 간다. 순서가 중요하다 —
    # 순응이 켜진 뒤에 위치 명령을 주면 스프링이 흡수해 팔이 그 자리에 선다.
    # 2026-09-21 실기에서 search_mm 8 을 줬는데 힘 제어가 99.8mm 를 가려 했고,
    # 4N 으로 20초에 7.5mm 밖에 못 갔다. 먼 구간은 딱딱할 때 끝내야 한다.
    search_mm = float(handle.get("search_mm", 25.0))
    if use_force and search_mm < approach_mm:
        if logger:
            logger.info(f"페그 앞 {search_mm:.0f}mm 까지 위치 제어로 간다 "
                        f"(순응 켜기 전)")
        _movel(TP.backed_off(pose.task, search_mm), motion)

    # 순응은 **항상** 켠다. 힘 제어와 다른 것이다 —
    #   힘 제어  "이 힘까지만 민다"      (미검증, use_force 로 끌 수 있다)
    #   순응제어  "걸리면 물러난다"       (끄면 로봇이 뻣뻣해져 오히려 더 세게 민다)
    # use_force 의 목적은 미검증 힘 제어를 안 태우는 것이지 안전망을 걷는 게 아니다.
    # 순응만으로도 미는 힘이 강성 × 남은 거리로 묶인다 (PR #37 리뷰, 정동휘)
    compliance(motion, on=True)
    try:
        if use_force:
            _insert_force_controlled(pose, motion, handle, logger=logger)
        else:
            # 사람이 손잡이를 눕혀 놓고 보고 있는 세션. 그냥 들어간다
            if logger:
                logger.warn("힘 제어를 건너뛴다 — 손잡이가 눕혀져 있는지 사람이 볼 것")
            _movel(pose.task, motion)
        _grip(probe)                             # 꽉 문다. 여기서 각이 확정된다
        if assume_grip:
            if logger:
                logger.warn("폭을 안 읽는다 — 물렸는지 눈으로 확인할 것 (빈손·사점 못 가림)")
            return 0.0
        width = read_width()
        verdict = classify(width, thresholds)
        if logger:
            logger.info(f"{pose.name} 등록 — 폭 {width:.1f}mm, {verdict}")
        if verdict == "empty":
            raise MotionError(
                f"{pose.name}: 빈손이다 (폭 {width:.1f}mm). 페그를 빗나갔다. "
                "기타가 밀렸거나 교시 좌표가 어긋났다 (#31)")
        if verdict == "dead":
            raise MotionError(
                f"{pose.name}: 손잡이가 서 있어 못 물었다 (폭 {width:.1f}mm). "
                "돌려서 빠져나올 공간이 없다 — 열기 폭 38mm 로는 59도쯤 선 손잡이를 "
                "지날 수 없고, 폭을 키우면 옆 페그를 친다. "
                "그 페그를 손으로 눕힌 뒤 다시 시도하라 (#31)")
        # **항상 0 이다.** 이 함수는 각을 '재는' 게 아니라 '맞추는' 것이다 —
        # 꽉 물면 손잡이가 손가락에 맞춰 정렬되므로, 물렸다는 건 곧 교시 자세와
        # 같아졌다는 뜻이다. 나중에 "왜 늘 0이지" 하고 고치려 들지 말 것.
        # 이후 adjust_peg 가 돌린 만큼이 peg_state 에 누적된다
        return 0.0
    finally:
        _grip("open")
        compliance(motion, on=False)             # 이동은 뻣뻣하게. 순응을 켠 채 다니지 않는다
        _retreat(pose, motion)


def adjust_peg(pose, joint, delta_deg, motion, roll_deg=0.0):
    """
    페그 조절. 받은 관절을 받은 각도만큼 돌린다.

    pose      — 페그 파지 자세 (teach_points.Pose)
    joint     — 돌릴 관절 번호 1~6
    delta_deg — 부호 붙은 회전량. 부호 변환은 teach_points.joint_delta 가 이미 했다
    roll_deg  — 손잡이가 교시 자세에서 얼마나 돌아가 있는지(도). 그만큼 손목을
                더 돌려서 들어간다. 부르는 쪽이 peg_state 에서 꺼내 준다 (#31).
                0 이면 교시 자세 그대로다

    **끝나고 나면 손잡이가 delta_deg 만큼 더 돌아가 있다.** 그 누적은 여기서
    안 한다 — 동작이 중간에 실패하면 얼마나 돌았는지 모르기 때문이다.
    성공을 확인한 쪽(robot_controller)이 peg_state.add_turn 으로 적는다.

    백래시 대응(목표보다 높으면 풀었다 조여 올리기)은 task_manager 가 결정한다
    (명세 9.1 확정). 여기서 조이는 방향 마무리를 덧붙이면 서버가 계산한 각도와
    이중으로 겹쳐 결과가 어긋난다.
    """
    _require_api()
    ensure_tcp()                                    # 페그 좌표는 robot.tcp 기준이다 (이슈 #21)

    # 손잡이 각 추적은 **peg_joint == 6 을 전제로 한다.** J6 만 툴 Z 둘레 회전이라
    # 관절 회전량이 곧 posx 의 C 변화량이다. 다른 관절이면 그 대응이 깨지는데,
    # YAML 은 아직 1~6 을 다 받고(peg_joint 가 "실물 확인 후 확정"이다) 부르는 쪽은
    # 관절 번호를 안 본다. 조용히 틀린 각을 누적하느니 여기서 멈춘다 (#31).
    if roll_deg and joint != 6:
        raise MotionError(
            f"손잡이 각 추적은 peg_joint == 6 을 전제로 한다 (지금 J{joint}). "
            "J6 만 툴 Z 둘레 회전이라 관절 회전량이 곧 C 변화량이다. "
            "다른 관절로 돌리려면 각 추적을 끄고(roll_deg=0) 써라 (#31)")

    entry = _rolled(pose, roll_deg)                 # 손잡이가 돌아가 있으면 맞춰 들어간다 (#31)
    # 돌리고 나면 손목이 그만큼 더 돌아가 있다. 나올 때 원래 자세로 되돌리면
    # 손목이 역회전하면서 아직 손가락 사이에 있는 페그를 다시 돌려 버린다.
    # J6 이 아니면 C 가 안 바뀌므로 들어온 자세 그대로 나간다
    c_delta = delta_deg if joint == 6 else 0.0
    exit_ = _rolled(pose, roll_deg + c_delta)

    # 들어가기 전에 반드시 연다. 직전 동작이 피크를 물고 있었으면 15mm 인 채로
    # 들어가 손잡이를 정면으로 들이받는다. 나올 때는 _release() 가 같은 폭으로 연다.
    # 폭은 gripper.presets.open 이 정한다(지금 38mm). 더 벌리면 옆 페그 손잡이를
    # 치고, 줄이면 눕힌 손잡이도 못 지난다. 창이 5mm 도 안 된다 (#31)
    _grip("open")
    _goto_pose(entry, motion)                       # 물러난 지점 -> 직선으로 페그 앞
    _grip("peg")                                    # 여유를 준 폭으로 파지
    turned = False
    try:
        delta = [0.0] * 6
        delta[joint - 1] = float(delta_deg)
        # 회전은 이동보다 느리게 간다. 줄에 장력을 거는 구간이다
        _movej(delta, turn_motion(motion), relative=True)
        turned = True
    finally:
        _release()                                  # 실패해도 페그는 놓는다
        # 다 돌았으면 돌아간 자세로, 도중에 멈췄으면 들어온 자세로 나온다.
        # 어느 쪽이든 직선 경로 자체는 같다 — 자세만 다르다
        _retreat(exit_ if turned else entry, motion)


def strum_all(string_numbers, points):
    """전체 줄을 훑어 화음. 측정 없는 완료 연출."""
    raise NotImplementedError("성현 담당 — 훑기 동작 교시 후 구현")


def return_home(pose, motion):
    """
    대기 자세 복귀. strum_all 이 화음 연출을 마친 뒤 호출한다.

    TuningState.PHASE_RETURN 을 호출할 전용 Action 이 인터페이스 13개에 없어서
    strum_all 이 복귀까지 책임진다 (2026-09-19 결정). 인터페이스를 다시 열지
    않으려는 선택이고, 기획 Phase 2 에서 화음 연출 직후가 곧 복귀라 순서상으로도
    이어진다.

    한계: 정상 완료 경로에만 있다. 취소·오류로 끝난 경우의 복귀는 수단이 없고
    지금은 사람이 회수한다.
    """
    _require_api()
    _movej(pose.joints, motion)
