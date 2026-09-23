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
import copy
import math
import time

from tunio_robot import teach_points as TP

# 두산 API 핸들. connect() 가 채운다. dry_run 이면 None 그대로다
_api = None
_node = None
_gripper = None
_j6_motion = None       # 실제 노드가 감시 설정을 연결한다. 단독 도구는 기존 경로 유지
_j6_sample = None       # (수신 monotonic 시각, J6). joint_states 콜백만 갱신
_cancel_check = None  # robot_controller의 현재 Action 취소 검사
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
    # tool/tcp 는 MANUAL 왕복 안에서 맞춘다 (아래 _ensure_tool_tcp) —
    # connect() 의 다른 무엇보다도 먼저 끝나야 한다
    _ensure_tool_tcp(robot_cfg, logger=logger)
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
    # tool/tcp 설정은 MANUAL 모드에서만 받아줘서(2026-09-23 tcp_diag.py --manual
    # 실기로 확정) connect() 가 왕복에 쓴다. 못 기다리면 이 두 호출에서도
    # 똑같이 영원히 멈춘다
    ("GetRobotMode", "dsr_controller2/system/get_robot_mode"),
    ("SetRobotMode", "dsr_controller2/system/set_robot_mode"),
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


def _read_mode(api):
    """
    현재 로봇 모드(MANUAL=0 / AUTONOMOUS=1)를 읽는다. 못 읽으면 None.

    get_robot_mode() 도 다른 get_* 와 같은 함정이 있다 — 실패하면 -1(int) 이거나
    UnboundLocalError 다(DSR_ROBOT2.py 648행). 둘 다 None 으로 본다
    (tools/tcp_diag.py 의 read_mode 와 같은 판정 — 이슈 #23 후속).
    """
    try:
        m = api.get_robot_mode()
    except Exception:                            # noqa: BLE001 — UnboundLocalError 포함
        return None
    return m if isinstance(m, int) and m >= 0 else None


def _set_mode(api, want, logger=None):
    """
    로봇 모드를 want 로 바꾸고 **get 으로** 확인한다. 바뀌었으면 True.

    set_robot_mode 의 반환은 0 이 성공 / -1 이 실패로, 다른 set_* 래퍼와 반대
    방향이라 진릿값으로 못 쓴다(tools/tcp_diag.py). 판정은 반드시 get 으로 한다.
    """
    try:
        ret = api.set_robot_mode(want)
    except Exception as e:                       # noqa: BLE001
        ret = None
        err = e
    else:
        err = None
    time.sleep(0.5)                              # 반영될 틈
    now = _read_mode(api)
    ok = (now == want)
    if logger:
        logger.info(f"set_robot_mode({want}) -> {ret}"
                    + (f" 예외 {err!r}" if err is not None else "")
                    + f" (0=성공,-1=실패), 지금 모드 {now}")
    return ok


def _ensure_tool_tcp(robot_cfg, logger=None):
    """
    tool/tcp 를 맞춘다.

    connect() 의 다른 무엇보다 먼저 끝나야 한다 — tool/tcp 가 안 맞은 채로는
    아무것도 움직이면 안 된다.

    설정 변경(set_tool/set_tcp/add_tcp)은 두산 컨트롤러가 **MANUAL 모드에서만**
    받아준다(2026-09-23 `tools/tcp_diag.py --manual` 실기로 확정 — AUTONOMOUS
    에서 셋 다 -1, MANUAL 에서 셋 다 0). `ros2_control`(dsr_hw_interface2.cpp
    279행)이 브링업마다 로봇을 AUTONOMOUS 로 강제 전환해 두므로, 여기서 MANUAL
    로 잠깐 왕복한다.

    왕복이 안 되는 경우(모드를 못 읽음 / 이미 MANUAL / 전환 자체가 거부됨)는
    막지 않고 그냥 진행한다 — `_ensure_named` 가 이미 set 실패를 get 대조로
    넘기게 돼 있어(펜던트가 미리 맞춰 둔 경우), 왕복 없이도 통과할 수 있다.
    왕복했는데 원래 모드로 못 돌아오면, 그건 이후 모든 이동 명령이 거부될 수
    있는 상태라 조용히 넘기지 않고 멈춘다.
    """
    api = _require_api()
    start_mode = _read_mode(api)
    switched = False

    if start_mode is None:
        if logger:
            logger.warn("로봇 모드를 못 읽어 MANUAL 왕복을 건너뛴다 — "
                        "set 이 거부돼도 get 대조로만 판정한다")
    elif start_mode == api.ROBOT_MODE_MANUAL:
        if logger:
            logger.info("이미 MANUAL 모드 — 왕복 없이 진행")
    else:
        switched = _set_mode(api, api.ROBOT_MODE_MANUAL, logger=logger)
        if not switched and logger:
            logger.warn("MANUAL 전환 실패 — 펜던트가 제어권을 쥐고 있을 수 있다. "
                        "왕복 없이 진행한다(set 실패해도 get 대조로 판정)")

    try:
        # 순서는 tool 먼저다 — 무게 설정이 먼저 들어가야 TCP 보정이 제 값을 쓴다
        _ensure_named("tool", robot_cfg.get("tool"), logger=logger)
        _ensure_named("tcp", robot_cfg.get("tcp"), logger=logger)
    finally:
        if switched and not _set_mode(api, start_mode, logger=logger):
            raise MotionError(
                f"MANUAL 왕복 후 원래 모드({start_mode})로 못 돌아왔다. "
                "펜던트에서 로봇 모드를 직접 확인하라 — MANUAL 로 남아 있으면 "
                "이후 이동 명령이 전부 거부될 수 있다")


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
    if _cancel_check is not None:
        from .motion_cancel import GuardedAPI
        return GuardedAPI(_api, _cancel_check)
    return _api


def _moved(ret, what):
    """
    이동 명령의 반환을 검사한다 — 버리면 안 움직여도 아무도 모른다.

    두산 래퍼는 이동도 `0 if success else -1` 로 돌려준다. 지금까지 이 값을
    버리고 있었다 — set_tcp 를 버리던 것과 같은 실수다 (#26). 순응제어가 켜져
    있거나 로봇이 거부하면 -1 이 오는데, 그대로 다음 줄로 넘어가 그리퍼를 닫아
    버린다. 빈손을 물고 "등록됐다" 가 된다.

    그 -1 이 진짜인지 확인했다 (성현, #50) — `DSR_ROBOT2.py` 2941행이
    `ret = 0 if (result.success == True) else -1` 이고, 그 success 는
    `dsr_controller2.cpp:448` movel_cb 이 `Drfl->movel()` 반환을 그대로 실은 값이다.
    `get_tcp` 처럼 true 하드코딩이 아니다 — CLAUDE.md 3.5.1 의 그 경고는 `get_*` 에만
    해당한다. 이 검사는 무력하지 않다.

    ⚠️ 다만 같은 함정이 이동에도 하나 있다 — 서비스 호출이 예외로 실패하면
    `except` 절이 `ret` 을 안 채운 채 `return ret` 을 타서 **-1 이 아니라
    `UnboundLocalError`** 가 난다 (2934~2941행). 그 경우 여기까지 오지 못한다.
    그래서 부르는 쪽은 `MotionError` 만 잡아선 안 된다 (pluck_string 의 스트로크 구간).

    -1 만 실패로 본다. None 이나 다른 값은 그냥 통과시킨다 — 래퍼 판마다 다르고,
    여기서 막는 건 **명시적 거부**다.
    """
    if ret == -1:
        raise MotionError(
            f"{what} 가 거부됐다 (-1). 순응제어가 켜진 채 위치 명령을 줬거나 "
            "로봇이 그 자세를 못 만든다. 안 움직였는데 다음 동작으로 넘어가면 "
            "빈손을 물게 된다")
    return ret


# 이동 속도 배율의 상한 (실기 검증 전 임시값). speed_scale 을 아무리 올려도
# 여기서 잘린다. 가속도 상한은 속도 상한의 2배로 본다 — 이 설정 전체가
# acc = vel x 2 로 짜여 있다 (5/10, 2/4, 100/200).
#
# **관절 이동의 진짜 상한은 손끝 속도다** (#58, 2026-09-21 사용자 지적).
# 같은 관절 속도가 팔 모양에 따라 손끝에서 240배까지 갈린다 —
#   경유지 자세(팔이 수직)    60 deg/s -> 손끝    6 mm/s   (J1 축에서 6mm)
#   6번 페그 자세(팔이 뻗음)  60 deg/s -> 손끝 1446 mm/s   (M0609 상한 1 m/s 초과)
# 그래서 고정된 관절 상한은 어떤 자세에선 위험하고 어떤 자세에선 쓸데없이 느리다.
# _movej 가 이번 이동의 기하를 보고 관절 속도를 직접 낮춘다 (_joint_vel_for).
SPEED_CAP_JOINT = 60.0        # deg/s — 손끝 상한이 먼저 걸리는 게 보통이다
SPEED_CAP_LINEAR = 250.0      # mm/s  — 직선 이동(movel)
# 관절 이동 중 손끝이 낼 수 있는 최고 속도. ISO/TS 15066 협동 감속 기준이 250mm/s 다.
SPEED_CAP_TCP = 250.0         # mm/s
# 지금 관절각을 못 읽어 기하를 못 볼 때. 실측 자세에서 360mm/s 쯤 나오는 값이다.
SPEED_CAP_JOINT_BLIND = 15.0  # deg/s


def _peak_tcp_mm_per_deg(start, goal, n=64):
    """
    가장 많이 도는 관절 1도당 손끝이 최대 몇 mm 움직이는지 잰다.

    movej 는 관절을 동시에 보간하므로 손끝 경로는 직선이 아니고, 속도도 구간마다
    다르다. 경로를 잘게 떠서 최고점을 찾는다. 시간이 아니라 '도당 mm' 로 재기
    때문에 속도 가정이 안 들어간다 — 관절 속도를 곱하면 그대로 mm/s 가 된다.
    """
    span = max(abs(float(goal[i]) - float(start[i])) for i in range(6))
    if span <= 1e-6:
        return 0.0
    from . import kinematics as K
    prev, peak = None, 0.0
    for k in range(n + 1):
        t = k / n
        j = [float(start[i]) + (float(goal[i]) - float(start[i])) * t for i in range(6)]
        tcp = K.fk(j)["tcp"]
        if prev is not None:
            step = sum((tcp[i] - prev[i]) ** 2 for i in range(3)) ** 0.5
            peak = max(peak, step / (span / n))
        prev = tcp
    return peak


def _joint_vel_for(goal, want_vel, tcp_cap=SPEED_CAP_TCP,
                   fallback_vel=SPEED_CAP_JOINT_BLIND):
    """
    손끝이 SPEED_CAP_TCP 를 넘지 않도록 관절 속도를 낮춘다 (#58).

    지금 관절각을 로봇에서 읽어 이번 이동의 기하를 본다. 못 읽으면 기하를 모르므로
    SPEED_CAP_JOINT_BLIND 로 떨어진다 — 모를 때는 느린 쪽이다.
    """
    api = _require_api()
    try:
        start = [float(v) for v in api.get_current_posj()]
        goal = [float(v) for v in goal]
        if len(start) != 6 or len(goal) != 6 or not all(math.isfinite(v) for v in start + goal):
            return min(want_vel, fallback_vel)
    except Exception:
        return min(want_vel, fallback_vel)
    try:
        peak = _peak_tcp_mm_per_deg(start, goal)
    except Exception:
        return min(want_vel, fallback_vel)
    if peak <= 0.0:
        return want_vel                      # 제자리다. 손끝이 안 움직인다
    return min(want_vel, min(tcp_cap, SPEED_CAP_TCP) / peak)


def _transit_speed(motion, vel_key="pick_vel_approach", acc_key="pick_acc_approach"):
    """비접촉 이동의 TCP 목표 속도. 미설정이면 호출 구간의 기존 값을 쓴다."""
    vel = motion.get("transit_tcp_vel", motion.get(vel_key, 100.0))
    acc = motion.get("transit_tcp_acc", motion.get(acc_key, 100.0))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in (vel, acc)):
        raise MotionError("비접촉 이동 속도·가속도는 유한한 양수여야 한다")
    return (_fast(motion, vel, SPEED_CAP_LINEAR),
            _fast(motion, acc, SPEED_CAP_LINEAR * 2))


def _transit_joint_speed(goal, motion, cap):
    """같은 관절 경로에서 목표 TCP 속도로 환산. 경로를 모르면 기존 느린 속도 유지."""
    tcp_vel, tcp_acc = _transit_speed(motion)
    fallback = _fast(motion, motion.get("vel", 30), min(cap, SPEED_CAP_JOINT_BLIND))
    if goal is None:
        return fallback, _fast(motion, motion.get("acc", 30), cap * 2)
    vel = _joint_vel_for(goal, cap, tcp_cap=tcp_vel, fallback_vel=fallback)
    return vel, min(vel * tcp_acc / tcp_vel, cap * 2)


def _speed_scale(motion):
    """이동 속도 배율. 없거나 이상하면 1.0(그대로)."""
    try:
        s = float(motion.get("speed_scale", 1.0))
    except (TypeError, ValueError):
        return 1.0
    return s if s > 0 else 1.0


def _fast(motion, value, cap):
    """
    **이동·접근** 속도에 배율을 먹이고 상한에서 자른다 (#58). 리스트도 받는다.

    검증된 물리값에는 절대 부르지 않는다 —
      turn_vel/turn_acc       페그 회전. 줄에 장력을 거는 동작이라 느린 게 설계다
                              (turn_motion 이 사본에 speed_scale=1.0 을 박아 막는다)
      pluck_vel/acc_stroke    피크가 줄을 긁는 구간. 소리 세기가 여기서 정해진다
                              (2026-09-20 검증값 100/200, PR #54)
      *_vel_fine/*_acc_fine   접촉 직전 마지막 구간. 빠르면 그대로 박는다
      힘 제어 진입             속도가 아니라 힘으로 묶여 있다

    cap 이 튜플이면 칸마다 다른 상한을 쓴다 — travel_vel 이 [직선 mm/s, 회전 deg/s]
    라 상한을 하나로 먹이면 회전이 직선 상한까지 올라간다.
    """
    s = _speed_scale(motion)
    if isinstance(value, (list, tuple)):
        caps = cap if isinstance(cap, tuple) else (cap,) * len(value)
        return [min(float(v) * s, caps[min(i, len(caps) - 1)])
                for i, v in enumerate(value)]
    return min(float(value) * s, cap)


def _guard_j6_goal(joints):
    """미리 계산한 목표 검사. 모션 중 실제 관절값은 joint_states 구독으로 받는다."""
    if not _j6_motion or not _j6_motion.get("j6_recovery_enabled", False):
        return
    sample = _j6_sample
    max_age = float(_j6_motion.get("j6_sample_max_age_sec", 1.0))
    if not math.isfinite(max_age) or max_age <= 0:
        raise MotionError("J6 감시 수신 허용 지연은 유한한 양수여야 한다")
    if sample is None or time.monotonic() - sample[0] > max_age:
        raise MotionError("J6 상시 감시 응답이 없거나 오래됐다 — 이동 중단")
    if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
        raise MotionError("J6 목표 관절각을 확인할 수 없다")
    if abs(joints[5]) >= _j6_threshold(_j6_motion):
        raise MotionError(f"J6 목표 {joints[5]:.1f}도가 회복 기준에 도달한다 — 이동 거부")


def _guard_j6_task(task, sol=None):
    if not _j6_motion or not _j6_motion.get("j6_recovery_enabled", False):
        return
    from DR_common2 import posx
    api = _require_api()
    if sol is None:
        sol = api.get_current_solution_space()
    _guard_j6_goal(list(api.ikin(posx(list(task)), sol)))


def _movej(joints, motion, relative=False):
    """관절 이동. relative=True 면 현재 자세 기준 상대 이동."""
    api = _require_api()
    from DR_common2 import posj
    if relative and _j6_motion:
        current = list(api.get_current_posj())
        _guard_j6_goal([a + b for a, b in zip(current, joints)])
    elif not relative:
        _guard_j6_goal(joints)
    mod = api.DR_MV_MOD_REL if relative else api.DR_MV_MOD_ABS
    vel = _fast(motion, motion.get("vel", 30), SPEED_CAP_JOINT)
    acc = _fast(motion, motion.get("acc", 30), SPEED_CAP_JOINT * 2)
    if not relative and "transit_tcp_vel" in motion:
        vel, acc = _transit_joint_speed(joints, motion, SPEED_CAP_JOINT)
    elif not relative:
        # 상대 이동은 목표 관절각을 여기서 모른다 — 절대 이동에만 건다.
        # 상대 movej 는 페그 회전(turn_motion, 배율 1.0)에만 쓰여서 원래 느리다.
        capped = _joint_vel_for(joints, vel)
        if capped < vel:
            acc *= capped / vel               # 가속도도 같은 비로 내린다
            vel = capped
    return _moved(api.movej(posj(list(joints)), vel=vel, acc=acc, mod=mod, radius=0), "movej")


def _movel(task, motion):
    """작업 좌표로 직선 이동. BASE 기준 [X, Y, Z, A, B, C]."""
    api = _require_api()
    from DR_common2 import posx
    # travel_vel/acc 는 [직선, 회전] 쌍이다 — mm/s 와 deg/s 가 한 리스트에 같이 있다.
    # 상한을 하나로 먹이면 회전이 250deg/s 가 되므로 칸마다 다른 상한을 쓴다.
    caps = (SPEED_CAP_LINEAR, SPEED_CAP_JOINT)
    v = _fast(motion, motion.get("travel_vel", [50.0, 30.0]), caps)
    a = _fast(motion, motion.get("travel_acc", [100.0, 60.0]),
              tuple(c * 2 for c in caps))
    _guard_j6_task(task)
    return _moved(api.movel(posx(list(task)), vel=v, acc=a), "movel")


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
    # movejx 도 관절 보간이지만 목표가 작업 좌표라 목표 관절각을 여기서 모른다.
    # ikin 을 부르면 알 수 있으나 그건 또 한 번의 서비스 왕복이고, 실패하면
    # 이동 자체가 막힌다. 기하를 못 보는 이동이므로 느린 쪽 상한을 쓴다 (#58).
    vel = _fast(motion, motion.get("vel", 30), SPEED_CAP_JOINT_BLIND)
    acc = _fast(motion, motion.get("acc", 30), SPEED_CAP_JOINT_BLIND * 2)
    if "transit_tcp_vel" in motion:
        # 목표 해를 읽어 속도만 환산한다. 경로/sol은 기존 movejx 그대로 유지한다.
        # 실제 보간과 IK의 차이를 고려해 기존 15deg/s 상한은 유지한다.
        try:
            goal = list(api.ikin(posx(list(task)), sol))
        except Exception:
            goal = None
        vel, acc = _transit_joint_speed(goal, motion, SPEED_CAP_JOINT_BLIND)
    _guard_j6_task(task, sol)
    return _moved(api.movejx(posx(list(task)), vel=vel, acc=acc, sol=sol), "movejx")


def turn_motion(motion, pose=None):
    """
    회전용 속도로 바꾼 설정 사본.

    페그를 돌리는 건 줄에 장력을 거는 동작이라 이동보다 느려야 한다.
    토크를 읽어 파지를 확인하는 것도 이 구간이다(CLAUDE.md 3.6).
    turn_vel/turn_acc 가 없으면 이동 속도를 그대로 쓴다.

    **speed_scale 을 1.0 으로 못박는다** (#58). 이 사본은 그대로 _movej 로 넘어가는데,
    거기서 배율이 곱해지면 "이동은 빨라도 회전은 아니다" 는 이 함수의 존재 이유가
    통째로 사라진다. 회전을 빠르게 하려면 turn_vel 을 직접 올릴 것.
    """
    out = dict(motion)
    out.pop("transit_tcp_vel", None)
    out.pop("transit_tcp_acc", None)
    out["vel"] = motion.get("turn_vel", motion.get("vel", 30))
    out["acc"] = motion.get("turn_acc", motion.get("acc", 30))
    # 줄별 값은 페그를 문 회전에만 쓴다. 빈손 되감기는 pose 없이 공통값을 쓴다.
    for key, dest in (("turn_vel", "vel"), ("turn_acc", "acc")):
        value = getattr(pose, key, None)
        if value is not None:
            out[dest] = value
    out["speed_scale"] = 1.0
    return out


def goto_route(route, motion):
    """
    경유지로 간다. null 인 관절은 지금 값을 그대로 둔다.

    관절 이동이라 특이점과 무관하다. 특이점이 문제가 되는 건 직선 이동뿐이다.
    """
    api = _require_api()
    current = list(api.get_current_posj())
    _movej(route.resolve(current), motion)


def _peg_route_required(j1, side, motion):
    defaults = {"right": [-60.0, 80.0], "left": [100.0, 240.0]}
    bounds = motion.get(f"peg_j1_skip_{side}_deg", defaults[side])
    try:
        low, high = map(float, bounds)
        if not (math.isfinite(low) and math.isfinite(high) and low < high):
            raise ValueError()
        if (side == "right" and high >= 90) or (side == "left" and low <= 90):
            raise ValueError()
    except (TypeError, ValueError) as exc:
        raise MotionError(f"페그 {side} J1 생략 범위 오류: {bounds}") from exc
    return j1 is None or not math.isfinite(j1) or not low <= j1 <= high


def approach_peg_side(side, route, motion, logger=None):
    """실제 J1이 해당 쪽의 설정 범위 안일 때만 경유를 생략한다."""
    j1 = _current_j1()
    required = _peg_route_required(j1, side, motion)
    if logger:
        logger.info(f"페그 {side}: J1={j1} — "
                    + ("+90도 경유" if required else "경유 생략"))
    if not required:
        return
    if route is None or route.joints[0] != 90.0:
        raise MotionError("페그 접근에는 J1=+90도인 routes.transit이 필요하다")
    goto_route(route, motion)


def _peg_transfer_floor(motion):
    """허용 높이와 모델 여유. 후자는 실기 검증 전 임시값이다."""
    floor = float(motion.get("pick_to_peg_min_z_mm", 214.0))
    margin = float(motion.get("pick_to_peg_margin_mm", 5.0))
    if not all(math.isfinite(v) and v > 0 for v in (floor, margin)):
        raise MotionError("피크→페그 높이/모델 여유는 유한한 양수여야 한다")
    return floor, margin


def _check_peg_transfer_j1(start, goal):
    if not all(-60.0 <= q[0] <= 240.0 for q in (start, goal)):
        raise MotionError("피크→페그 J1은 -60~240도 안에서만 이동해야 한다")


def _check_peg_transfer_path(start, goal, floor, margin):
    """관절 보간 모델의 TCP·J3~J6·손목 케이싱 높이를 검사한다.

    구간 중점과 최대 변위 상한으로 세분화한다. 드라이버의 실제 궤적/링크
    외형 전체에 대한 충돌 인증은 아니다. 불확실한 구간은 허용하지 않는다.
    """
    from . import kinematics as K
    if any(len(j) != 6 or not all(math.isfinite(v) for v in j) for j in (start, goal)):
        raise MotionError("피크→페그 경로의 관절각이 유효하지 않다")
    _check_peg_transfer_j1(start, goal)

    def height(j):
        f = K.fk(j)
        points = [f["tcp"], *f["origins"][2:], f["wrist_casing"]]
        z = min(p[2] for p in points)
        if not math.isfinite(z) or z < floor + margin:
            raise MotionError(f"피크→페그 경로 높이 부족: 모델 최저 Z={z:.1f}mm, "
                              f"필요 {floor + margin:.1f}mm (하한+모델 여유)")
        return z

    height(start)
    height(goal)
    pending = [(start, goal, 0)]
    while pending:
        a, b, depth = pending.pop()
        mid = [(x + y) / 2 for x, y in zip(a, b)]
        z = height(mid)
        # URDF 체인 길이+TCP+케이싱 반경보다 큰 1400mm를 회전 반경 상한으로 쓴다.
        bound = 1400.0 * sum(math.radians(abs(x - y)) for x, y in zip(a, b)) / 2
        if z - bound >= floor + margin:
            continue
        if depth >= 14:
            raise MotionError("피크→페그 경로 높이 여유를 확인하지 못해 중단")
        pending.extend(((a, mid, depth + 1), (mid, b, depth + 1)))


def _pick_to_peg_approach(pose, side, route, motion, logger=None, confirm=None, execute=True):
    """반환 자세로 페그 앞까지 접근한 뒤 TCP를 구속하지 않는 관절 이동으로 정렬한다."""
    api = _require_api()
    from DR_common2 import posx
    from . import kinematics as K
    floor, margin = _peg_transfer_floor(motion)
    start = _read_peg_joints(motion)
    current, _ = read_posx()
    current = [float(v) for v in current]
    if len(current) != 6 or not all(math.isfinite(v) for v in current):
        raise MotionError("피크→페그: 현재 TCP를 읽지 못했다")
    if current[2] < floor:
        raise MotionError(f"피크→페그: 상승 미완료 Z={current[2]:.1f} < {floor:.1f}mm")
    if pose.kind != "task" or getattr(pose, "sol", None) is None:
        raise MotionError("피크→페그: 작업 좌표와 교시된 sol이 필요하다")
    back = float(motion.get("approach_mm", 100.0))
    if not math.isfinite(back) or back <= 0:
        raise MotionError("피크→페그: 접근 거리는 유한한 양수여야 한다")
    approach = TP.backed_off(pose.task, back)
    high = list(approach[:3]) + current[3:]
    high[2] = max(current[2], floor + 2 * margin, approach[2])
    try:
        goal = [float(v) for v in api.ikin(posx(high), pose.sol)]
        if len(goal) != 6 or not all(math.isfinite(v) for v in goal):
            raise ValueError("유한한 관절각 6개가 아니다")
        error = math.dist(K.fk(goal)["tcp"], high[:3])
        if error > margin:
            raise ValueError(f"IK/FK 위치 대조 오차 {error:.1f}mm")
    except Exception as exc:
        raise MotionError(f"피크→페그 높은 접근점 IK 실패: {exc}") from exc
    lower = list(approach[:3]) + current[3:]
    orient_goal = None
    # 하강 끝점도 먼저 검사한다. 제한을 풀어 진행하지 않는다.
    j6, _ = _peg_joint_limits(motion)
    for label, task in (("하강", lower), ("파지 자세", approach)):
        try:
            joints = [float(v) for v in api.ikin(posx(task), pose.sol)]
            if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
                raise ValueError("유효한 관절각이 아니다")
            if motion.get("j6_recovery_enabled", False):
                _check_peg_turn(joints, 0.0, motion)
            if abs(joints[5]) > j6:
                raise ValueError(f"J6={joints[5]:+.2f}(허용 ±{j6:g})")
            _check_peg_transfer_j1(start, joints)
            if label == "파지 자세":
                orient_goal = joints
            if math.dist(K.fk(joints)["tcp"], task[:3]) > margin:
                raise ValueError("IK/FK 위치 불일치")
        except Exception as exc:
            raise MotionError(f"피크→페그 {label} 목표 검사 실패: {exc}") from exc
    plan = []
    if _peg_route_required(start[0], side, motion):
        if route is None or route.joints[0] != 90.0:
            raise MotionError("피크→페그: J1=+90도 경유지가 필요하다")
        plan.append(route.resolve(start))
    plan.append(goal)
    # 어떤 이동도 보내기 전에 경유를 포함한 전체 높은 이동 구간을 검사한다.
    previous = start
    j6, _ = _peg_joint_limits(motion)
    if logger:
        logger.info(f"경로 진단: 현재 TCP={current}, 현재 관절={start}")
        logger.info(f"경로 진단: 높은 목표 TCP={high}, sol={pose.sol}, IK 관절={goal}")
        logger.info(f"경로 진단: J6 허용 ±{j6:g}도")
    for target in plan:
        if motion.get("j6_recovery_enabled", False):
            _check_peg_turn(target, 0.0, motion)
        if abs(target[5]) > j6:
            raise MotionError(
                f"피크→페그 목표 관절 제한 초과: J6={target[5]:+.2f}도 (허용 ±{j6:g}); "
                f"목표 Z={high[2]:.2f}mm, sol={pose.sol}. 이동하지 않는다")
        _check_peg_transfer_j1(previous, target)
        previous = target
    if logger:
        logger.info(f"피크→페그: 페그 앞 접근 후 관절 자세 변경, "
                    f"상단 Z={high[2]:.1f}mm → 접근점에서만 Z={approach[2]:.1f}mm로 하강")
    if not execute:
        return {"current_tcp": current, "current_joints": start, "high_tcp": high,
                "approach_tcp": approach, "lower_tcp": lower, "joint_plan": plan,
                "orient_joints": orient_goal}
    for target in plan:
        if confirm:
            confirm(f"높은 경로 관절 이동: {[round(v, 2) for v in target]}")
        # 실제 관절값의 J1 금지 구간과 도달 여부는 계속 확인한다.
        _check_peg_transfer_j1(_read_peg_joints(motion), target)
        _movej(target, motion)
        actual = _read_peg_joints(motion)
        if max(abs(a - b) for a, b in zip(actual, target)) > 1.0:
            raise MotionError("피크→페그: 경유 관절 위치 미도달")
    if confirm:
        confirm(f"페그 앞 XY={approach[:2]}에서 Z={approach[2]:.1f}mm로 수직 하강")
    actual, _ = read_posx()
    if len(actual) != 6 or not all(math.isfinite(v) for v in actual):
        raise MotionError("피크→페그: 하강 전 TCP 읽기 실패")
    if actual[2] < floor or math.dist(actual[:3], high[:3]) > 2.0:
        raise MotionError("피크→페그: 높은 접근점 미도달, 하강 금지")
    # 반환 방향을 유지해 페그 앞까지 하강한 뒤에만 자세를 바꾼다.
    v, a = _transit_speed(motion)
    _movel_base(lower, v, a)
    actual, _ = read_posx()
    if (len(actual) != 6 or not all(math.isfinite(x) for x in actual)
            or math.dist(actual[:3], lower[:3]) > 2.0):
        raise MotionError("피크→페그 하강 미도달: 자세 변경 금지")

    if confirm:
        confirm("페그 앞에서 TCP 비고정 관절 이동으로 파지 자세 변경")
    _check_peg_transfer_j1(_read_peg_joints(motion), orient_goal)
    orient_motion = dict(motion, vel=motion.get("peg_orient_vel", 30.0),
                         acc=motion.get("peg_orient_acc", 60.0), speed_scale=1.0)
    _movej(orient_goal, orient_motion)
    actual = _read_peg_joints(motion)
    if max(abs(a - b) for a, b in zip(actual, orient_goal)) > 1.0:
        raise MotionError("피크→페그 파지 자세 미도달 — 진입 금지")


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
    if _cancel_check is not None:
        _cancel_check()
        from .motion_cancel import GuardedAPI
        original = _gripper.api
        _gripper.api = GuardedAPI(original, _cancel_check)
        try:
            result = _gripper.select(preset_name)
            _cancel_check()
            return result
        finally:
            _gripper.api = original
    return _gripper.select(preset_name)


def _release():
    """그리퍼를 연다."""
    return _grip("open")


def pluck_string(pose, motion, standby=None, *, prepare_only=False, prepared=False):
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

    prepare_only: 줄 위 접근점까지만 이동하며 접촉·발현하지 않는다.
    prepared: 준비 Action 성공 후 대기 자세 경유를 생략한다. 접근점은 다시 명령한다.

    순서 (성현 2026-09-19, 2026-09-21 대기 자세 좌표화):
      대기 자세 → 접촉점 위 pluck_up_mm → 접촉점(느리게) → 옆으로 스트로크
      → 즉시 후퇴(5mm 이상) → 접촉점 위 → 대기 자세
    """
    _require_api()
    ensure_tcp()                                        # 발현 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    pose, half_turn = _j6_pose(pose, motion)
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
    if half_turn:
        stroke[:2] = [-v for v in stroke[:2]]
        retract[:2] = [-v for v in retract[:2]]
    v_app, a_app = _transit_speed(motion, "pluck_vel_approach", "pluck_acc_approach")
    v_fine, a_fine = motion.get("pluck_vel_fine", 20), motion.get("pluck_acc_fine", 50)
    v_str, a_str = motion.get("pluck_vel_stroke", 1000), motion.get("pluck_acc_stroke", 2000)

    if _retract_depth_mm(retract) < 5.0:
        raise MotionError("후퇴 거리가 5mm 미만이다 (CLAUDE.md 3.4). pluck_retract_tool 을 확인하라")

    contact = list(x)
    above = list(x)
    above[2] += up_mm                                   # Base +Z

    if not prepared:
        _goto_standby(standby, motion, v_app, a_app)    # 기타 위 대기
    _movel_base(above, v_app, a_app)                    # 접촉점 위 2cm
    if prepare_only:
        return  # 마이크 ready 전에는 접촉점으로 내려가지 않는다
    _movel_base(contact, v_fine, a_fine)                # 접촉점 (느리게)

    # 스트로크·후퇴는 **피크가 줄에 닿아 있는 유일한 구간**이다. 이제 이동 거부를
    # 예외로 올리니까(#50), 여기서 그냥 빠져나가면 진동하는 줄에 피크를 얹은 채
    # 멈춘다 — 버징이 나고 그 줄 측정이 통째로 망가진다 (CLAUDE.md 3.4).
    # 그래서 실패해도 **일단 줄에서 떼고** 나서 예외를 다시 올린다.
    # MotionError 만 잡으면 안 된다 — 래퍼가 UnboundLocalError 를 던지는 판이
    # 있다 (_moved 의 주석). 줄에서 떼는 건 어느 예외에서든 해야 한다.
    try:
        _movel_tool_rel(stroke, v_str, a_str, radius=3)  # 옆으로 살짝 → 발현
        _movel_tool_rel(retract, v_str, a_str)           # 즉시 후퇴 (버징 방지)
    except Exception:
        try:
            _movel_base(above, v_app, a_app)             # 무슨 일이 있어도 줄에서 뗀다
        except Exception:
            pass                                         # 이것마저 안 되면 원래 예외가 먼저다
        raise

    _movel_base(above, v_app, a_app)                    # 다시 2cm 위
    _recover_j6(motion)
    _goto_standby(standby, motion, v_app, a_app)        # 기타 위 대기


def _standby_xyz(standby):
    """대기 자세의 TCP 위치 [X,Y,Z]. 못 구하면 None."""
    if standby.kind == "joint":
        try:
            from . import kinematics as K
            return [float(v) for v in K.fk(list(standby.joints))["tcp"]]
        except Exception:
            return None
    x = getattr(standby, "task", None)
    if not x or len(x) < 3:
        return None
    return [float(v) for v in x[:3]]


def _current_xyz():
    """지금 TCP 위치 [X,Y,Z]. 못 읽으면 None."""
    api = _require_api()
    try:
        now = api.get_current_posx()
    except Exception:
        return None
    if isinstance(now, (list, tuple)) and now and isinstance(now[0], (list, tuple)):
        now = now[0]                       # (posx, sol) 로 주는 판이 있다
    if not isinstance(now, (list, tuple)) or len(now) < 3:
        return None
    try:
        return [float(v) for v in now[:3]]
    except (TypeError, ValueError):
        return None


def _goto_standby(standby, motion, vel, acc):
    """
    Standby 가 있으면 그리로 간다. joints 면 movej, task(x) 면 Base 직선(movel).

    **이미 거기 있으면 안 간다** (#58). 발현 대기 자세는 네 줄의 접근점
    (접촉점 위 pluck_up_mm)에서 10~20mm 밖에 안 떨어져 있다 —

      pluck_standby TCP  [642.60, -12.03, 192.52]   (fk, 오차 +-3mm)
      6번 접근점         [645.85, -29.08, 202.28]   17.1mm
      5번 접근점         [645.23, -19.14, 202.18]   12.0mm
      2번 접근점         [645.24, -1.00, 201.78]    14.4mm
      1번 접근점         [645.34,   8.06, 201.32]   22.1mm

    그래서 발현이 끝날 때마다 "거의 제자리" 로 한 번 갔다가 다음 발현이 다시
    내려오는 왕복이 붙었다. 거리는 20mm 인데 관절 이동 한 번 값을 그대로 낸다.
    2026-09-21 통합 실기에서 사용자가 지적한 그 동작이다.

    현재 자세를 **로봇에서 읽어서** 판단한다. 플래그로 기억하지 않는 이유는
    _holding_pick 과 같다 — 중간에 예외가 나면 기억이 실제와 어긋난다.
    못 읽으면 건너뛰지 않고 그냥 간다 (안전한 쪽).
    """
    if standby is None:
        return
    skip_mm = float(motion.get("standby_skip_mm", 30.0))
    if skip_mm > 0:
        here, there = _current_xyz(), _standby_xyz(standby)
        if here and there:
            gap = sum((here[i] - there[i]) ** 2 for i in range(3)) ** 0.5
            if gap <= skip_mm:
                return                     # 이미 거기다. 가 봐야 제자리
    if standby.kind == "joint":
        _movej(standby.joints, motion)
    else:
        _movel_base(standby.task, vel, acc)


def _target_j1(task, sol):
    """그 작업 좌표를 그 팔 모양(sol)으로 풀었을 때의 J1. 못 풀면 None."""
    api = _require_api()
    try:
        from DR_common2 import posx
        j = api.ikin(posx(list(task)), sol)
    except Exception:
        return None
    if not j or len(j) < 6 or all(abs(float(v)) < 1e-9 for v in j):
        return None
    return float(j[0])


def _current_j1():
    """지금 J1. 못 읽으면 None."""
    api = _require_api()
    try:
        return float(list(api.get_current_posj())[0])
    except Exception:
        return None


def j1_flip(now_j1, want_j1, motion):
    """
    이 전환이 J1 을 뒤집는가. (뒤집는다, 돌아야 하는 각) 로 낸다.

    **판단은 여기 한 곳에서만 한다.** _ensure_solution 과 tools/sol_check.py 가
    같이 부른다 — 도구가 찍는 값이 곧 실기에서 코드가 내릴 판단이어야 한다.
    따로 계산하면 도구는 "경유 안 함" 이라 찍고 코드는 경유하는 일이 생긴다.

    문턱은 motion["j1_flip_deg"](기본 120도). 사용자 기준(2026-09-21) — "반대값"은
    150~180도쯤 차이 나는 경우이고 230도 차이도 옮겨야 한다. 그 아래는 근사값이라
    그 자리에서 이어 간다. 120 은 둘 사이의 여유값이고 실측 전 임시값이다.

    **모듈러로 접지 않는다.** movej 는 관절 공간에서 곧이곧대로 보간하므로 230도
    차이는 실제로 230도를 돈다. 130도로 줄여 보면 안 된다.

    둘 중 하나라도 None(못 읽음)이면 뒤집는 것으로 본다 — 모르면서 뒤집는 것보다
    한 번 더 들르는 쪽이 싸다. 그때 각은 None 이다.
    """
    if now_j1 is None or want_j1 is None:
        return True, None
    turn = abs(float(want_j1) - float(now_j1))
    return turn >= float(motion.get("j1_flip_deg", 120.0)), turn


def _ensure_solution(task, sol, route, motion, logger=None):
    """
    목표 팔 모양이 지금과 **J1 반대쪽**이면 J1=90 경유지를 먼저 지난다 (#58).

    2026-09-21 사용자 확인 — 거치대에서 팔 모양을 바꿀 때 J1 이 뒤집히면
    **어느 쪽으로 가든** J1 이 90 인 자세를 거쳐야 기타에 안 부딪힌다. 왼쪽으로
    갈 때도 오른쪽으로 돌아올 때도 같다. routes.transit 이 그 자세다
    ([90, 0, 0, 0, 0, null]) — 팔이 수직으로 서서 기타에서 떨어진 자리다.

    **판단은 sol 번호가 아니라 J1 각으로 한다.** sol 은 어깨·팔꿈치·손목 세 비트라
    번호가 달라도 팔꿈치나 손목만 다르면 J1 은 그대로다. 그때는 그 자리에서
    이어서 작업하면 되므로 경유가 낭비다. 기타를 가로지르는 건 J1 이 뒤집힐 때뿐이다.

    문턱은 motion["j1_flip_deg"](기본 120도). 사용자가 말한 "반대값" 이 **150~180도쯤
    차이 나는 경우**이고 230도 차이도 옮겨야 할 수 있다 (2026-09-21). 그 아래는
    근사값이라 그 자리에서 이어 간다. 120 은 그 둘 사이에 여유를 둔 값이다.

    **모듈러로 접지 않는다.** movej 는 관절 공간에서 곧이곧대로 보간하므로 230도
    차이는 실제로 230도를 돈다. 130도로 줄여 보면 안 된다.

    둘 중 하나라도 못 읽으면 지난다 — 모르면서 뒤집는 것보다 싸다.

    바꿨으면 True. 부르는 쪽은 그때 via 를 건너뛴다 — 경유지가 더 안전한 출발점이다.
    """
    if sol is None:
        return False
    now, want = _current_j1(), _target_j1(task, sol)
    flip, turn = j1_flip(now, want, motion)
    if not flip:
        if logger:
            logger.info(f"J1 {now:.0f} -> {want:.0f}도 ({turn:.0f}도). "
                        "같은 쪽이라 경유지를 안 탄다")
        return False
    why = (f"J1 {now:.0f} -> {want:.0f}도 ({turn:.0f}도 뒤집힘)" if turn is not None
           else f"J1 을 못 읽었다 (지금 {now}, 목표 {want})")
    if route is None:
        raise MotionError(
            f"거치대 팔 모양을 바꿔야 하는데 경유지가 없다 — {why}. "
            "J1=90 을 안 거치고 뒤집으면 기타에 부딪힌다 (routes.transit 확인)")
    if logger:
        logger.info(f"{why}. J1=90 경유지를 먼저 지난다")
    goto_route(route, motion)
    return True


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


def pick_up(holder, motion, standby=None, via=None, sol=None, route=None,
            logger=None):
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

    sol — 거치대 위 대기점에 **어느 팔 모양으로** 설 지 (IK 해 0~7). 페그의 sol 과
          같은 뜻이다 (#58). 주면 그 대기점까지 movejx(관절 보간)로 간다 —
          movel 은 지금 팔 모양을 유지하므로 갈래를 못 바꾼다.
          지금 거치대 접근은 J1 이 오른쪽인 해 하나뿐이라, 피크를 집고 발현한 뒤
          6·5번(왼쪽) 페그로 가려면 경유지를 타야 했다. 왼쪽 해로 집어 두면 그
          경유가 없어진다. None 이면 지금 팔 모양을 그대로 쓴다 (예전 동작).

          내려가기·들어 올리기는 sol 과 무관하게 **직선**이다. 거치대·스토퍼를
          수직으로 빠져나와야 하고, 직선은 팔 모양을 안 바꾼다.

    route — 팔 모양을 바꿀 때 지나는 경유지(routes.transit, J1=90). sol 을 주면
            같이 줘야 한다. 자세한 이유는 _ensure_solution.

    RG2.select() 의 settle_sec(기본 0.3초)만으로는 부족해 집자마자 들어 올리는 동작이
    물리적으로 안 끝난 그리퍼를 밀고 올라갔다 (2026-09-20 실기). 그래서 집은 뒤
    motion["pick_settle_sec"](기본 1.0초)만큼 한 번 더 멈췄다 움직인다.
    """
    _require_api()
    ensure_tcp()                                         # 거치대 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    holder, _ = _j6_pose(holder, motion, sol=sol)
    contact, above = _pick_points(holder, motion)
    preset = getattr(holder, "grip_preset", None) or motion.get("pick_preset", "close")
    v_app, a_app = _transit_speed(motion)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)
    settle_sec = float(motion.get("pick_settle_sec", 1.0))

    flipped = _ensure_solution(above, sol, route, motion, logger)  # J1 뒤집히면 경유
    if not flipped and via is not None:
        _movej(via.joints, motion)                      # 경유 자세 (관절 보간)
    _release()                                          # 열고 간다
    if sol is None:
        _movel_base(above, v_app, a_app)                # 거치대 위 (직선, 팔 모양 유지)
    else:
        _movejx(above, motion, sol=sol)                 # 거치대 위 (팔 모양을 정해서)
    _movel_base(contact, v_fine, a_fine)                # 내려가서
    _grip(preset)                                       # 집는다
    time.sleep(settle_sec)                              # 그리퍼가 물리적으로 다 닫히기를 기다린다
    _movel_base(above, v_app, a_app)                    # 들어 올린다
    _recover_j6(motion)
    if standby is not None:
        _movej(standby.joints, motion)                  # 기타 위 대기


def _check_pick_return_position(target, motion, stage, logger=None):
    """반환 위치를 못 확인하면 그리퍼를 열지 않는다. mm 단위 임시 허용값."""
    tolerance = float(motion.get("pick_return_tolerance_mm", 2.0))
    if not math.isfinite(tolerance) or tolerance <= 0:
        raise MotionError("pick_return_tolerance_mm 는 유한한 양수여야 한다")
    actual = _current_xyz()
    if actual is None or not all(math.isfinite(v) for v in actual):
        raise MotionError(f"피크 반환 {stage}: 현재 위치를 읽지 못해 중단")
    error = sum((a - b) ** 2 for a, b in zip(actual, target[:3])) ** 0.5
    if logger:
        logger.info(f"피크 반환 {stage}: 목표 {target[:3]}, 실제 {actual}, 오차 {error:.2f}mm")
    if error > tolerance:
        raise MotionError(f"피크 반환 {stage}: 위치 오차 {error:.2f}mm > {tolerance:.2f}mm")


def pick_return(holder, motion, via=None, home=None, sol=None, route=None,
                logger=None):
    """
    피크를 거치대에 돌려놓는다.

    via — 거치대로 내려가기 전에 먼저 들르는 안전 경유 관절 자세(Pose, joints).
          발현 직후엔 팔이 기타 위 대기 자세(pluck_standby)에 있는데, 거기서
          거치대 위로 바로 movel 하는 경로는 검증된 적이 없다. frames.standby
          를 경유해 안전한 관절 자세를 한 번 거친 뒤 내려간다.

    sol — 거치대 위 대기점에 설 팔 모양(IK 해 0~7). pick_up 의 sol 과 같다 (#58).
          **반환 쪽이 더 중요하다** — 반환이 끝나면 바로 페그를 잡으러 가기
          때문이다. 다음에 갈 페그 쪽의 해로 서 있으면 좌우 전환 경유지를
          건너뛸 수 있다. None 이면 지금 팔 모양을 그대로 쓴다.

    순서: (via 가 있으면) 경유 자세 → 거치대 위 pick_up_mm → 거치대(느리게)
          → 그리퍼 열기 → 수직 상승/도착 확인 → (home 이 있으면) 홈 자세.
    pick_to_peg_min_z_mm 설정 시 빈손 상승은 하한+모델 여유 2배 이상으로 한다.
    """
    _require_api()
    ensure_tcp()                                         # 거치대 좌표도 robot.tcp 기준이다 (이슈 #21·#32)
    holder, _ = _j6_pose(holder, motion, sol=sol)
    contact, above = _pick_points(holder, motion)
    v_app, a_app = _transit_speed(motion)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)

    flipped = _ensure_solution(above, sol, route, motion, logger)  # J1 뒤집히면 경유
    if not flipped and via is not None:
        _movej(via.joints, motion)                      # 경유 자세
    if sol is None:
        _movel_base(above, v_app, a_app)                # 거치대 위 (직선, 팔 모양 유지)
    else:
        _movejx(above, motion, sol=sol)                 # 거치대 위 (팔 모양을 정해서)
    _check_pick_return_position(above, motion, "거치대 위", logger)
    _movel_base(contact, v_fine, a_fine)                # 수직으로 내려가서
    _check_pick_return_position(contact, motion, "반환점", logger)
    _release()                                          # 위치 확인 후 놓는다
    retreat = list(above)
    if "pick_to_peg_min_z_mm" in motion:
        floor, margin = _peg_transfer_floor(motion)
        retreat[2] = max(retreat[2], floor + 2 * margin)
    _movel_base(retreat, v_app, a_app)                  # 빈손 수직 상승 후 높이 확보
    _check_pick_return_position(retreat, motion, "반환 후 상승", logger)
    _recover_j6(motion)
    if home is not None:
        _movej(home.joints, motion)                     # 홈


def _retract_depth_mm(delta):
    """후퇴 벡터의 병진 크기(mm)."""
    return sum(float(v) ** 2 for v in delta[:3]) ** 0.5


def _movel_base(pos, vel, acc):
    """
    직선 이동 (Base 절대 posx). 속도를 직접 준다. _movel 은 motion 절의 travel_* 를 쓴다.

    반환을 _moved 로 본다 (#50). 거치대로 내려가는 명령이 거부돼도 다음 줄에서
    그리퍼는 닫힌다 — 빈손을 물고 "집었다" 가 된다. 피크는 놓치면 비전이 없어
    되찾을 수 없다.
    """
    api = _require_api()
    from DR_common2 import posx
    _guard_j6_task(pos)
    return _moved(api.movel(posx(list(pos)), vel=vel, acc=acc, ref=api.DR_BASE, radius=0),
                  "movel_base")


def _movel_tool_rel(delta, vel, acc, radius=0):
    """
    Tool(TCP) 좌표 기준 상대 직선 이동. 스트로크·후퇴에 쓴다.

    피크 자세를 기준으로 움직이므로 기타가 조금 옮겨져도 방향이 유지된다.
    radius 를 주면 다음 모션과 블렌딩(멈춤 없이 이어짐).

    반환을 _moved 로 본다 (#50). 발현 직후 5mm 후퇴가 이 경로다 — 거부된 걸
    모르고 넘어가면 진동하는 줄에 피크가 닿은 채로 다음 동작이 이어진다.
    """
    api = _require_api()
    from DR_common2 import posx
    return _moved(api.movel(posx(list(delta)), vel=vel, acc=acc,
                            ref=api.DR_TOOL, mod=api.DR_MV_MOD_REL, radius=radius),
                  "movel_tool_rel")


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
    turned = copy.copy(pose)
    turned.task = roll_about_tool_z(pose.task, roll_deg)
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


def _cleanup_register(pose, motion, logger, keep_error):
    """
    register_peg 의 뒷정리 — 놓고, 순응을 끄고, 물러난다 (#52).

    **원래 실패 사유를 덮지 않는다.** finally 안에서 새 예외가 나면 파이썬이 원래
    예외를 그것으로 대체한다. 하필 제일 알고 싶은 때 겹친다 — 순응이 안 풀리는
    상황과 진입이 실패하는 상황은 원인이 같아서(에러 래치·통신 지연·모드 전환
    실패) 같이 일어난다. 그래서 진짜 진단인 "빈손이다 (폭 15.1mm)" 가 뒷정리
    실패인 "movel 가 거부됐다 (-1)" 로 바뀌어 사람을 엉뚱한 곳으로 보낸다.

    keep_error=True   본문이 이미 실패했다. 정리 실패는 경고로만 남긴다
    keep_error=False  본문이 성공했다. 정리 실패를 그대로 올린다 — 성공이라고 해
                      놓고 팔이 페그에 붙어 있으면 다음 동작이 거기서 출발한다
    """
    def _step(what, fn):
        try:
            return fn()
        except Exception as exc:
            if not keep_error:
                raise
            if logger:
                logger.warn(f"정리: {what} 실패 — {exc}. 원래 실패 사유를 살린다 (#52)")
            return None

    _step("그리퍼 열기", lambda: _grip("open"))

    # 순응 끄기는 **반환을 본다.** 안 풀리면 위치 명령이 -1 을 돌려주므로(#31)
    # 이어서 후퇴를 부르면 그 -1 이 원래 사유를 덮는다. 아예 안 부른다.
    if _step("순응 끄기", lambda: compliance(motion, on=False)) == -1:
        msg = ("순응제어가 안 풀렸다 (release_compliance_ctrl -1). 후퇴를 건너뛴다 — "
               "팔이 페그 앞에 붙은 채 남고 순응이 켜진 상태다. 펜던트에서 확인할 것 (#52)")
        if not keep_error:
            raise MotionError(msg)
        if logger:
            logger.warn(msg)
        return
    _step("후퇴", lambda: _retreat(pose, motion))


def register_peg(pose, motion, handle, read_width, logger=None,
                 use_force=True, assume_grip=False, from_pick=False, side=None, route=None):
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

    pose, _ = _j6_pose(pose, motion)
    _grip("open")
    approach_mm = float(motion.get("approach_mm", 100.0))
    back = TP.backed_off(pose.task, approach_mm)
    if from_pick:
        _pick_to_peg_approach(pose, side, route, motion, logger)
    else:
        _movejx(back, motion, sol=getattr(pose, "sol", None))

    # 먼 구간은 **순응을 켜기 전에** 위치 제어로 간다. 순서가 중요하다 —
    # 순응이 켜진 뒤에 위치 명령을 주면 스프링이 흡수해 팔이 그 자리에 선다.
    # 2026-09-21 실기에서 search_mm 8 을 줬는데 힘 제어가 99.8mm 를 가려 했고,
    # 4N 으로 20초에 7.5mm 밖에 못 갔다. 먼 구간은 딱딱할 때 끝내야 한다.
    search_mm = float(handle.get("search_mm", 25.0))
    # 힘 제어를 쓰면 search_mm 앞까지만, 안 쓰면 **페그까지** 여기서 간다.
    # --no-force 도 순응을 켠 뒤에 100mm 를 가려다 그 자리에 서 있었다
    # (2026-09-21 실기). 순응 아래서는 위치 명령이 안 먹는다
    stop_mm = search_mm if use_force else 0.0
    if stop_mm < approach_mm:
        where = f"앞 {stop_mm:.0f}mm 까지" if stop_mm else "까지"
        if logger:
            logger.info(f"페그 {where} 위치 제어로 간다 (순응 켜기 전)")
        _movel(TP.backed_off(pose.task, stop_mm), motion)

    # 순응은 **항상** 켠다. 힘 제어와 다른 것이다 —
    #   힘 제어  "이 힘까지만 민다"      (미검증, use_force 로 끌 수 있다)
    #   순응제어  "걸리면 물러난다"       (끄면 로봇이 뻣뻣해져 오히려 더 세게 민다)
    # use_force 의 목적은 미검증 힘 제어를 안 태우는 것이지 안전망을 걷는 게 아니다.
    # 순응만으로도 미는 힘이 강성 × 남은 거리로 묶인다 (PR #37 리뷰, 정동휘)
    compliance(motion, on=True)
    failed = True                # 뒷정리가 원래 사유를 덮지 않게 갈라 준다 (#52)
    try:
        if use_force:
            _insert_force_controlled(pose, motion, handle, logger=logger)
        else:
            # 사람이 손잡이를 눕혀 놓고 보고 있는 세션. 그냥 들어간다
            if logger:
                logger.warn("힘 제어를 건너뛴다 — 손잡이가 눕혀져 있는지 사람이 볼 것")
                logger.warn("진입은 이미 끝났다(순응 켜기 전). 순응은 파지·후퇴만 지킨다")
        _grip(probe)                             # 꽉 문다. 여기서 각이 확정된다
        if assume_grip:
            if logger:
                logger.warn("폭을 안 읽는다 — 물렸는지 눈으로 확인할 것 (빈손·사점 못 가림)")
            failed = False
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
        failed = False
        return 0.0
    finally:
        _cleanup_register(pose, motion, logger, keep_error=failed)
        if not failed:
            _recover_j6(motion)


def _peg_joint_limits(motion):
    """J6 현장 제한과 임시 여유각. J5 소프트웨어 제한은 사용자 요청으로 제거."""
    try:
        j6 = float(motion.get("peg_j6_limit_deg", 360.0))
        margin = float(motion.get("peg_joint_margin_deg", 5.0))
        if not all(math.isfinite(v) for v in (j6, margin)):
            raise ValueError("유한한 숫자가 아니다")
        if not (0 < j6 <= 360 and 0 < margin < j6):
            raise ValueError("제한은 0~360도, 여유는 제한보다 작은 양수여야 한다")
    except (TypeError, ValueError) as exc:
        raise MotionError(f"페그 관절 제한 설정 오류: {exc}") from exc
    return j6, margin


def _read_peg_joints(motion, *, for_recovery=False):
    """일반 이동은 J6 한계를 검사한다. 회복 시작값만 한계 초과를 허용한다."""
    j6, _ = _peg_joint_limits(motion)
    try:
        joints = [float(v) for v in _require_api().get_current_posj()]
        if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
            raise ValueError("유한한 관절각 6개가 아니다")
    except Exception as exc:
        raise MotionError(f"페그 관절각 읽기 실패 — 이동 중단: {exc}") from exc
    if not for_recovery and abs(joints[5]) > j6:
        raise MotionError(f"J6 {joints[5]:+.1f}도가 한계 ±{j6:g}도 밖이다. "
                          "일반 이동 금지 — 접촉 이탈 후 J6 회복이 필요하다")
    return joints


def _check_peg_turn(joints, delta_deg, motion):
    """현재 J6와 요청 후 J6를 함께 검사한다. 페그 근처에서는 되감지 않는다."""
    limit, margin = _peg_joint_limits(motion)
    end = joints[5] + delta_deg
    if motion.get("j6_recovery_enabled", False):
        if max(abs(joints[5]), abs(end)) >= _j6_threshold(motion):
            raise MotionError("J6 회복 기준에 도달한다 — 접촉 전에 회복해야 한다")
    if max(abs(joints[5]), abs(end)) > limit - margin:
        raise MotionError(f"J6 회전 여유 부족: {joints[5]:+.1f} → {end:+.1f}도, "
                          f"허용 ±{limit - margin:.1f}도. 접근점에서 되감기가 필요하다")


def _j6_threshold(motion):
    value = float(motion.get("j6_recover_at_deg", 280.0))
    if not math.isfinite(value) or not 180 < value < 300:
        raise MotionError("J6 회복 기준은 180도 초과 300도 미만이어야 한다")
    return value


def _recover_j6(motion, delta_deg=0.0):
    """호출자는 접촉에서 벗어났음을 보장한다. 콜백에서는 절대 호출하지 않는다."""
    if not motion.get("j6_recovery_enabled", False):
        return 0.0
    joints = _read_peg_joints(motion, for_recovery=True)
    threshold = _j6_threshold(motion)
    if max(abs(joints[5]), abs(joints[5] + delta_deg)) < threshold:
        return 0.0
    candidates = [offset for offset in (-180.0, 180.0)
                  if max(abs(joints[5] + offset), abs(joints[5] + offset + delta_deg)) < threshold]
    if not candidates:
        raise MotionError("180도 회복으로 J6 여유를 확보할 수 없다")
    offset = min(candidates, key=lambda x: abs(joints[5] + x))
    target = list(joints)
    target[5] += offset
    recovery = dict(motion, vel=float(motion.get("j6_recovery_vel", 30.0)),
                    acc=float(motion.get("j6_recovery_acc", 60.0)), speed_scale=1.0)
    if not all(math.isfinite(recovery[key]) and recovery[key] > 0 for key in ("vel", "acc")):
        raise MotionError("J6 회복 속도·가속도는 유한한 양수여야 한다")
    recovery.pop("transit_tcp_vel", None)
    recovery.pop("transit_tcp_acc", None)
    tolerance = float(motion.get("peg_unwind_tolerance_deg", 1.0))
    if not math.isfinite(tolerance) or not 0 < tolerance <= 5:
        raise MotionError("J6 회복 도달 허용치 오류")
    _movej(target, recovery)
    actual = _read_peg_joints(motion)
    if max(abs(a - b) for a, b in zip(actual, target)) > tolerance:
        raise MotionError("J6 180도 회복 미도달")
    if _node is not None:
        _node.get_logger().info(f"J6 180도 회복: {joints[5]:.1f} → {actual[5]:.1f}도")
    return offset


def _j6_pose(pose, motion, delta_deg=0.0, *, sol=None):
    """180도 대칭인 그리퍼/피크의 목표 중 현재 J6에서 가까운 유효 자세를 고른다."""
    if not motion.get("j6_recovery_enabled", False):
        return pose, 0.0
    from DR_common2 import posx
    api = _require_api()
    current = _read_peg_joints(motion)
    threshold = _j6_threshold(motion)
    if sol is None:
        sol = getattr(pose, "sol", None)
    if sol is None:
        sol = api.get_current_solution_space()
    candidates = []
    for offset in (0.0, -180.0, 180.0):
        candidate = _rolled(pose, offset)
        try:
            joints = list(api.ikin(posx(candidate.task), sol))
            if len(joints) != 6 or not all(math.isfinite(v) for v in joints):
                continue
            if max(abs(joints[5]), abs(joints[5] + delta_deg)) >= threshold:
                continue
            candidates.append((abs(joints[5] - current[5]), candidate, offset))
        except Exception:
            continue
    if not candidates:
        raise MotionError("J6 회전 여유가 있는 대칭 목표 자세가 없다")
    _, candidate, offset = min(candidates, key=lambda item: item[0])
    return candidate, offset


def _prepare_peg_turn(delta_deg, motion):
    """페그에서 떨어진 접근점 전용. J6만 360도 되감아 같은 끝 자세를 유지한다."""
    if motion.get("j6_recovery_enabled", False):
        return _recover_j6(motion, delta_deg)
    try:
        tolerance = float(motion.get("peg_unwind_tolerance_deg", 1.0))
        if not math.isfinite(tolerance) or not 0 < tolerance <= 5.0:
            raise ValueError("0 초과 5도 이하만 허용한다")
    except (TypeError, ValueError) as exc:
        raise MotionError(f"되감기 도달 허용치 오류: {exc}") from exc
    joints = _read_peg_joints(motion)
    try:
        _check_peg_turn(joints, delta_deg, motion)
        return
    except MotionError:
        if motion.get("peg_unwind_enabled", False) is not True:
            raise MotionError("J6 되감기가 필요하다. 접근점 회전 공간을 실기 확인한 뒤 "
                              "motion.peg_unwind_enabled를 true로 설정하라")
    candidates = []
    for offset in (-360.0, 360.0):
        target = list(joints)
        target[5] += offset
        try:
            _check_peg_turn(target, delta_deg, motion)
        except MotionError:
            continue
        candidates.append(target)
    if not candidates:
        raise MotionError("J6를 360도 되감아도 요청 회전 여유가 없다. 요청각·현장 제한을 확인하라")
    target = min(candidates, key=lambda q: max(abs(q[5]), abs(q[5] + delta_deg)))
    # 페그 회전과 같은 느린 속도. J1~J5는 고정하며 C 좌표만 바꾸지 않는다.
    if _node is not None:
        _node.get_logger().info(
            f"페그 접근점에서 J6 되감기: {joints[5]:+.1f} → {target[5]:+.1f}도")
    _movej(target, turn_motion(motion))
    actual = _read_peg_joints(motion)
    if any(abs(a - b) > tolerance for a, b in zip(actual, target)):
        raise MotionError("J6 되감기 목표 관절각에 도달하지 못했다 — 진입 중단")
    _check_peg_turn(actual, delta_deg, motion)


def adjust_peg(pose, joint, delta_deg, motion, roll_deg=0.0,
               from_pick=False, side=None, route=None, logger=None):
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
    호출자가 peg_state.turning 으로 먼저 무효화하고, 후퇴까지 성공하면 각을 확정한다.

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

    if not math.isfinite(delta_deg) or not math.isfinite(roll_deg):
        raise MotionError("페그 회전각은 유한한 숫자여야 한다")
    if joint == 6:
        if pose.kind != "task":
            raise MotionError("J6 페그 조절은 접근점이 있는 작업 좌표가 필요하다")
        back = float(motion.get("approach_mm", 100.0))
        if not math.isfinite(back) or back <= 0:
            raise MotionError("페그 접근 거리는 양수여야 한다")
        _read_peg_joints(motion)                    # 이미 한계 밖이면 첫 이동도 금지

    entry, half_turn = _j6_pose(_rolled(pose, roll_deg), motion, delta_deg)
    # 돌리고 나면 손목이 그만큼 더 돌아가 있다. 나올 때 원래 자세로 되돌리면
    # 손목이 역회전하면서 아직 손가락 사이에 있는 페그를 다시 돌려 버린다.
    # J6 이 아니면 C 가 안 바뀌므로 들어온 자세 그대로 나간다
    c_delta = delta_deg if joint == 6 else 0.0
    exit_ = _rolled(pose, roll_deg + half_turn + c_delta)

    # 들어가기 전에 반드시 연다. 직전 동작이 피크를 물고 있었으면 15mm 인 채로
    # 들어가 손잡이를 정면으로 들이받는다. 나올 때는 _release() 가 같은 폭으로 연다.
    # 폭은 gripper.presets.open 이 정한다(지금 38mm). 더 벌리면 옆 페그 손잡이를
    # 치고, 줄이면 눕힌 손잡이도 못 지난다. 창이 5mm 도 안 된다 (#31)
    _grip("open")
    if from_pick:
        _pick_to_peg_approach(entry, side, route, motion, logger)
    if joint == 6:
        if not from_pick:
            _movejx(TP.backed_off(entry.task, back), motion, sol=getattr(entry, "sol", None))
        recovered = _prepare_peg_turn(delta_deg, motion) or 0.0
        entry = _rolled(entry, recovered)
        exit_ = _rolled(exit_, recovered)
        _movel(entry.task, motion)
        _check_peg_turn(_read_peg_joints(motion), delta_deg, motion)
    elif from_pick:
        _movel(entry.task, motion)
    else:
        _goto_pose(entry, motion)
    _grip("peg")                                    # 여유를 준 폭으로 파지
    try:
        if joint == 6:
            _check_peg_turn(_read_peg_joints(motion), delta_deg, motion)
        delta = [0.0] * 6
        delta[joint - 1] = float(delta_deg)
        # 회전은 이동보다 느리게 간다. 줄에 장력을 거는 구간이다
        _movej(delta, turn_motion(motion, pose), relative=True)
        if joint == 6:
            _check_peg_turn(_read_peg_joints(motion), 0.0, motion)
    except Exception as exc:
        # 부분 회전 후 entry 자세로 되돌리면 페그를 역회전시킬 수 있다.
        # 현재 자세의 방향을 유지해 물러난다. 읽기 실패 시 추정 후퇴는 하지 않는다.
        try:
            _release()
            if joint == 6:
                _read_peg_joints(motion)           # 한계 밖이면 자동 후퇴도 하지 않는다
            task, _ = read_posx()
            task = [float(v) for v in task]
            if len(task) != 6 or not all(math.isfinite(v) for v in task):
                raise MotionError("현재 TCP 자세가 유효하지 않다")
            _retreat(TP.Pose(pose.name, True, task=task, tcp=pose.tcp), motion)
        except Exception as cleanup:
            raise MotionError(f"{exc}; 페그 해제·후퇴도 실패: {cleanup}") from exc
        raise
    else:
        _release()                                  # 열기가 실패하면 강제 후퇴하지 않는다
        _retreat(exit_, motion)                     # 돌아간 자세로 나와 페그를 되돌리지 않는다
        _recover_j6(motion)                          # 페그와 분리된 뒤에만


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
