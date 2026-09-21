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
  - pluck_string: 피크를 집어 힘 제어로 타현 + 발현 직후 5mm 후퇴 + 피크 반환
  - strum_all: 전체 줄 훑기
  - return_home: 대기 자세 복귀 (strum_all 끝에 호출됨. 이동만이라 구현 완료)
  - 파지 확인 — RG2 의 grip detected 를 이 ROS 2 패키지가 토픽으로 안 내보낸다
    (gripper.py 주석 참조). 확인 수단을 정해야 한다
  - pluck_tcp — 발현·거치대 좌표를 읽은 TCP 이름을 YAML 에 적고,
    pluck_string / pick_up / pick_return 시작 시 ensure_tcp 로 켠다 (이슈 #21, 성현)
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
    standby — 기타 위 대기 자세(Pose, joints). 주면 발현 전후에 들른다.

    순서 (성현 2026-09-19):
      대기 자세 → 접촉점 위 pluck_up_mm → 접촉점(느리게) → 옆으로 스트로크
      → 즉시 후퇴(5mm 이상) → 접촉점 위 → 대기 자세
    """
    _require_api()
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

    if standby is not None:
        _movej(standby.joints, motion)                  # 기타 위 대기
    _movel_base(above, v_app, a_app)                    # 접촉점 위 2cm
    _movel_base(contact, v_fine, a_fine)                # 접촉점 (느리게)
    _movel_tool_rel(stroke, v_str, a_str, radius=3)     # 옆으로 살짝 → 발현
    _movel_tool_rel(retract, v_str, a_str)              # 즉시 후퇴 (버징 방지)
    _movel_base(above, v_app, a_app)                    # 다시 2cm 위
    if standby is not None:
        _movej(standby.joints, motion)                  # 기타 위 대기


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


def pick_up(holder, motion, standby=None):
    """
    거치대에서 피크를 집는다 (성현 2026-09-20).

    순서: 그리퍼 열기 → 거치대 위 pick_up_mm → 거치대(느리게) → 그리퍼 닫기(프리셋)
          → 거치대 위 → (standby 가 있으면) 기타 위 대기 자세

    holder  — 거치대 자세 (teach_points.Pose, x). holder.grip_preset 이 있으면 그 프리셋으로,
              없으면 motion["pick_preset"], 그것도 없으면 "close" 로 잡는다.
              폭·힘은 WebLogic 에 박힌 값이라 코드가 못 정한다 (gripper.py).
    파지 확인은 하지 않는다 (CLAUDE.md 3.6, 잡았다고 가정).

    RG2.select() 의 settle_sec(기본 0.3초)만으로는 부족해 집자마자 들어 올리는 동작이
    물리적으로 안 끝난 그리퍼를 밀고 올라갔다 (2026-09-20 실기). 그래서 집은 뒤
    motion["pick_settle_sec"](기본 1.0초)만큼 한 번 더 멈췄다 움직인다.
    """
    _require_api()
    contact, above = _pick_points(holder, motion)
    preset = getattr(holder, "grip_preset", None) or motion.get("pick_preset", "close")
    v_app, a_app = motion.get("pick_vel_approach", 100), motion.get("pick_acc_approach", 100)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)
    settle_sec = float(motion.get("pick_settle_sec", 1.0))

    _release()                                          # 열고 간다
    _movel_base(above, v_app, a_app)                    # 거치대 위 5cm
    _movel_base(contact, v_fine, a_fine)                # 내려가서
    _grip(preset)                                       # 집는다
    time.sleep(settle_sec)                              # 그리퍼가 물리적으로 다 닫히기를 기다린다
    _movel_base(above, v_app, a_app)                    # 들어 올린다
    if standby is not None:
        _movej(standby.joints, motion)                  # 기타 위 대기


def pick_return(holder, motion, home=None):
    """
    피크를 거치대에 돌려놓는다.

    순서: 거치대 위 pick_up_mm → 거치대(느리게) → 그리퍼 열기 → 거치대 위
          → (home 이 있으면) 홈 자세
    """
    _require_api()
    contact, above = _pick_points(holder, motion)
    v_app, a_app = motion.get("pick_vel_approach", 100), motion.get("pick_acc_approach", 100)
    v_fine, a_fine = motion.get("pick_vel_fine", 30), motion.get("pick_acc_fine", 50)

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


def read_force():
    """툴에 걸리는 외력 [Fx,Fy,Fz,Mx,My,Mz] (Tool 기준). 로그·힘 제어 파라미터 산정용."""
    api = _require_api()
    return api.get_tool_force(api.DR_TOOL)


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

    entry = _rolled(pose, roll_deg)                 # 손잡이가 돌아가 있으면 맞춰 들어간다 (#31)
    # 돌리고 나면 손목이 그만큼 더 돌아가 있다. 나올 때 원래 자세로 되돌리면
    # 손목이 역회전하면서 아직 손가락 사이에 있는 페그를 다시 돌려 버린다.
    # peg_joint 는 툴 Z 둘레 회전(손목 롤)이라 관절 회전량이 곧 C 변화량이다
    exit_ = _rolled(pose, roll_deg + delta_deg) if joint == 6 else entry

    # 들어가기 전에 반드시 연다. 직전 동작이 피크를 물고 있었으면 15mm 인 채로
    # 들어가 손잡이를 정면으로 들이받는다. 나올 때는 _release() 가 같은 폭으로 연다.
    # **완전 개방이 아니라 30mm 다** — 더 벌리면 일정 각에서 옆 페그(간격 38mm)를
    # 친다. 30mm 는 손잡이에 안 닿는 폭이라 진입·후퇴 중 간섭이 없다 (#31)
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
