"""
pluck_dummy_test — robot_controller 없이 motion_primitives.pluck_string 을 직접 부른다.

목적: 성현 담당 pluck_string 1차 구현(위치 제어만)을 로봇에 붙여 확인한다.
      Action·YAML·컨트롤러를 거치지 않고, 여기서 Pose 객체와 motion dict 를
      손으로 만들어 넘긴다 (더미 데이터).

전제: 홈 자세에서 RG2 가 피크를 이미 잡고 있다. DO_PICK=True 면 집기·반환도 한다.

흐름 (성현 2026-09-20):
  홈 → (DO_PICK 이면 피크 집기) → [줄마다] 접촉점 위 2cm → 접촉점 → 옆으로 살짝(발현)
  → 즉시 후퇴 → 접촉점 위 2cm → 기타 위 대기 → (다음 줄) ... → (DO_PICK 이면 피크 반환) → 홈

실행:
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇IP> gripper:=rg2
  source install/setup.bash        # tunio_robot 을 import 하려면 (빌드 전이면 src 경로로 대체됨)
  python3 tools/pluck_dummy_test.py

==== [교시값] 블록만 채우면 된다 ==========================================
  HOME_J, STANDBY_J : 관절각 6개 (get_current_posj 로 읽은 값)
  CONTACT_X[n]      : n번 줄 접촉점. Base 절대 posx [X,Y,Z,A,B,C] (get_current_posx)
                      ROBOT_CFG["tcp"] 를 켠 상태에서 읽은 값이어야 한다
  STRINGS_TO_PLUCK  : 이번에 칠 줄 순서
===========================================================================
"""
import os
import sys
import time

# 빌드 전에도 돌게 src 경로를 추가한다 (install 이 source 돼 있으면 그쪽이 먼저)
_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "tunio_robot")
if os.path.isdir(_SRC):
    sys.path.append(os.path.abspath(_SRC))

import rclpy  # noqa: E402
from tunio_robot import motion_primitives as MP  # noqa: E402
from tunio_robot.teach_points import Pose  # noqa: E402
import yaml  # noqa: E402

# ─────────────────────────── [로봇 설정] ─────────────────────────────
ROBOT_CFG = {
    "id": "dsr01",
    "model": "m0609",
    "tool": None,       # "Tool Weight" 로 켜려 했으나 set_tool/set_tcp/add_tcp 전부 -1 로 거부됨
    "tcp": None,        # (2026-09-20). 펜던트 제어권 문제로 보임, 팀 확인 전까지 None(플랜지)으로 되돌린다.
}
# None 이면 TP 에 지금 켜져 있는 것(플랜지, 확인됨)을 그대로 쓴다. 아래 CONTACT_X 도 플랜지 기준으로
# 되돌렸다. GripperDA_v1 이 다시 켜지게 되면 §"TCP 켜기" 절의 계산 변환값(182.x)으로 되돌릴 것.
# 주의: connect() 는 set_tool/set_tcp 를 디스커버리 대기 없이 부르면 멈춘다 (#19, #21).
#       그래서 connect() 에는 tool/tcp 를 None 으로 주고, 서비스 대기 뒤에 main() 에서 직접 켠다
#       (ROBOT_CFG 의 tool/tcp 가 있을 때만).

DO_PICK = True             # 집기 → 4줄 발현 → 반환 전체 확인 (2026-09-20)
PICK_PRESET = "close"       # DO_PICK=True 일 때 피크를 물 프리셋. yaml gripper.presets 의 이름
STEP_CONFIRM = True         # True: 단계마다 Enter 를 기다린다 (첫 실행 권장)
SERVICE_WAIT_SEC = 90.0     # 서비스 디스커버리 대기 상한. 디스커버리가 불안정해(#23) 여유를 크게 둔다
_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "src", "tunio_robot", "config", "teach_points.yaml")

# ─────────────────────────── [교시값] ────────────────────────────────
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]                    # 두산 기본 홈
STANDBY_J = [-1.69, 44.18, 31.30, 0.13, 104.52, 90.34]       # 기타 위 대기 자세 (관절각)

# n번 줄 접촉점 — 피크 끝이 줄에 살짝 닿는 Base 절대 posx. **TCP 없음(플랜지) 기준.**
#   2026-09-19 실기로 4줄 다 소리 확인된 값. ROBOT_CFG["tcp"]=None 과 짝이다.
#   GripperDA_v1 이 다시 켜지면 Z 에 +208(툴 Z 방향)을 더한 182.x 값으로 바꿀 것
#   (계산: 6→182.28  5→182.18  2→181.78  1→181.32, X·Y·자세는 그대로).
CONTACT_X = {
    6: [645.85, -29.08, 390.28, 18.53, -180.0, 111.67],
    5: [645.23, -19.14, 390.18, 31.81, 180.0, 124.62],
    2: [645.24, -1.0, 389.78, 53.43, -180.0, 145.63],
    1: [645.34, 8.06, 389.32, 56.94, -180.0, 148.73],
}
STRINGS_TO_PLUCK = [6, 5, 2, 1]

# 피크 거치대 — 피크를 무는 자리. Base 절대 posx (DO_PICK=True 일 때만 쓴다)
PICK_X = [591.63, -143.75, 380.0, 178.55, 180.0, -90.0]

# 줄별 스트로크 방향. pluck_stroke_tool([-8,0,0]) 에 **곱하는** 값이다.
#   6·5번: +1 → 그대로 -8mm (사람 시점 오른쪽)
#   2·1번: -1 → +8mm (사람 시점 왼쪽). 빈 공간이 반대라 훑는 방향이 반대다.
#   2026-09-20 실기 확인. 확정되면 teach_points.yaml pluck.<줄>.stroke_sign 으로 옮긴다
STROKE_SIGN = {6: +1, 5: +1, 2: -1, 1: -1}

# ─────────────────────────── [동작 파라미터] ─────────────────────────
# 값은 pluck_test.py 에서 검증한 것. pluck_string 이 motion.get(키, 기본값) 으로 읽는다.
# 나중에 teach_points.yaml 의 motion 절로 그대로 옮긴다.
MOTION = {
    "vel": 30, "acc": 30,                 # movej (대기 자세)
    "pluck_up_mm": 20.0,                  # 접촉점 위 대기 높이 (Base +Z)
    "pluck_stroke_tool": [-8.0, 0.0, 0.0, 0.0, 0.0, 0.0],   # 옆으로 살짝 (Tool 상대, mm)
    "pluck_retract_tool": [0.0, 0.0, -8.0, 0.0, 0.0, 0.0],  # 줄에서 빠져나옴 (5mm 이상)
    "pluck_vel_approach": 100, "pluck_acc_approach": 100,   # 접근점까지
    "pluck_vel_fine": 20, "pluck_acc_fine": 50,             # 접근점 → 접촉점
    "pluck_vel_stroke": 100, "pluck_acc_stroke": 200,       # 스트로크·후퇴
    "pick_up_mm": 50.0,                   # 거치대 위 대기 높이 (Base +Z)
    "pick_vel_approach": 100, "pick_acc_approach": 100,     # 거치대 위까지
    "pick_vel_fine": 30, "pick_acc_fine": 50,               # 거치대 위 → 거치대
    "pick_preset": PICK_PRESET,
}


def _confirm(msg):
    print(msg)
    if STEP_CONFIRM:
        input("    >>> Enter 를 누르면 진행 (Ctrl+C 중단): ")


def _log_pose(tag):
    try:
        px, sol = MP.read_posx()
        print(f"[{tag}] posx={[round(v, 2) for v in px]} sol={sol}")
        f = MP.read_force()
        if isinstance(f, (list, tuple)) and len(f) == 6:
            print(f"[{tag}] tool_force={[round(v, 2) for v in f]}")
    except Exception as e:                       # noqa: BLE001 — 로그 실패는 진행을 막지 않는다
        print(f"[{tag}] 상태 읽기 실패: {e}")


def _print_ros_env():
    """브링업과 같은 도메인·디스커버리 설정인지 한눈에 보게 찍는다 (#21 재발 방지)."""
    keys = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT",
            "ROS_LOCALHOST_ONLY")
    print("ROS 환경:", "  ".join(f"{k}={os.environ.get(k, '(없음)')}" for k in keys))
    print("  ↑ 브링업을 띄운 터미널과 같아야 한다. 다르면 서비스가 하나도 안 보인다")


def _wait_services(node, robot_id):
    """
    DSR_ROBOT2 는 디스커버리를 기다리지 않는다. 호출 전에 서비스가 보이는지 확인한다.

    반드시 **DSR 이 쓰는 그 노드**(MP._node)로 기다려야 한다. 다른 노드로 기다리면
    그 노드만 디스커버리가 끝나고, DSR 노드는 아직이라 첫 호출이 영원히 멈춘다.

    디스커버리가 불안정해(이슈 #23) 조용히 30초씩 막혀 있으면 멈춘 건지 기다리는
    중인지 구분이 안 된다. 5초마다 지금 보이는 서비스 수를 찍어 진행 상황을 보여준다.
    """
    from dsr_msgs2.srv import GetCurrentPosj, GetCurrentTcp, GetCurrentTool, MoveJoint, MoveLine
    targets = [
        (GetCurrentTool, "dsr_controller2/tool/get_current_tool"),
        (GetCurrentTcp, "dsr_controller2/tcp/get_current_tcp"),
        (GetCurrentPosj, "dsr_controller2/aux_control/get_current_posj"),
        (MoveJoint, "dsr_controller2/motion/move_joint"),
        (MoveLine, "dsr_controller2/motion/move_line"),
    ]
    t0 = time.monotonic()
    while True:
        n = len(node.get_service_names_and_types())
        elapsed = time.monotonic() - t0
        missing = []
        for srv_type, name in targets:
            cli = node.create_client(srv_type, name)
            ok = cli.wait_for_service(timeout_sec=0.0)
            node.destroy_client(cli)
            if not ok:
                missing.append(name)
        print(f"  … {elapsed:5.1f}초  서비스 {n}개 보임  아직 없음: {missing or '없음'}")
        if not missing:
            print(f"  전부 보임 ({elapsed:.1f}초)")
            break
        if elapsed >= SERVICE_WAIT_SEC:
            raise RuntimeError(
                f"{SERVICE_WAIT_SEC:.0f}초 안에 못 찾음: {missing}. "
                f"디스커버리 불안정(이슈 #23) — 재시도하거나 tools/tcp_diag.py 로 더 길게 확인")
        time.sleep(5.0)
    time.sleep(1.0)   # 나머지 클라이언트(DSR_ROBOT2 가 만든 것) 디스커버리 여유


def _apply_tcp(api):
    """디스커버리 뒤에 tool·tcp 를 켠다. 실패 원인을 진단할 수 있게 전후 값을 찍는다."""
    print(f"[0] 시도 전 - 활성 tool='{api.get_tool()}' tcp='{api.get_tcp()}'")

    tool_ret = api.set_tool(ROBOT_CFG["tool"])
    note = "  (실패: TP 에 이 이름이 없다)" if tool_ret == -1 else ""
    print(f"[0] set_tool('{ROBOT_CFG['tool']}') -> {tool_ret}{note}")

    tcp_ret = api.set_tcp(ROBOT_CFG["tcp"])
    note = "  (실패: TP 에 이 이름이 없다)" if tcp_ret == -1 else ""
    print(f"[0] set_tcp('{ROBOT_CFG['tcp']}') -> {tcp_ret}{note}")

    active_tcp = api.get_tcp()
    if active_tcp != ROBOT_CFG["tcp"]:
        msg = (
            f"활성 TCP 가 '{active_tcp}' 다. '{ROBOT_CFG['tcp']}' 이어야 좌표가 맞는다.\n"
            "  set_tool/set_tcp 가 즉시 -1 을 반환했다면 통신은 됐고 TP 가 그 이름을"
            " 못 찾았다는 뜻이다.\n"
            "  펜던트 Workcell Manager 에서 Tool·TCP 등록 이름을 정확히 확인하라"
            " (철자·대소문자·공백,\n"
            "  컨트롤러 재시작/프로젝트 전환으로 초기화됐을 수 있음)."
        )
        raise SystemExit(msg)
    print(f"[0] 활성 TCP 확인 완료: '{active_tcp}'")


def main():
    _print_ros_env()
    rclpy.init()

    gripper_cfg = None
    if DO_PICK:
        with open(_YAML, "r", encoding="utf-8") as f:
            gripper_cfg = yaml.safe_load(f)["gripper"]      # 프리셋 표는 yaml 이 단일 출처
        if PICK_PRESET not in gripper_cfg["presets"]:
            raise SystemExit(f"PICK_PRESET '{PICK_PRESET}' 이 yaml gripper.presets 에 없다: "
                             f"{sorted(gripper_cfg['presets'])}")

    print(f"[0] 두산 API 연결 (connect) — 그리퍼 {'사용' if DO_PICK else '없이'}")
    api = MP.connect(dict(ROBOT_CFG, tool=None, tcp=None), gripper_cfg=gripper_cfg)
    # connect() 가 만든 DSR 전용 노드로 서비스 디스커버리를 기다린다 (이걸 빼면 첫 호출에서 멈춘다)
    _wait_services(MP._node, ROBOT_CFG["id"])
    if ROBOT_CFG["tool"] or ROBOT_CFG["tcp"]:
        _apply_tcp(api)                          # 둘 다 None 이면 TP 에 켜져 있는 걸 그대로 쓴다
    else:
        print(f"[0] TCP 전환 안 함 (TP 그대로) — 활성 tool='{api.get_tool()}' tcp='{api.get_tcp()}'")

    home = Pose("frames.home", True, joints=HOME_J)
    standby = Pose("frames.standby", True, joints=STANDBY_J)
    holder = Pose("pick.holder", True, task=PICK_X)
    holder.grip_preset = PICK_PRESET
    if not any(abs(v) > 1e-6 for v in STANDBY_J):
        # 전부 0 인 관절각은 팔이 수직으로 펴지는 자세다. 교시 전 placeholder 로 움직이면 안 된다
        raise SystemExit("STANDBY_J 가 비어 있다(전부 0). 기타 위 대기 자세를 먼저 채워라")
    if DO_PICK and not any(abs(v) > 1e-6 for v in PICK_X):
        raise SystemExit("PICK_X 가 비어 있다(전부 0). 거치대 좌표를 먼저 채워라")

    try:
        _confirm("[1] 홈 자세로")
        MP._movej(home.joints, MOTION)
        _log_pose("home")

        if DO_PICK:
            _confirm(f"[2] 피크 집기: 열기 → 거치대 위 {MOTION['pick_up_mm']:.0f}mm → 거치대 → "
                     f"닫기('{PICK_PRESET}') → 위 → 대기 자세")
            MP.pick_up(holder, MOTION, standby=standby)
            _log_pose("picked")

        for n in STRINGS_TO_PLUCK:
            pose = Pose(f"pluck.{n}", True, task=CONTACT_X[n])   # 더미 Pose (접촉점, Base posx)
            pose.stroke_sign = STROKE_SIGN.get(n, 1)              # 이 줄의 스트로크 방향
            if not any(abs(v) > 1e-6 for v in pose.task):
                print(f"[{n}번] CONTACT_X 가 비어 있다 — 건너뜀")
                continue
            stroke = [v * pose.stroke_sign for v in MOTION['pluck_stroke_tool'][:3]]
            _confirm(f"[{n}번 줄] 대기 → 위 {MOTION['pluck_up_mm']:.0f}mm → 접촉 → "
                     f"스트로크 {stroke} → "
                     f"후퇴 {MOTION['pluck_retract_tool'][:3]} → 위 → 대기")
            MP.pluck_string(pose, MOTION, standby=standby)
            _log_pose(f"after_{n}")
            print(f"[{n}번 줄] 완료 — 소리가 났는지 확인")

        if DO_PICK:
            _confirm(f"[3] 피크 반환: 거치대 위 {MOTION['pick_up_mm']:.0f}mm → 거치대 → 열기 → 위 → 홈")
            MP.pick_return(holder, MOTION, home=home)
            _log_pose("returned")
        else:
            _confirm("[끝] 홈 복귀")
            MP._movej(home.joints, MOTION)
        print("완료")

    except KeyboardInterrupt:
        print("사용자 중단")
    except Exception as e:                       # noqa: BLE001
        print(f"로봇 오류: {type(e).__name__}: {e}")
    finally:
        try:
            print("[finally] 홈 복귀")
            MP._movej(home.joints, MOTION)
        except Exception as e:                   # noqa: BLE001
            print(f"홈 복귀 실패: {e}")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
