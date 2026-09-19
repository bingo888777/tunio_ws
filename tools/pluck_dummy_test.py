"""
pluck_dummy_test — robot_controller 없이 motion_primitives.pluck_string 을 직접 부른다.

목적: 성현 담당 pluck_string 1차 구현(위치 제어만)을 로봇에 붙여 확인한다.
      Action·YAML·컨트롤러를 거치지 않고, 여기서 Pose 객체와 motion dict 를
      손으로 만들어 넘긴다 (더미 데이터).

전제: 홈 자세에서 RG2 가 피크를 이미 잡고 있다. 피크 집기·반환은 2단계.

흐름 (성현 2026-09-19):
  홈 → 기타 위 대기 → [줄마다] 접촉점 위 2cm → 접촉점 → 옆으로 살짝(발현)
  → 즉시 후퇴 → 접촉점 위 2cm → 기타 위 대기 → (다음 줄) ... → 홈

실행:
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇IP> gripper:=rg2
  source install/setup.bash        # tunio_robot 을 import 하려면 (빌드 전이면 src 경로로 대체됨)
  python3 tools/pluck_dummy_test.py

==== [교시값] 블록만 채우면 된다 ==========================================
  HOME_J, STANDBY_J : 관절각 6개 (get_current_posj 로 읽은 값)
  CONTACT_X[n]      : n번 줄 접촉점. Base 절대 posx [X,Y,Z,A,B,C] (get_current_posx)
                      교시할 때 켜져 있던 TCP 가 ROBOT_CFG["tcp"] 와 같아야 한다
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

# ─────────────────────────── [로봇 설정] ─────────────────────────────
ROBOT_CFG = {
    "id": "dsr01",
    "model": "m0609",
    "tool": None,          # TP 에 등록된 툴 무게 이름. None 이면 set_tool 생략 (TP 설정 유지)
    "tcp": None,           # TP 에 등록된 피크 TCP 이름. None 이면 set_tcp 생략 (TP 설정 유지)
}
# 그리퍼는 초기화하지 않는다 (connect 에 gripper_cfg 를 안 준다). 피크는 이미 잡고 있음 가정
STEP_CONFIRM = True        # True: 줄마다 Enter 를 기다린다 (첫 실행 권장)

# ─────────────────────────── [교시값] ────────────────────────────────
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]                    # 두산 기본 홈
STANDBY_J = [-1.37, 46.28, 23.98, 0.14, 109.74, 88.99]                   # TODO 기타 위 대기 자세 (관절각)

# n번 줄 접촉점 — 피크 끝이 줄에 살짝 닿는 Base 절대 posx. TODO 4줄 채우기
CONTACT_X = {
    6: [642.93, -29.60, 188.70, 57.46, -175.68, 150.31],
    5: [642.65, -22.09, 183.56, 10.24, 180.0, 100.0],
    2: [642.54, -5.21, 182.97, 6.47, -180.0, 97.10],
    1: [642.78, 2.66, 182.74, 8.86, -180.0, 99.41],
}
STRINGS_TO_PLUCK = [6, 5]

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
    "pluck_vel_stroke": 1000, "pluck_acc_stroke": 2000,     # 스트로크·후퇴
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


def _wait_services(node, robot_id):
    """
    DSR_ROBOT2 는 디스커버리를 기다리지 않는다. 호출 전에 서비스가 보이는지 확인한다.

    반드시 **DSR 이 쓰는 그 노드**(MP._node)로 기다려야 한다. 다른 노드로 기다리면
    그 노드만 디스커버리가 끝나고, DSR 노드는 아직이라 첫 호출이 영원히 멈춘다.
    """
    from dsr_msgs2.srv import GetCurrentPosj, GetCurrentTcp, GetCurrentTool, MoveJoint, MoveLine
    for srv_type, name in [
        (GetCurrentTool, "dsr_controller2/tool/get_current_tool"),
        (GetCurrentTcp, "dsr_controller2/tcp/get_current_tcp"),
        (GetCurrentPosj, "dsr_controller2/aux_control/get_current_posj"),
        (MoveJoint, "dsr_controller2/motion/move_joint"),
        (MoveLine, "dsr_controller2/motion/move_line"),
    ]:
        cli = node.create_client(srv_type, name)
        print(f"서비스 대기: /{robot_id}/{name}")
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"서비스 없음: /{robot_id}/{name} — 브링업이 떠 있는지 확인")
        node.destroy_client(cli)
    time.sleep(1.0)   # 나머지 클라이언트(DSR_ROBOT2 가 만든 것) 디스커버리 여유


def main():
    rclpy.init()

    print("[0] 두산 API 연결 (connect) — 그리퍼 없이")
    api = MP.connect(ROBOT_CFG, gripper_cfg=None)
    # connect() 가 만든 DSR 전용 노드로 서비스 디스커버리를 기다린다 (이걸 빼면 첫 호출에서 멈춘다)
    _wait_services(MP._node, ROBOT_CFG["id"])
    print("[0] 툴/TCP 조회 중...")
    print(f"활성 tool='{api.get_tool()}' tcp='{api.get_tcp()}' (교시 때와 같아야 한다)")

    home = Pose("frames.home", True, joints=HOME_J)
    standby = Pose("frames.standby", True, joints=STANDBY_J)
    if not any(abs(v) > 1e-6 for v in STANDBY_J):
        # 전부 0 인 관절각은 팔이 수직으로 펴지는 자세다. 교시 전 placeholder 로 움직이면 안 된다
        raise SystemExit("STANDBY_J 가 비어 있다(전부 0). 기타 위 대기 자세를 먼저 채워라")

    try:
        _confirm("[1] 홈 자세로")
        MP._movej(home.joints, MOTION)
        _log_pose("home")

        for n in STRINGS_TO_PLUCK:
            pose = Pose(f"pluck.{n}", True, task=CONTACT_X[n])   # 더미 Pose (접촉점, Base posx)
            if not any(abs(v) > 1e-6 for v in pose.task):
                print(f"[{n}번] CONTACT_X 가 비어 있다 — 건너뜀")
                continue
            _confirm(f"[{n}번 줄] 대기 → 위 {MOTION['pluck_up_mm']:.0f}mm → 접촉 → "
                     f"스트로크 {MOTION['pluck_stroke_tool'][:3]} → "
                     f"후퇴 {MOTION['pluck_retract_tool'][:3]} → 대기")
            MP.pluck_string(pose, MOTION, standby=standby)
            _log_pose(f"after_{n}")
            print(f"[{n}번 줄] 완료 — 소리가 났는지 확인")

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
