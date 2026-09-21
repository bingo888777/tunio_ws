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
    # 2026-09-21 — None(플랜지 유지)에서 실제 이름으로 되돌렸다. move.py 와 같은 이름이다.
    "tool": "Tool Weight",
    "tcp": "GripperDA_v1",
}
# 왜 되돌렸나 (2026-09-21)
#   · connect() 가 **항상** set_tool/set_tcp 를 부르고 get 으로 확인하게 바뀌었다(#26).
#     배포용이라 펜던트 설정에 기대면 안 된다 — 설정이 안 된 로봇에서도 돌아야 한다
#   · connect() 안에 디스커버리 대기가 들어가 예전 멈춤(#19·#21)의 원인이 없어졌다
#   · 판정을 set 반환값이 아니라 get 결과로 하므로, set 이 -1 로 거부돼도(#23)
#     펜던트가 켜 뒀으면 그대로 진행한다. 둘 다 아니면 움직이기 전에 멈춘다
#   · 아래 CONTACT_X·PICK_X 도 이 TCP 기준으로 같이 옮겼다. 짝이 안 맞으면 208mm 어긋난다

DO_PICK = True             # 집기 → 4줄 발현 → 반환 전체 확인 (2026-09-20)
PICK_PRESET = "close"       # DO_PICK=True 일 때 피크를 물 프리셋. yaml gripper.presets 의 이름
STEP_CONFIRM = True         # True: 단계마다 Enter 를 기다린다 (첫 실행 권장)
SERVICE_WAIT_SEC = 90.0     # 서비스 디스커버리 대기 상한. 디스커버리가 불안정해(#23) 여유를 크게 둔다
_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "src", "tunio_robot", "config", "teach_points.yaml")

# ─────────────────────────── [교시값] ────────────────────────────────
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]                    # 두산 기본 홈
STANDBY_J = [-1.69, 44.18, 31.30, 0.13, 104.52, 90.34]       # 기타 위 대기 자세 (관절각)

# n번 줄 접촉점 — 피크 끝이 줄에 살짝 닿는 Base 절대 posx.
# **ROBOT_CFG["tcp"](=GripperDA_v1) 기준이다.**
#
#   원래 값은 플랜지 기준(Z 약 390)이었고, 2026-09-19 실기로 4줄 다 소리까지
#   확인된 것이다. 2026-09-21 에 GripperDA_v1 을 다시 켜기로 하면서 성현이 이
#   자리에 적어 둔 계산값으로 옮겼다 — "Z 에 +208(툴 Z 방향), X·Y·자세는 그대로".
#   kinematics.shift_tcp 로 다시 계산해도 같은 값이 나온다. 이 자세들은 B=±180
#   (아래를 본다)이라 툴 Z 가 Base -Z 와 나란해서 Z 만 208 내려간다.
#
#   ⚠️ 옮긴 값 자체는 실기로 안 돌려봤다. 산술만 맞다. 처음 한 번은 천천히.
#      되돌리려면 Z 에 +208 을 더하고 ROBOT_CFG 의 tool/tcp 를 None 으로 두면 된다.
CONTACT_X = {
    6: [645.85, -29.08, 182.28, 18.53, -180.0, 111.67],
    5: [645.23, -19.14, 182.18, 31.81, 180.0, 124.62],
    2: [645.24, -1.0, 181.78, 53.43, -180.0, 145.63],
    1: [645.34, 8.06, 181.32, 56.94, -180.0, 148.73],
}
STRINGS_TO_PLUCK = [6, 5, 2, 1]

# 피크 거치대 — 피크를 무는 자리. Base 절대 posx (DO_PICK=True 일 때만 쓴다)
#   위 CONTACT_X 와 같이 GripperDA_v1 기준으로 옮겼다 (2026-09-21).
#   플랜지 기준 원값 [591.63, -143.75, 380.0, 178.55, 180.0, -90.0] 에서 Z -208.
#   (369.46 → 380.0 은 성현 2026-09-21 실측 — 너무 깊게 물려 10.5mm 얕게, PR #30)
#   teach_points.yaml 의 pick.holder 도 이 값(591.63, -143.75, TCP 172.0)으로
#   맞춰뒀다 — 09-19 교시값과의 93mm 차이는 거치대가 옮겨진 것으로 결론 났다 (#32)
PICK_X = [591.63, -143.75, 172.0, 178.55, 180.0, -90.0]

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
    # connect() 가 디스커버리 대기 → set_tool/set_tcp → get 으로 확인까지 한다 (#26).
    # 예전에는 tool/tcp 를 None 으로 넘기고 여기서 따로 켰는데, 그러면 배포 경로와
    # 이 도구가 서로 다른 순서로 돌아 무엇을 검증했는지 흐려진다. 이제 같은 길을 쓴다
    api = MP.connect(dict(ROBOT_CFG, service_wait_sec=SERVICE_WAIT_SEC),
                     gripper_cfg=gripper_cfg)
    print(f"[0] 활성 tool='{api.get_tool()}' tcp='{api.get_tcp()}' "
          f"(CONTACT_X·PICK_X 가 이 TCP 기준이어야 한다)")

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
