"""
tcp_diag — set_tool/set_tcp 가 -1 을 반환하는 원인 진단. 로봇은 전혀 움직이지 않는다.

DSR_ROBOT2.py 를 보면 set_tcp/set_tool 의 -1 은 통신 실패가 아니라
'result.success == False', 즉 컨트롤러가 명시적으로 거부한 것이다 (이슈 #19/#21 후속).
로봇 상태(모드·서보·알람)를 먼저 읽어 "이름 문제"인지 "상태 문제"인지를 가른다.

실행:
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇IP>
  source install/setup.bash
  python3 tools/tcp_diag.py
"""
import os
import sys
import time

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "tunio_robot")
if os.path.isdir(_SRC):
    sys.path.append(os.path.abspath(_SRC))

import rclpy  # noqa: E402
from tunio_robot import motion_primitives as MP  # noqa: E402

ROBOT_CFG = {"id": "dsr01", "model": "m0609", "tool": None, "tcp": None}
TOOL_NAME = "Tool Weight"
TCP_NAME = "GripperDA_v1"


def _print_ros_env():
    keys = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT")
    print("ROS 환경:", "  ".join(f"{k}={os.environ.get(k, '(없음)')}" for k in keys))


def _wait_services(node, robot_id, total_wait=90.0):
    from dsr_msgs2.srv import (GetCurrentTcp, GetCurrentTool, GetLastAlarm, GetRobotMode,
                               GetRobotState, MoveLine, SetCurrentTcp, SetCurrentTool)
    targets = [
        (GetCurrentTool, "dsr_controller2/tool/get_current_tool"),
        (GetCurrentTcp, "dsr_controller2/tcp/get_current_tcp"),
        (SetCurrentTool, "dsr_controller2/tool/set_current_tool"),
        (SetCurrentTcp, "dsr_controller2/tcp/set_current_tcp"),
        (GetRobotMode, "dsr_controller2/system/get_robot_mode"),
        (GetRobotState, "dsr_controller2/system/get_robot_state"),
        (GetLastAlarm, "dsr_controller2/system/get_last_alarm"),
        (MoveLine, "dsr_controller2/motion/move_line"),
    ]
    t0 = time.monotonic()
    while True:
        n = len(node.get_service_names_and_types())
        elapsed = time.monotonic() - t0
        print(f"  … {elapsed:5.1f}초  이 노드가 보는 서비스 {n}개")
        missing = []
        for srv_type, name in targets:
            cli = node.create_client(srv_type, name)
            ok = cli.wait_for_service(timeout_sec=0.0)
            node.destroy_client(cli)
            if not ok:
                missing.append(name)
        if not missing:
            print(f"  전부 보임 ({elapsed:.1f}초)")
            break
        if elapsed >= total_wait:
            raise RuntimeError(f"{total_wait:.0f}초 안에 못 찾음: {missing}")
        print(f"  아직 없음: {missing}")
        time.sleep(5.0)
    time.sleep(1.0)


def main():
    _print_ros_env()
    rclpy.init()
    print("[0] connect (그리퍼 없이)")
    api = MP.connect(ROBOT_CFG)
    _wait_services(MP._node, ROBOT_CFG["id"])

    print("\n==== 로봇 상태 (이동·설정 변경 없음) ====")
    try:
        mode = api.get_robot_mode()
        print(f"get_robot_mode() = {mode}  (0=MANUAL, 1=AUTONOMOUS 가 일반적)")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_robot_mode() 실패: {e}")
    try:
        state = api.get_robot_state()
        print(f"get_robot_state() = {state}")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_robot_state() 실패: {e}")
    try:
        alarm = api.get_last_alarm()
        print(f"get_last_alarm() = {alarm}")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_last_alarm() 실패: {e}")
    try:
        ctrl_mode = api.get_control_mode()
        print(f"get_control_mode() = {ctrl_mode}"
              "  (성현 2026-09-20: '서보를 펜던트가 쥐고 있다' 제보 확인용)")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_control_mode() 실패: {e}")
    try:
        ctrl_space = api.get_control_space()
        print(f"get_control_space() = {ctrl_space}")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_control_space() 실패: {e}")

    print("\n==== 현재 등록된 tool/tcp (변경 전) ====")
    print(f"get_tool() = '{api.get_tool()}'")
    print(f"get_tcp()  = '{api.get_tcp()}'")

    print("\n==== 위치 조회만 (이동 아님, 서비스가 살아있는지) ====")
    try:
        posx, sol = api.get_current_posx(api.DR_BASE)
        print(f"get_current_posx() = {[round(v, 1) for v in posx]} sol={sol}")
    except Exception as e:                                       # noqa: BLE001
        print(f"get_current_posx() 실패: {e}")

    print(f"\n==== set_tool('{TOOL_NAME}') 시도 ====")
    print(f"-> {api.set_tool(TOOL_NAME)}")
    print(f"get_tool() 재확인 = '{api.get_tool()}'")

    print(f"\n==== set_tcp('{TCP_NAME}') 시도 ====")
    print(f"-> {api.set_tcp(TCP_NAME)}")
    print(f"get_tcp() 재확인 = '{api.get_tcp()}'")

    test_name = "tunio_diag_test_tcp"
    print(f"\n==== 새 TCP 만들어서 확인: add_tcp('{test_name}', [0,0,208,0,0,0]) ====")
    print("  (기존 이름 문제인지, API 자체가 막혀 있는지 가른다)")
    ret = api.add_tcp(test_name, [0.0, 0.0, 208.0, 0.0, 0.0, 0.0])
    print(f"add_tcp() -> {ret}")
    if ret == 0:
        print(f"  생성 성공 -> set_tcp('{test_name}') 시도")
        print(f"  -> {api.set_tcp(test_name)}")
        print(f"  get_tcp() = '{api.get_tcp()}'")
        print(f"  정리: del_tcp('{test_name}') -> {api.del_tcp(test_name)}")
    else:
        print("  생성도 실패 -> add_tcp/set_tcp API 자체가 지금 막혀 있다 (제어권·권한 문제)")

    print("\n==== 읽는 법 ====")
    print("get_robot_mode/state 가 정상(대기/자동)인데 set_tool/set_tcp 만 -1 이면")
    print("  → 등록 자체나 제어권 문제. 펜던트에서 Tool/TCP 목록을 직접 확인하라.")
    print("get_robot_mode/state 가 비정상(정지/알람)이면")
    print("  → 보호정지 해제·서보 온·자동 모드 전환 후 다시 시도하라.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
