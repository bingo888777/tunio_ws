"""
tcp_diag — set_tool/set_tcp 가 -1 을 반환하는 원인 진단. 로봇은 전혀 움직이지 않는다.

DSR_ROBOT2.py 를 보면 set_tcp/set_tool 의 -1 은 통신 실패가 아니라
'result.success == False', 즉 컨트롤러가 명시적으로 거부한 것이다 (이슈 #19/#21 후속).
로봇 상태(모드·서보·알람)를 먼저 읽어 "이름 문제"인지 "상태 문제"인지를 가른다.

--manual — 자동/수동 모드를 갈라 본다 (2026-09-21 추가, 이슈 #23 후속)
  두산 드라이버 소스를 따라가면 set_current_tcp_cb 는 Drfl->set_tcp() 를 그대로
  넘기는 두 줄이다. 거부는 닫힌 libDRFL.a 안에서 난다 — ROS 쪽에서는 더 못 본다.
  남은 가설 중 제일 그럴듯한 게 **로봇 모드**다. 조회·이동은 되는데 설정 변경
  계열(set_tool/set_tcp/add_tcp)만 통째로 막히는 게, 설정 변경을 수동 모드에서만
  받는 경우의 증상과 맞는다.
  이 옵션은 현재 모드에서 한 번 재고, 수동으로 바꿔 한 번 더 재고, **원래 모드로
  되돌린다.** 두 결과를 나란히 찍어 모드가 원인인지 아닌지 바로 갈린다.

  ⚠️ 수동 모드에서는 외부 이동 명령이 거부될 수 있고 직접교시가 켜질 수 있다.
     이 도구는 아무것도 움직이지 않지만, 돌리기 전에 팔 주변을 치울 것.
  ⚠️ 되돌리기는 Ctrl+C 와 예외에도 돈다. 단 kill -9 로는 못 막으니 그 뒤에는
     펜던트에서 모드를 눈으로 확인할 것.

실행:
  ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=<로봇IP>
  source install/setup.bash
  python3 tools/tcp_diag.py
  python3 tools/tcp_diag.py --manual
"""
import argparse
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

# DRFC.h 의 ROBOT_MODE. 0 부터 순서대로다
MODE_MANUAL = 0
MODE_AUTONOMOUS = 1
MODE_NAMES = {0: "MANUAL(수동)", 1: "AUTONOMOUS(자동)", 2: "MEASURE(측정)"}


def mode_name(m):
    return MODE_NAMES.get(m, f"알 수 없음({m})")


def _print_ros_env():
    keys = ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER", "ROS_SUPER_CLIENT")
    print("ROS 환경:", "  ".join(f"{k}={os.environ.get(k, '(없음)')}" for k in keys))


def _wait_services(node, robot_id, total_wait=90.0):
    from dsr_msgs2.srv import (GetCurrentTcp, GetCurrentTool, GetLastAlarm, GetRobotMode,
                               GetRobotState, MoveLine, SetCurrentTcp, SetCurrentTool,
                               SetRobotMode)
    targets = [
        (GetCurrentTool, "dsr_controller2/tool/get_current_tool"),
        (GetCurrentTcp, "dsr_controller2/tcp/get_current_tcp"),
        (SetCurrentTool, "dsr_controller2/tool/set_current_tool"),
        (SetCurrentTcp, "dsr_controller2/tcp/set_current_tcp"),
        (GetRobotMode, "dsr_controller2/system/get_robot_mode"),
        # --manual 이 쓴다. 없으면 모드 전환 자체를 못 하므로 여기서 먼저 걸린다
        (SetRobotMode, "dsr_controller2/system/set_robot_mode"),
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


def read_mode(api):
    """
    현재 로봇 모드를 읽는다. 못 읽으면 None.

    래퍼가 실패를 -1 로 돌려주기도 하고 UnboundLocalError 를 내기도 한다
    (except 절이 ret 을 안 채운 채 return ret 을 탄다). 둘 다 None 으로 본다.
    """
    try:
        m = api.get_robot_mode()
    except Exception as e:                                       # noqa: BLE001
        print(f"  get_robot_mode() 실패: {e}")
        return None
    if not isinstance(m, int) or m < 0:
        print(f"  get_robot_mode() 가 모드로 볼 수 없는 값을 줬다: {m!r}")
        return None
    return m


def set_mode(api, want):
    """
    모드를 바꾸고 **읽어서** 확인한다. 바뀌었으면 True.

    set_robot_mode 의 반환은 뒤집혀 있다 — 0 이 성공이고 -1 이 실패다.
    그래서 진릿값으로 쓰면 안 된다. 판정은 반환값이 아니라 get 으로 한다
    (set_tcp 를 get 으로 판정하는 것과 같은 이유다. CLAUDE.md 3.5.1).
    """
    try:
        ret = api.set_robot_mode(want)
    except Exception as e:                                       # noqa: BLE001
        print(f"  set_robot_mode({want}) 예외: {e}")
        ret = None
    time.sleep(0.5)                                              # 반영될 틈
    now = read_mode(api)
    ok = (now == want)
    print(f"  set_robot_mode({want}) -> {ret}  (0=성공, -1=실패)  "
          f"지금 모드 = {mode_name(now) if now is not None else '못 읽음'}")
    return ok


def try_settings(api, label):
    """set_tool / set_tcp / add_tcp 셋을 시도하고 결과를 돌려준다. 안 움직인다."""
    print(f"\n{'=' * 62}")
    print(f"설정 변경 시도 — {label}")
    print("=" * 62)
    out = {}

    print(f"\n-- set_tool('{TOOL_NAME}')")
    out["set_tool"] = api.set_tool(TOOL_NAME)
    print(f"   -> {out['set_tool']}   get_tool() = '{api.get_tool()}'")

    print(f"\n-- set_tcp('{TCP_NAME}')")
    out["set_tcp"] = api.set_tcp(TCP_NAME)
    print(f"   -> {out['set_tcp']}   get_tcp() = '{api.get_tcp()}'")

    test_name = "tunio_diag_test_tcp"
    print(f"\n-- add_tcp('{test_name}', [0,0,208,0,0,0])")
    print("   (기존 이름 문제인지, API 자체가 막혀 있는지 가른다)")
    out["add_tcp"] = api.add_tcp(test_name, [0.0, 0.0, 208.0, 0.0, 0.0, 0.0])
    print(f"   -> {out['add_tcp']}")
    if out["add_tcp"] == 0:
        print(f"   생성 성공 -> set_tcp('{test_name}') -> {api.set_tcp(test_name)}")
        print(f"   get_tcp() = '{api.get_tcp()}'")
        print(f"   정리: del_tcp('{test_name}') -> {api.del_tcp(test_name)}")
    else:
        print("   생성도 실패 -> add_tcp/set_tcp API 자체가 지금 막혀 있다")
    return out


def all_ok(res):
    return all(v == 0 for v in res.values())


def main():
    ap = argparse.ArgumentParser(
        description="set_tool/set_tcp 거부 원인 진단. 로봇은 움직이지 않는다")
    ap.add_argument("--manual", action="store_true",
                    help="수동 모드에서도 재보고 원래 모드로 되돌린다. "
                         "모드가 원인인지 가른다 (이슈 #23)")
    args = ap.parse_args()

    _print_ros_env()
    rclpy.init()
    print("[0] connect (그리퍼 없이)")
    api = MP.connect(ROBOT_CFG)
    _wait_services(MP._node, ROBOT_CFG["id"])

    print("\n==== 로봇 상태 (이동·설정 변경 없음) ====")
    start_mode = read_mode(api)
    print(f"get_robot_mode() = {start_mode}  ({mode_name(start_mode)})")
    # get_robot_state/get_last_alarm/get_control_mode/get_control_space/
    # get_current_posx 는 뺐다 — spin_until_future_complete 에 타임아웃이 없어(#31)
    # 이 현장에서 매번 다른 호출이 무작위로 멈춘다(2026-09-23 실기, get_last_alarm
    # 도 한 번은 되고 한 번은 멈췄다). 핵심 질문(모드가 set_tool/set_tcp 거부의
    # 원인인가)과 무관한 진단성 출력이라 전부 뺀다 — 남는 호출 수를 줄이는 게
    # 곧 멈출 확률을 줄이는 길이다.

    print("\n==== 현재 등록된 tool/tcp (변경 전) ====")
    print(f"get_tool() = '{api.get_tool()}'")
    print(f"get_tcp()  = '{api.get_tcp()}'")

    before = try_settings(api, f"현재 모드 — {mode_name(start_mode)}")
    after = None
    switched = False

    if args.manual:
        if start_mode is None:
            print("\n--manual: 현재 모드를 못 읽어서 전환하지 않는다. "
                  "되돌릴 데를 모르는 채로 바꾸면 안 된다")
        elif start_mode == MODE_MANUAL:
            print("\n--manual: 이미 수동 모드다. 전환할 것이 없다")
        else:
            try:
                print(f"\n{'=' * 62}")
                print("수동 모드로 전환한다")
                print("=" * 62)
                print("  ⚠️ 외부 이동 명령이 거부될 수 있고 직접교시가 켜질 수 있다.")
                print("     이 도구는 아무것도 움직이지 않는다. 팔 주변은 치워 둘 것")
                switched = set_mode(api, MODE_MANUAL)
                if switched:
                    after = try_settings(api, "수동 모드 — MANUAL")
                else:
                    print("  전환 실패. 모드 변경 자체가 거부됐다 —")
                    print("  펜던트가 제어권을 쥐고 있을 가능성이 크다. "
                          "펜던트에서 직접 수동으로 바꾸고 --manual 없이 다시 돌려라")
            finally:
                # Ctrl+C 와 예외에도 반드시 돈다. 되돌리는 걸 사람이 기억하게 두지 않는다
                if switched:
                    print(f"\n{'=' * 62}")
                    print(f"원래 모드로 되돌린다 — {mode_name(start_mode)}")
                    print("=" * 62)
                    if not set_mode(api, start_mode):
                        print("  ⚠️ 되돌리기 실패. **펜던트에서 모드를 직접 확인하라.**")

    print(f"\n{'=' * 62}")
    print("읽는 법")
    print("=" * 62)
    if after is not None:
        print(f"  현재 모드({mode_name(start_mode)})  set_tool {before['set_tool']} / "
              f"set_tcp {before['set_tcp']} / add_tcp {before['add_tcp']}")
        print(f"  수동 모드(MANUAL)      set_tool {after['set_tool']} / "
              f"set_tcp {after['set_tcp']} / add_tcp {after['add_tcp']}")
        print("  (0 = 성공, -1 = 거부)")
        if all_ok(after) and not all_ok(before):
            print("\n  → **원인은 로봇 모드다.** 설정 변경은 수동 모드에서만 받는다.")
            print("     배포 시 connect() 가 set 앞뒤로 모드를 바꾸게 하거나,")
            print("     펜던트에서 켜 두는 지금 방식을 유지한다 (이슈 #23)")
        elif all_ok(before):
            print("\n  → 애초에 현재 모드에서도 됐다. 이 현장은 이미 풀린 상태다")
        else:
            print("\n  → **모드가 원인이 아니다.** 수동에서도 거부된다.")
            print("     남은 후보는 제어권(펜던트 점유)과 컨트롤러 설정이다.")
            print("     dsr_controller2.cpp 2798행에서 제어권 요청이 주석 처리돼 있다 —")
            print("     그쪽을 살려 보는 것이 다음 단계다")
    else:
        print("  모드·상태가 정상인데 set_tool/set_tcp 만 -1 이면")
        print("    → 등록 자체나 제어권 문제. `--manual` 로 모드부터 갈라 보라")
        print("  모드·상태가 비정상(정지·알람)이면")
        print("    → 보호정지 해제·서보 온 뒤 다시 시도하라")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
