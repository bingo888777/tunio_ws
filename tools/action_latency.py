#!/usr/bin/env python3
"""
ROS 2 Action 왕복 지연 — goal 전송부터 result 수신까지.

왜 서비스 왕복(ros_latency.py)과 따로 재는가
  Action 은 goal 요청·수락, 결과 요청·응답에 상태 토픽까지 오간다.
  응답 시간 예산에 들어가는 것은 이쪽이다. 로컬에서 잰 Action/서비스 비율을
  원격 서비스 지연에 곱하면 양쪽 노드를 다른 PC 에 띄우지 않고도 추정할 수 있다.

왜 기본이 자체 서버인가 (이슈 #25)
  이 도구의 첫 판은 실제 /robot/adjust_peg 에 goal 을 보냈다. 그런데 AdjustPeg 는
  읽기 전용이 아니다 — 23회 × 5도 = 115도, 6번 줄 기준 -255센트다. 풀리면 탐색
  범위(목표 ±2반음) 밖으로 나가 튜너가 그 줄을 아예 못 찾고, 넥이 펴져 손대지
  않은 줄까지 틀어진다. 그때 막아 준 것은 robot_controller 의 dry_run 기본값
  하나뿐이었고, 그건 이 도구가 확인하지도 않는 남의 노드 설정이었다.

  그래서 기본값을 바꿨다. 이 도구가 자기 Action 서버를 띄우고 거기에 잰다.
  로봇을 전혀 건드리지 않으며, Action 스택 왕복을 재는 목적에는 이걸로 충분하다.

  실제 robot_controller 에 재려면 --target robot 을 준다. 그때는 먼저 그 노드의
  dry_run 파라미터를 읽어 true 가 아니면 거부한다. 읽지 못해도 거부한다.

사용 예
    python3 tools/action_latency.py                    # 자체 서버 (안전, 기본)
    python3 tools/action_latency.py --n 50
    python3 tools/action_latency.py --target robot     # dry_run 확인 후에만 진행
"""
import argparse
import statistics
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters

from tunio_interfaces.action import AdjustPeg

REAL_ACTION = '/robot/adjust_peg'
PROBE_ACTION = '/latency_probe/adjust_peg'   # 이 도구가 직접 띄우는 서버
CONTROLLER_NODE = '/robot_controller'


def check_dry_run(node, ex, timeout=30.0):
    """
    robot_controller 의 dry_run 파라미터를 읽는다.

    반환: (진행해도 되는가, 사람이 읽을 사유)
    확인하지 못하면 진행하지 않는다. 모르는 상태로 페그 goal 을 보내지 않는다.
    """
    cli = node.create_client(GetParameters, f'{CONTROLLER_NODE}/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return False, (f'{CONTROLLER_NODE} 의 파라미터 서비스를 {timeout:.0f}초 안에 '
                       '찾지 못했다. robot_controller 가 떠 있는지 확인하라')
    req = GetParameters.Request()
    req.names = ['dry_run']
    fut = cli.call_async(req)
    ex.spin_until_future_complete(fut, timeout_sec=10.0)
    node.destroy_client(cli)
    if not fut.done() or fut.result() is None:
        return False, 'dry_run 파라미터를 읽지 못했다 (응답 없음)'
    vals = fut.result().values
    if not vals or vals[0].type == 0:     # PARAMETER_NOT_SET
        return False, 'robot_controller 에 dry_run 파라미터가 없다'
    if not vals[0].bool_value:
        return False, ('robot_controller 가 dry_run=false 로 떠 있다. '
                       '이 측정은 실제 페그 조절 goal 을 보내므로 중단한다')
    return True, 'dry_run=true 확인됨'


def measure(node, ex, action_name, n, warmup=3):
    cli = ActionClient(node, AdjustPeg, action_name)
    if not cli.wait_for_server(timeout_sec=30.0):
        print(f'  {action_name} 서버를 30초 안에 찾지 못했다')
        return None

    times, fails = [], 0
    for i in range(n + warmup):
        g = AdjustPeg.Goal()
        # 자체 서버에서는 의미 없는 값이고, robot 모드는 dry_run 을 확인한 뒤에만
        # 여기까지 온다. 그래도 최소 각도, 푸는 방향으로 둔다.
        g.string_number, g.direction, g.angle_deg = 6, -1, 0.1
        t0 = time.perf_counter()
        fut = cli.send_goal_async(g)
        ex.spin_until_future_complete(fut, timeout_sec=10.0)
        if not fut.done() or not fut.result().accepted:
            fails += 1
            continue
        rf = fut.result().get_result_async()
        ex.spin_until_future_complete(rf, timeout_sec=10.0)
        dt = (time.perf_counter() - t0) * 1000.0
        if not rf.done():
            fails += 1
            continue
        if i >= warmup:
            times.append(dt)
        time.sleep(0.05)

    if not times:
        print(f'  전부 실패 ({fails}회)')
        return None
    times.sort()
    def p(q):
        return times[min(len(times) - 1, int(len(times) * q))]
    print(f'  n={len(times)} 실패={fails}   최소 {times[0]:6.1f}  '
          f'중앙 {statistics.median(times):6.1f}  평균 {statistics.mean(times):6.1f} ms')
    print(f'  p90 {p(0.90):6.1f}   p99 {p(0.99):6.1f}   최대 {times[-1]:6.1f} ms')
    return times


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=20, help='측정 횟수')
    ap.add_argument('--target', choices=['self', 'robot'], default='self',
                    help="self=이 도구가 띄운 서버(기본, 로봇 안 건드림) / "
                         "robot=실제 robot_controller (dry_run 확인 후에만)")
    args = ap.parse_args()

    rclpy.init()
    node = Node('action_latency_probe')
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    srv_node = None
    if args.target == 'self':
        name = PROBE_ACTION
        srv_node = Node('latency_probe')
        ActionServer(srv_node, AdjustPeg, PROBE_ACTION,
                     lambda gh: (gh.succeed(), AdjustPeg.Result(success=True))[1])
        ex.add_node(srv_node)
        # 서버가 뜨는 동안 잠깐 돌려 준다
        t = threading.Thread(target=lambda: [ex.spin_once(timeout_sec=0.1)
                                             for _ in range(20)], daemon=True)
        t.start()
        t.join()
        print('대상: 이 도구가 띄운 Action 서버 (로봇을 건드리지 않는다)')
    else:
        name = REAL_ACTION
        print(f'대상: {REAL_ACTION} — 먼저 {CONTROLLER_NODE} 의 dry_run 을 확인한다')
        ok, why = check_dry_run(node, ex)
        print(f'  {why}')
        if not ok:
            print('\n중단했다. 이 측정은 실제 페그 조절 goal 을 보낸다.\n'
                  '재려면 robot_controller 를 dry_run 기본값(true)으로 띄워라:\n'
                  '    ros2 run tunio_robot robot_controller')
            node.destroy_node()
            if srv_node:
                srv_node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    print('\nAction 왕복 (goal 전송 → result 수신)')
    measure(node, ex, name, args.n)

    node.destroy_node()
    if srv_node:
        srv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
