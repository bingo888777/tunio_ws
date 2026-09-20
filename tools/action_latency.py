#!/usr/bin/env python3
"""
ROS 2 Action 왕복 지연 — goal 전송부터 result 수신까지.

서비스 왕복(ros_latency.py)보다 무거운 이유
  Action 은 goal 요청·수락, 결과 요청·응답에 상태 토픽까지 오간다.
  응답 시간 예산에 들어가는 것은 이쪽이다.

이 값의 쓰임
  로컬에서 잰 Action/서비스 비율을 원격 서비스 지연에 곱하면
  원격 Action 지연을 추정할 수 있다. 양쪽 노드를 다른 PC 에 띄우지
  않고도 예산을 계산하려는 것이다.
"""
import statistics
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from tunio_interfaces.action import AdjustPeg

N = int(sys.argv[1]) if len(sys.argv) > 1 else 20

rclpy.init()
node = Node("action_latency_probe")
ex = SingleThreadedExecutor()
ex.add_node(node)
cli = ActionClient(node, AdjustPeg, "/robot/adjust_peg")

if not cli.wait_for_server(timeout_sec=30.0):
    print("robot_controller 가 30초 안에 안 보인다")
    sys.exit(1)

times, fails = [], 0
for i in range(N + 3):
    g = AdjustPeg.Goal()
    g.string_number, g.direction, g.angle_deg = 6, -1, 5.0
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
    if i >= 3:
        times.append(dt)
    time.sleep(0.08)

times.sort()
if times:
    p = lambda q: times[min(len(times) - 1, int(len(times) * q))]
    print(f"Action 왕복 (goal 전송 → result 수신)")
    print(f"  n={len(times)} 실패={fails}   최소 {times[0]:6.1f}  "
          f"중앙 {statistics.median(times):6.1f}  평균 {statistics.mean(times):6.1f} ms")
    print(f"  p90 {p(0.90):6.1f}   p99 {p(0.99):6.1f}   최대 {times[-1]:6.1f} ms")
else:
    print(f"전부 실패 ({fails}회)")

node.destroy_node()
rclpy.shutdown()
