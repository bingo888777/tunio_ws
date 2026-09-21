#!/usr/bin/env python3
"""
ROS 2 서비스 왕복 지연 측정 — 3 PC 구성의 통신 비용을 잰다.

왜 필요한가
  측정 응답 목표가 2.2초인데 6번 줄 실측이 이미 2.05초다. 남은 여유가 0.15초뿐이라
  통신 지연이 그보다 크면 목표를 못 맞춘다. TODO 6절 '통신 지연 측정' 항목.

무엇을 재는가
  읽기 전용 서비스를 한 노드에서 반복 호출해 왕복 시간을 잰다. 노드 생성과
  디스커버리는 루프 밖에서 한 번만 하므로 순수 왕복만 남는다.
  (ros2 service call CLI 로는 못 잰다. 매번 노드를 새로 만들어 디스커버리가 섞인다)

한계
  Action 왕복은 이보다 크다. goal 요청·수락, 결과 요청·응답에 상태 토픽까지
  오가므로 대략 2~3배로 봐야 한다. 이 값은 하한이다.
"""
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node


def run(node, client, label, n, warmup=3):
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    req = client.srv_type.Request()
    times, fails = [], 0
    for i in range(n + warmup):
        t0 = time.perf_counter()
        fut = client.call_async(req)
        ex.spin_until_future_complete(fut, timeout_sec=5.0)
        dt = (time.perf_counter() - t0) * 1000.0
        if not fut.done():
            fails += 1
            continue
        if i >= warmup:          # 처음 몇 번은 캐시가 안 데워져 느리다
            times.append(dt)
        time.sleep(0.05)
    ex.remove_node(node)
    if not times:
        print(f"{label}: 전부 실패 ({fails}회)")
        return None
    times.sort()
    p = lambda q: times[min(len(times) - 1, int(len(times) * q))]
    print(f"{label}")
    print(f"  n={len(times)} 실패={fails}   "
          f"최소 {times[0]:6.1f}  중앙 {statistics.median(times):6.1f}  "
          f"평균 {statistics.mean(times):6.1f} ms")
    print(f"  p90 {p(0.90):6.1f}   p99 {p(0.99):6.1f}   최대 {times[-1]:6.1f} ms")
    return times


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=30, help="측정 횟수")
    ap.add_argument("--remote-node", default="dsr01",
                    help="원격 PC 의 노드 이름 (파라미터 서비스를 부른다)")
    ap.add_argument("--local-node", default=None,
                    help="같은 PC 의 노드 이름. 주면 기준선으로 같이 잰다")
    args = ap.parse_args()

    rclpy.init()
    node = Node("latency_probe")
    from rcl_interfaces.srv import GetParameters

    targets = [("원격 (다른 PC)", f"/{args.remote_node}/get_parameters")]
    if args.local_node:
        targets.append(("로컬 (같은 PC)", f"/{args.local_node}/get_parameters"))

    print("ROS 2 서비스 왕복 지연\n" + "=" * 46)
    for label, srv in targets:
        cli = node.create_client(GetParameters, srv)
        print(f"\n[{label}] {srv}")
        if not cli.wait_for_service(timeout_sec=30.0):
            print("  서비스가 30초 안에 안 보인다 — 건너뜀")
            continue
        run(node, cli, "  결과", args.n)
        node.destroy_client(cli)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
