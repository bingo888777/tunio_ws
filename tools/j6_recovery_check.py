#!/usr/bin/env python3
"""J6 회복 실기 점검. 기본은 joint_states 조회만, --execute에서 확인 후 1회 회복.

로봇 드라이버만 켜고 robot_controller/조그 등 다른 이동 도구는 종료한다.
접촉이 없는 회전 공간은 작업자가 확인한다. 한계까지 올리는 준비 이동은 하지 않는다.
"""
import argparse
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/tunio_robot'))
from tunio_robot import motion_primitives as MP, runlog as RL, teach_points as TP  # noqa: E402


def recover_once(motion, confirm=input):
    """운영 회복 함수를 그대로 호출. 확인 대기 중 관절이 바뀌면 실행하지 않는다."""
    if not motion.get('j6_recovery_enabled', False):
        raise MP.MotionError('YAML의 J6 회복이 꺼져 있다')
    before = MP._read_peg_joints(motion, for_recovery=True)
    threshold = MP._j6_threshold(motion)
    if abs(before[5]) < threshold:
        return {'status': 'not_needed', 'before': before, 'offset': 0.0}
    offset = -180.0 if before[5] > 0 else 180.0
    print(f'J6 {before[5]:.2f} → {before[5] + offset:.2f}도; J1~J5는 유지')
    print('페그·줄·거치대에서 떨어져 있고 손목/피크/케이블의 회전 공간이 있어야 합니다.')
    if confirm('다른 이동 프로그램을 종료했고 공간을 확인했다면 RECOVER 입력: ').strip() != 'RECOVER':
        return {'status': 'canceled', 'before': before}
    current = MP._read_peg_joints(motion, for_recovery=True)
    if max(abs(a - b) for a, b in zip(before, current)) > 1.0:
        raise MP.MotionError('확인 중 관절각이 바뀌었다. 실행하지 않고 다시 확인한다')
    offset = MP._recover_j6(motion)
    after = MP._read_peg_joints(motion)
    return {'status': 'recovered' if offset else 'not_needed',
            'before': before, 'after': after, 'offset': offset}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--yaml', default=str(Path(__file__).resolve().parents[1]
                    / 'src/tunio_robot/config/teach_points.yaml'))
    ap.add_argument('--execute', action='store_true', help='확인 후 운영 함수로 회복 1회 실행')
    ap.add_argument('--out-dir', default=str(Path.home() / '.tunio/robot_data'))
    args = ap.parse_args()
    points = TP.load(args.yaml)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from tunio_robot.robot_controller import RobotController

    rclpy.init()
    node = rclpy.create_node('tunio_j6_recovery_check')
    monitor = SimpleNamespace(points=points, _j6_pending=False, get_logger=node.get_logger)
    samples = []
    record = {'tool': 'j6_recovery_check', 'execute': args.execute,
              'motion': points.motion, 'status': 'failed', 'samples': samples}
    MP._j6_motion = points.motion
    MP._j6_sample = None

    def on_state(msg):
        RobotController._on_j6(monitor, msg)
        sample = MP._j6_sample
        if sample:
            samples.append({'time_monotonic': sample[0], 'j6_deg': sample[1]})

    node.create_subscription(JointState, f"/{points.robot['id']}/joint_states",
                             on_state, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + 5
        while MP._j6_sample is None and time.monotonic() < deadline:
            time.sleep(.05)
        if MP._j6_sample is None:
            raise MP.MotionError('joint_states 수신 없음')
        print(f'현재 J6={MP._j6_sample[1]:.2f}도 / 회복 기준 ±{MP._j6_threshold(points.motion):g}도')
        if not args.execute:
            record['status'] = 'read_only'
            print('조회만 완료. 회복 검증은 --execute로 실행합니다.')
        else:
            if 'robot_controller' in node.get_node_names():
                raise MP.MotionError('robot_controller를 종료한 뒤 도구를 단독 실행하세요')
            MP.connect(points.robot, points.gripper, logger=node.get_logger())
            record.update(recover_once(points.motion))
            print(record['status'])
            if record['status'] == 'not_needed':
                print('회복 기준 미만이므로 이동하지 않았습니다. 실기 회복 통과가 아닙니다.')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        record['error'] = str(exc) or '사용자 중단'
        print(f"중단: {record['error']}")
        print('도구 종료는 진행 중인 로봇 이동의 정지 명령이 아닙니다.')
        return 1
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        path = RL.save('j6_recovery_check', record, args.out_dir)
        print(f'기록: {path or "저장 실패"}')
        node.destroy_node()
        if MP._node is not None:
            MP._node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
