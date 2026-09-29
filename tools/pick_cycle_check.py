#!/usr/bin/env python3
"""sol 지정 피크 집기/반환 실기 점검. 기본 계획만 출력, --execute는 단계별 확인 후 이동."""
import argparse
import copy
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/tunio_robot'))
from tunio_robot import motion_primitives as MP, runlog as RL, teach_points as TP  # noqa: E402


def run_cycle(points, sol, confirm=input, record=None):
    from DR_common2 import posx
    record = record if record is not None else {}
    holder = copy.copy(points.pick_holder())
    holder.sol = sol  # 대칭 목표 선택도 현재 sol 대신 시험할 sol로 계산한다.
    motion = dict(points.motion, speed_scale=1.0, vel=5.0, acc=10.0)
    holder, _ = MP._j6_pose(holder, motion)
    contact, above = MP._pick_points(holder, motion)
    retreat = list(above)
    floor, margin = MP._peg_transfer_floor(motion)
    retreat[2] = max(retreat[2], floor + 2 * margin)
    api = MP._require_api()
    current = MP._read_peg_joints(motion)
    targets = {}
    for label, task in [('above', above), ('contact', contact), ('retreat', retreat)]:
        joints = list(api.ikin(posx(task), sol))
        # 유한성·6관절은 J1 검사 전에 확인한다.
        if (len(joints) != 6 or not all(math.isfinite(v) for v in joints)
                or all(abs(v) < 1e-9 for v in joints)):
            raise MP.MotionError(f'{label}: IK 실패')
        MP._check_peg_transfer_j1(current, joints)
        MP._check_peg_turn(joints, 0, motion)
        targets[label] = joints
    route = points.route()
    flip, _ = MP.j1_flip(current[0], targets['above'][0], motion)
    via = None
    if flip:
        if route is None:
            raise MP.MotionError('J1 전환에 필요한 경유점 없음')
        via = route.resolve(current)
        MP._check_peg_transfer_j1(current, via)
        MP._check_peg_transfer_j1(via, targets['above'])
        MP._check_peg_turn(via, 0, motion)
    record.update(sol=sol, contact=contact, above=above, retreat=retreat,
                  ik=targets, via=via, steps=[])
    print(f'sol {sol}, 거치대 {contact[:3]}, 위쪽 {above[:3]}, 반환 후 {retreat[:3]}')
    print(f"접근 목표 관절: {targets['above']}; 경유: {via}")

    def step(label, action):
        if confirm(f'{label} — Enter=실행, 그 외 입력=중단: ').strip():
            raise RuntimeError('사용자 중단 — 자동 반환/개방하지 않음')
        action()
        actual = MP._read_peg_joints(motion)
        record['steps'].append({'step': label, 'joints': actual})
        print(f'완료: {label}, 관절={actual}')

    def move_above():
        MP._movej(targets['above'], motion)
        if api.get_current_solution_space() != sol:
            raise MP.MotionError('요청 sol 미도달 — 하강 금지')
        MP._check_pick_return_position(above, motion, '거치대 위')

    def vertical(task, label):
        MP._movel_base(task, 10.0, 20.0)
        MP._check_pick_return_position(task, motion, label)

    def grip():
        MP._grip(getattr(holder, 'grip_preset', None) or motion.get('pick_preset', 'close'))
        time.sleep(float(motion.get('pick_settle_sec', 1.0)))

    if via is not None:
        step('J1 전환 경유', lambda: MP._movej(via, motion))
    step('거치대 위 접근', move_above)
    step('그리퍼 열기 (빈손 확인)', MP._release)
    step('집기 위치로 천천히 하강', lambda: vertical(contact, '집기 위치'))
    step('피크 집기', grip)
    step('피크 든 채 수직 상승', lambda: vertical(above, '집기 후 상승'))
    step('반환 위치로 천천히 하강', lambda: vertical(contact, '반환 위치'))
    step('피크 놓기', MP._release)
    step('빈손 수직 상승', lambda: vertical(retreat, '반환 후 상승'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sol', type=int, choices=range(8), default=7)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--yaml', default=str(Path(__file__).resolve().parents[1]
                        / 'src/tunio_robot/config/teach_points.yaml'))
    args = parser.parse_args()
    print(f'sol {args.sol}: 접근 → 하강·집기 → 상승 → 하강·반환 → 빈손 상승')
    print('발현·페그 조절·자동 J6 회복 없이 한 번만 실행합니다.')
    if not args.execute:
        print('계획만 출력했습니다. 실제 단계별 시험은 --execute를 붙이세요.')
        return 0
    import rclpy
    rclpy.init()
    node = rclpy.create_node('tunio_pick_cycle_check')
    report = {'tool': 'pick_cycle_check', 'success': False}
    try:
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=.1)
        if 'robot_controller' in node.get_node_names():
            raise RuntimeError('robot_controller를 종료하고 단독 실행하세요')
        if input('피크는 거치대에 있고 빈손이며 다른 이동 도구를 종료했다면 START 입력: ').strip() != 'START':
            raise RuntimeError('시작 취소')
        points = TP.load(args.yaml)
        MP.connect(points.robot, points.gripper, logger=node.get_logger())
        run_cycle(points, args.sol, record=report)
        report['success'] = True
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Ctrl+C'
        print(f"중단: {report['error']}. 자동 개방/복귀하지 않습니다. Ctrl+C는 로봇 정지 명령이 아닙니다.")
        return 1
    finally:
        print('기록:', RL.save('pick_cycle_check', report, str(Path.home() / '.tunio/robot_data')))
        node.destroy_node()
        if MP._node is not None:
            MP._node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
