#!/usr/bin/env python3
"""실제 M0609 단계별 이동 시험. 기본은 안내만, --check-only 연결 진단, --execute 실물 이동."""
import argparse
import json
import math
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/tunio_robot'))
from tunio_robot import motion_primitives as MP, runlog as RL, teach_points as TP  # noqa: E402
from pick_to_peg_check import confirm  # noqa: E402


def run(points, scene, peg, motion, *, execute=False, high_only=False, ask=confirm):
    """실제 현재값·공식 IK를 사용하는 공통 경로. 그리퍼/파지/회전 명령은 보내지 않는다."""
    MP.ensure_tcp()
    if scene == 'transition':
        if MP._peg_route_required(MP._current_j1(), 'right', motion):
            raise MP.MotionError('전환 시험은 오른쪽 작업 자세(J1 -60~80도)에서 시작하세요. '
                                 '출발점으로 자동 이동하거나 역방향으로 돌아가지 않습니다')
        pose, _ = MP._j6_pose(points.peg(peg), motion)
        return MP._pick_to_peg_approach(
            pose, TP.side_of(peg), points.route(), motion,
            confirm=ask, execute=execute, high_only=high_only)
    pose = points.pick_holder() if scene == 'holder' else points.pluck(peg)
    pose, _ = MP._j6_pose(pose, motion)
    floor, margin = MP._peg_transfer_floor(motion)
    offset = motion.get('pick_up_mm', 50) if scene == 'holder' else motion.get('pluck_up_mm', 20)
    target = list(pose.task)
    target[2] = max(target[2] + float(offset), floor + 2 * margin)
    return MP._transfer_above_floor(target, motion, confirm=ask, execute=execute)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene', choices=['transition', 'holder', 'pluck'], default='transition')
    ap.add_argument('--peg', type=int, choices=[6, 5, 2, 1], default=6)
    ap.add_argument('--high-only', action='store_true', help='전환을 높은 접근점까지만 실행')
    ap.add_argument('--yaml', default=str(ROOT/'src/tunio_robot/config/teach_points.yaml'))
    ap.add_argument('--speed-scale', type=float, default=1.0,
                    help='YAML 값 대신 쓸 시험 이동 배율. 기본 1.0, 최대 1')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--execute', action='store_true', help='실물 이동, 각 구간 y 확인')
    mode.add_argument('--check-only', action='store_true', help='실제 현재값/IK 조회. 이동 없음')
    args = ap.parse_args(argv)
    if not math.isfinite(args.speed_scale) or not 0 < args.speed_scale <= 1:
        ap.error('시험 speed-scale은 0 초과 1 이하여야 합니다')
    if args.scene == 'transition' and args.peg not in (6, 5):
        ap.error('새 전환 실기 시험은 오른쪽→왼쪽 6·5번만 지원합니다')
    if args.high_only and args.scene != 'transition':
        ap.error('--high-only는 transition에만 사용합니다')
    points = TP.load(args.yaml)
    motion = dict(points.motion, speed_scale=args.speed_scale)
    print(f'실기 도구: {args.scene}, {args.peg}번, speed_scale={args.speed_scale:g}')
    print('현재 자세에서 시작. 그리퍼 열기/집기/발현/페그 회전/자동 복귀 없음.')
    print('6·5번 마지막 하강은 RG2/TCP가 설정 박스와 겹칩니다. 사용자 확인에 따라 유지합니다.')
    print('Ctrl+C는 후속 명령 차단 및 Soft Stop 요청입니다. 즉시 정지는 펜던트 비상정지를 사용하세요.')
    if not (args.execute or args.check_only):
        print('로봇에 연결하지 않았습니다. 조회는 --check-only, 이동은 --execute.')
        return 0

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from dsr_msgs2.srv import CheckMotion, MoveStop
    from tunio_robot.motion_cancel import CancelSession, request_and_confirm_stop
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('work_motion_check')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = None
    session = CancelSession()
    log = dict(tool='work_motion_check', scene=args.scene, peg=args.peg,
               execute=args.execute, high_only=args.high_only, motion=motion, success=False, stages=[])
    path = RL.session_path('work_motion_check', str(Path.home()/'.tunio/robot_data'))
    moving = False
    try:
        for _ in range(20):
            rclpy.spin_once(node, executor=executor, timeout_sec=.1)
        busy = set(node.get_node_names()) & {'robot_controller', 'task_manager', 'single_string_test'}
        if busy:
            raise RuntimeError(f'통합 노드를 먼저 종료하세요: {sorted(busy)}')
        prefix = f"/{points.robot['id']}/dsr_controller2/motion"
        stop = node.create_client(MoveStop, prefix+'/move_stop')
        status = node.create_client(CheckMotion, prefix+'/check_motion')
        if args.execute and not (stop.wait_for_service(timeout_sec=3)
                                 and status.wait_for_service(timeout_sec=3)):
            raise RuntimeError('정지/상태 서비스를 확인하지 못했습니다')
        confirm('두산 브링업만 유지하고 다른 조작 도구를 종료했는지 확인. '\
                '실기 연결(TCP/툴 설정 및 모드 전환 포함)')
        MP.connect(points.robot, points.gripper, logger=node.get_logger())
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        diagnostic = run(points, args.scene, args.peg, motion, high_only=args.high_only)
        log['plan'] = diagnostic
        print(json.dumps(diagnostic, ensure_ascii=False, indent=2))
        RL.write(path, log)
        if args.execute:
            confirm('빈손·그리퍼 open 38mm, 기타/거치대와 비접촉 상태인지 확인. 실제 이동 시작')
            MP._cancel_check = session.check
            moving = True

            def step(message):
                q = MP._read_peg_joints(motion)
                tcp, sol = MP.read_posx()
                log['stages'].append(dict(next=message, joints=q, tcp=list(tcp), sol=int(sol)))
                RL.write(path, log)
                print(f'현재 관절: {q}\n현재 TCP: {list(tcp)}')
                confirm(message)
            run(points, args.scene, args.peg, motion, execute=True,
                high_only=args.high_only, ask=step)
            log['final_joints'] = MP._read_peg_joints(motion)
            log['final_tcp'] = list(MP.read_posx()[0])
        log['success'] = True
        print('완료. 현재 위치에서 종료합니다.' if args.execute else '조회 완료. 이동 명령 없음.')
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        log['error'] = str(exc) or 'KeyboardInterrupt'
        if moving:
            session.requested.set()
            session.work_done.set()
            request_and_confirm_stop(stop, status, session, 5.0, .1)
            log['stop_error'] = session.error
            print(f'정지 확인 실패: {session.error}' if session.error else 'Soft Stop·IDLE 확인 완료')
        print(f"중단: {log['error']}. 자동 복귀 없음.", file=sys.stderr)
        return 1
    finally:
        MP._cancel_check = None
        print(f'기록: {RL.write(path, log) or "저장 실패"}')
        executor.shutdown(timeout_sec=2)
        if thread:
            thread.join(timeout=2)
        if MP._node is not None:
            MP._node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
