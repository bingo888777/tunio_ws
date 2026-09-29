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
    """공통 실기 경로. pick-pluck만 실제 집기와 1회 발현을 수행한다."""
    MP.ensure_tcp()
    if scene == 'pick-pluck':
        holder, pluck = points.pick_holder(), points.pluck(peg)
        if not execute:
            return {'scene': scene, 'holder_approach': run(
                points, 'holder', peg, motion, execute=False),
                'sequence': ['피크 집기', '홈 없이 발현 상단으로 이동', '1회 발현 후 상승'],
                'note': '조회는 거치대 상단 경로까지. 집기 후 실제 자세로 발현 경로를 계획합니다.'}
        ask('피크 집기: 거치대 상단 이동 → 그리퍼 열기 → 하강 → 닫기 → 상승')
        MP.pick_up(holder, motion, height_transfer=True)
        ask('피크가 들린 상태를 확인. 고정 홈 없이 발현 상단으로 이동')
        MP.pluck_string(pluck, motion, height_transfer=True, prepare_only=True)
        ask(f'{peg}번 줄 1회 발현: 하강 → 스트로크·즉시 후퇴 → 발현 상단으로 상승')
        MP.pluck_string(pluck, motion, height_transfer=True, prepared=True)
        return {'pick_state': 'held_assumed', 'plucks_commanded': 1,
                'home_commanded': False, 'pick_returned': False}
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
    ap.add_argument('--scene', choices=['transition', 'holder', 'pluck', 'pick-pluck'], default='transition')
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
    if args.scene == 'pick-pluck':
        print('실제 피크 집기 → 홈 없이 발현 준비 → 1회 발현. 피크를 든 채 상단에서 종료합니다.')
        print('집기 성공 자동 판정 없음. 놓치면 되찾을 수 없으므로 단계 확인에서 중단하세요.')
        print('시작은 빈손, 피크는 거치대에 있어야 합니다. 자동 반환·홈 복귀·음정 측정 없음.')
    else:
        print('현재 자세에서 시작. 그리퍼 열기/집기/발현/페그 회전/자동 복귀 없음.')
    print('기존 낮은 접근·이탈 구간은 RG2/TCP가 설정 박스와 겹칩니다. 실시간 충돌 검사는 없습니다.')
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
            result = run(points, args.scene, args.peg, motion, execute=True,
                         high_only=args.high_only, ask=step)
            if result is not None:
                log['result'] = result
            log['final_joints'] = MP._read_peg_joints(motion)
            log['final_tcp'] = list(MP.read_posx()[0])
        log['success'] = True
        if args.execute and args.scene == 'pick-pluck':
            print('1회 발현 명령 완료. 피크를 든 것으로 가정하고 발현 상단에서 종료합니다. 자동 반환 없음.')
        else:
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
