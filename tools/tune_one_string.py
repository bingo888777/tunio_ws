#!/usr/bin/env python3
"""한 줄 전체 조율을 실제 task_manager 로직과 ROS Action으로 시험한다.

일반 task_manager를 종료하고 실행한다. 기본은 계획만 출력하고,
--execute를 주면 연결된 robot_controller/audio_analyzer에 실제 요청한다.
로봇의 dry_run=false이면 실제로 움직인다. 조율 알고리즘·재시도·각도 상한은
서버 코드를 그대로 사용한다. --wipe 없이는 넥 닦기(WipeStrings)를 호출하지 않는다.
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import threading

# 테스트와 설치본의 조율 로직이 갈리지 않도록 이 체크아웃의 서버 코드를 쓴다.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'tunio_server'))


def run_test(node, no, do_wipe=False):
    """유효 등록은 로봇에서 재사용한다. 실패한 측정도 기존 정책대로 재시도한다.

    do_wipe — 검증 뒤 넥 닦기(WipeStrings)도 실행한다. 메인 시나리오(_run_job)와
    같은 순서(검증 → 닦기 → 복귀)와 같은 일시정지 재시도 방식을 쓴다 — 4줄을
    다 안 돌아도 실제 통합 경로(Action·Paused 처리 포함)로 닦기만 빠르게
    반복 시험하려는 용도다(2026-09-29). wipe_check.py 는 task_manager 를
    거치지 않아 이 경로와 다르다.
    """
    from tunio_server.task_manager import Paused

    node._register_pegs([no])
    node._tune_string(no)
    node._verify_pass([no])
    if do_wipe:
        while True:
            try:
                node._wipe()
                break
            except Paused as p:
                node._hold_after_pause(0, p.what)
    node._return_home()
    return node.results[no]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--string', type=int, choices=[6, 5, 2, 1], default=1)
    ap.add_argument('--wipe', action='store_true',
                    help='검증 뒤 넥 닦기(WipeStrings)도 실행 — 메인 시나리오 순서 재현, 4줄 전체 없이 빠른 반복 시험용')
    ap.add_argument('--execute', action='store_true', help='실제 ROS Action 요청 허용')
    ap.add_argument('--calibration', type=Path, default=Path(__file__).resolve().parents[1]
                    / 'src/tunio_server/config/peg_calibration.yaml')
    ap.add_argument('--onset-timeout', type=float, default=40.0,
                    help='피크 집기 포함 소리 대기 초(시험용 임시값, 기본 40)')
    ap.add_argument('--output', type=Path, default=None, help='시험 결과 JSON 경로')
    args = ap.parse_args()
    if not 0 < args.onset_timeout < float('inf'):
        ap.error('--onset-timeout은 유한한 양수여야 한다')
    wipe_leg = ' → 넥 닦기' if args.wipe else ''
    print(f'{args.string}번: 미등록 시 등록 → 발현/측정 → 필요시 조절/재측정 '
          f'→ 최종 검증{wipe_leg} → 피크 반환/대기 복귀')
    print('이미 목표 음정이면 페그를 돌리지 않는다. 4줄 DB 저장은 하지 않는다.')
    if not args.execute:
        print('계획만 확인했습니다. 실행하려면 --execute를 추가하세요.')
        return 0

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from tunio_interfaces.msg import StringResult, TuningSettings, TuningState
    from tunio_interfaces.srv import CommandTuning
    from tunio_server.task_manager import TaskManager

    # 일반 서버와 명령/상태 이름을 공유하지 않는다. 로봇·오디오는 같은 Action을 쓴다.
    ros_args = ['--ros-args', '-r', '__node:=single_string_test',
                '-p', f'peg_calibration_file:={args.calibration.resolve()}']
    for name in ('start', 'pause', 'resume', 'cancel', 'get_settings', 'set_settings', 'state'):
        ros_args += ['-r', f'/tuning/{name}:=/single_string_test/{name}']
    rclpy.init(args=ros_args)
    node = None
    ex = None
    thread = None
    stamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
    output = args.output or Path.home() / '.tunio/robot_data' / f'{stamp}_single_string.json'
    report = {'string': args.string, 'started_at': stamp, 'success': False,
              'calibration': str(args.calibration.resolve()),
              'onset_timeout_sec': args.onset_timeout}
    code = 1
    try:
        # 글로벌 이름 remap을 적용하지 않는 노드로 기존 제어 서버를 확인한다.
        probe = Node('single_string_probe', use_global_arguments=False)
        try:
            for name in ('/tuning/start', '/single_string_test/start'):
                normal = probe.create_client(CommandTuning, name)
                if normal.wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(f'{name} 서버가 실행 중입니다. 종료 후 다시 실행하세요.')
                probe.destroy_client(normal)
        finally:
            probe.destroy_node()
        node = TaskManager()
        missing = node._servers_not_ready()
        if missing:
            raise RuntimeError('Action 서버 없음: ' + ', '.join(missing))
        # 조절이 필요해진 후에야 캘리브레이션 누락을 발견하지 않도록 미리 검증한다.
        node.calibration.get(args.string)
        node.job_id = 'single_' + stamp
        node.active_settings = TuningSettings()
        for field in node.settings.get_fields_and_field_types():
            setattr(node.active_settings, field, getattr(node.settings, field))
        node.active_settings.onset_timeout_sec = args.onset_timeout
        node._worker = threading.current_thread()  # 시험 중 추가 시작 요청 거부
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        thread = threading.Thread(target=ex.spin, daemon=True)
        thread.start()
        rec = run_test(node, args.string, do_wipe=args.wipe)
        # 검증 측정 실패인데 이전 성공을 유지하는 생산 정책과 별도로 시험은 엄격히 본다.
        report['result'] = {k: getattr(rec, k) for k in rec.get_fields_and_field_types()}
        report['success'] = (rec.result == StringResult.RESULT_SUCCESS and rec.has_verified_hz)
        node._set_phase(TuningState.PHASE_DONE, 0)
        code = 0 if report['success'] else 1
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        print(report['error'])
        print('중단 시 자동 회수하지 않습니다. 로봇은 계속 동작 중일 수 있으니 상태를 확인하세요.')
        if node is not None:
            node._set_phase(TuningState.PHASE_ERROR, 0)
    finally:
        if ex is not None:
            ex.shutdown()
        if thread is not None:
            thread.join()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(f'결과: {output} / {"성공" if report["success"] else "실패"}')
    return code


if __name__ == '__main__':
    sys.exit(main())
