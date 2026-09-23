"""실제 ROS Action으로 피크 정책을 검증한다. 이동·폭·오디오는 대역이다."""
import threading
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tunio_interfaces.action import MeasurePitch
from tunio_robot import grip_width as GW
from tunio_robot import motion_primitives as MP
from tunio_robot.robot_controller import RobotController
from tunio_server.task_manager import Aborted, TaskManager


@pytest.mark.parametrize('stage', ['pickup', 'post_pluck', 'resume'])
@pytest.mark.parametrize('outcome', ['recovered', 'exhausted', 'unreadable'])
def test_pick_policy_across_real_actions(stage, outcome, monkeypatch):
    # 생성자는 dry_run으로 시작한다. 실물 연결 함수는 테스트 내내 금지한다.
    monkeypatch.setattr(MP, 'connect', Mock(side_effect=AssertionError('실물 연결 금지')))
    rclpy.init()
    robot = RobotController()
    manager = TaskManager()
    audio = rclpy.create_node('pick_test_audio')
    executor = MultiThreadedExecutor(num_threads=4)
    events = []

    def measure(goal):
        goal.publish_feedback(MeasurePitch.Feedback(ready=True))
        req = goal.request
        goal.succeed()
        return MeasurePitch.Result(success=True, job_id=req.job_id,
                                   measurement_id=req.measurement_id,
                                   string_number=req.string_number, frequency_hz=82.4)

    server = ActionServer(audio, MeasurePitch, '/audio/measure_pitch', measure,
                          callback_group=ReentrantCallbackGroup())
    robot.dry_run = False
    robot._holding_pick = stage != 'pickup'
    robot._from_pick = stage == 'pickup'  # 반환 후 재집기도 상태를 갱신해야 한다
    readings = {'recovered': [13.0, 13.0, 13.7],
                'exhausted': [13.0, 13.0, 13.0],
                'unreadable': [GW.WidthError('실패'), GW.WidthError('재읽기 실패')]}[outcome]
    reader = Mock(side_effect=readings + [13.7])
    robot._read_width = reader

    def pick(*args, **kwargs):
        events.append('pick')
        kwargs['verify_pick']()

    def pluck(*args, **kwargs):
        if kwargs.get('prepare_only'):
            events.append('prepare')
            return
        events.append('pluck')
        kwargs['verify_pick']()

    monkeypatch.setattr(MP, 'pick_up', pick)
    monkeypatch.setattr(MP, 'pluck_string', pluck)
    monkeypatch.setattr(MP, 'retreat_pick', lambda *a: events.append('retreat'))
    monkeypatch.setattr(MP, 'pick_return', lambda *a, **kw: events.append('return'))
    monkeypatch.setattr(MP, 'return_home', lambda *a: events.append('home'))
    manager.job_id = 'pick-integration'
    manager.active_settings = manager.settings
    manager._registered_pegs = {6, 5, 2, 1}
    if stage == 'resume':
        manager._resumed_since_play.set()
    for node in (robot, manager, audio):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert manager.cli_play.wait_for_server(timeout_sec=5)
        assert manager.cli_measure.wait_for_server(timeout_sec=5)
        if outcome == 'exhausted':
            with pytest.raises(Aborted, match='홈 복귀 완료'):
                manager._measure_once(6)
            assert events[-1] == 'home'
            assert events.count('home') == 1
            assert '피크' in manager.robot_notice
            assert robot._pick_fault
        else:
            assert manager._measure_once(6).ok
            assert events.count('home') == 0
            assert robot._holding_pick is True and robot._pick_fault is None
            assert robot._from_pick is False
            if outcome == 'unreadable':
                assert '피크 폭 확인 불가' in manager.robot_notice
            else:
                assert manager.robot_notice == ''
        assert events.count('return') == int(stage != 'pickup' and outcome != 'unreadable')
        assert events.count('pluck') == (0 if outcome == 'exhausted'
                                         and stage != 'post_pluck' else 1)
        assert not manager._active_goals
        assert not manager._resumed_since_play.is_set()
    finally:
        executor.shutdown()
        thread.join(3)
        server.destroy()
        for node in (robot, manager, audio):
            node.destroy_node()
        rclpy.shutdown()
