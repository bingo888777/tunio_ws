"""로봇 미등록 실패가 ROS 상태와 웹 JSON까지 전달되고 재등록 뒤 복구되는지 확인."""
import threading
from unittest.mock import Mock

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from tunio_interfaces.msg import TuningState
from tunio_interfaces.srv import CommandTuning
from tunio_server.fake_hardware import FakeHardware
from tunio_server.task_manager import Aborted, Cancelled, TaskManager
from tunio_server.web_node import _state_to_dict


def test_missing_registration_error_reaches_web_and_recovers():
    rclpy.init()
    fake = FakeHardware()
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=4)
    thread = None
    try:
        fake.set_parameters([Parameter('require_peg_registration', value=True),
                             Parameter('prepare_delay_sec', value=0.0),
                             Parameter('ready_delay_sec', value=0.01),
                             Parameter('measure_delay_sec', value=0.02),
                             Parameter('play_delay_sec', value=0.01)])
        node.pub_state = Mock()
        node._tune_string = lambda no: node._measure_once(no)
        node._verify_pass = Mock()
        node._save = Mock()
        executor.add_node(fake)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        assert node._servers_not_ready() == []
        node.job_id = 'unregistered'
        node.active_settings = node.settings
        node._run_job()
        state = node.pub_state.publish.call_args.args[0]
        assert state.phase == TuningState.PHASE_ERROR
        assert '6번 페그 등록값 없음' in _state_to_dict(state)['error_message']
        assert '페그 재등록을 먼저 실행하세요' in state.error_message
        assert fake._prepared_pluck is None
        node._save.assert_not_called()
        # 실제 시작 서비스의 초기화 경로로 오류를 지운 후 별도 재등록한다.
        response = node.on_reregister_pegs(None, CommandTuning.Response())
        assert response.accepted
        node._worker.join(5)
        assert not node._worker.is_alive()
        assert node.phase == TuningState.PHASE_IDLE
        assert node.error_message == ''
        assert fake._registered_strings == {6, 5, 2, 1}
        node._save.assert_not_called()
        measure = node._tune_string
        for outcome in (None, Aborted('시험 오류'), Cancelled()):
            if not node._registration_ready:
                response = node.on_reregister_pegs(None, CommandTuning.Response())
                assert response.accepted
                node._worker.join(5)
                assert node._registration_ready

            def tune(no):
                if outcome is not None:
                    raise outcome
                measure(no)
            node._tune_string = tune
            response = node.on_start(None, CommandTuning.Response())
            assert response.accepted
            assert not node._registration_ready
            node._worker.join(5)
            assert not node._worker.is_alive()
            expected = (TuningState.PHASE_DONE if outcome is None else
                        TuningState.PHASE_CANCELED if isinstance(outcome, Cancelled)
                        else TuningState.PHASE_ERROR)
            assert node.phase == expected
            response = node.on_start(None, CommandTuning.Response())
            assert not response.accepted
            assert '재등록을 먼저' in response.message
            # 로봇 대역에 등록값이 남아 있어도 관제는 다음 조율을 허용하지 않는다.
            assert fake._registered_strings == {6, 5, 2, 1}
    finally:
        executor.shutdown()
        if thread:
            thread.join(3)
        node.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()
