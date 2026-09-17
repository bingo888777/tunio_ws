"""
task_manager — 작업 순서, 센트 판정, 재시도, 최종 검증, 상태 발행.
기획서 4·5절의 흐름을 상태기계로 구현할 자리.

TODO(정동휘):
  - Phase 1 조율 루프: 줄 선택 -> 측정 Action -> play_string -> 판정 -> adjust_peg
  - 측정 재시도(최대 3) / 페그 조절(최대 5) 카운터 분리 관리
  - Phase 2 최종 검증 패스: 조절 없이 4줄 재측정, 틀어진 줄 status=3
  - pause/resume/cancel 처리, /tuning/state 주기 발행
  - Phase 3 이송(후순위): transfer_guitar Action
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from tunio_interfaces.action import MeasurePitch, PlayString, AdjustPeg
from tunio_interfaces.msg import TuningState
from tunio_interfaces.srv import StartTuning, ControlCommand, GetSettings, SetSettings

from tunio_server import tuning_logic as L


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        # 설정 (기획서 9.1, 시작 전에만 변경)
        self.tolerance_cents = 5.0     # 임시값
        self.max_adjust = 5
        self.measure_timeout = 10.0    # 타임아웃(정상 응답 목표 2초)

        # Action 클라이언트
        self.cli_measure = ActionClient(self, MeasurePitch, "/audio/measure_pitch")
        self.cli_play = ActionClient(self, PlayString, "/robot/play_string")
        self.cli_adjust = ActionClient(self, AdjustPeg, "/robot/adjust_peg")

        # 상태 발행
        self.pub_state = self.create_publisher(TuningState, "/tuning/state", 10)

        # 명령 서비스
        self.create_service(StartTuning, "/tuning/start", self.on_start)
        self.create_service(ControlCommand, "/tuning/pause", self.on_pause)
        self.create_service(ControlCommand, "/tuning/resume", self.on_resume)
        self.create_service(ControlCommand, "/tuning/cancel", self.on_cancel)
        self.create_service(GetSettings, "/tuning/get_settings", self.on_get_settings)
        self.create_service(SetSettings, "/tuning/set_settings", self.on_set_settings)

        self.get_logger().info("task_manager 시작. 줄 순서: %s" % L.STRING_ORDER)

    # --- 서비스 콜백 (뼈대) ---
    def on_start(self, req, res):
        res.accepted = True          # TODO: 준비 상태 확인
        res.message = "수락"
        return res

    def on_pause(self, req, res):
        res.accepted = True; res.message = "일시정지"; return res

    def on_resume(self, req, res):
        res.accepted = True; res.message = "재개"; return res

    def on_cancel(self, req, res):
        res.accepted = True; res.message = "취소"; return res

    def on_get_settings(self, req, res):
        res.tolerance_cents = self.tolerance_cents
        res.max_adjust = self.max_adjust
        res.measure_timeout_sec = self.measure_timeout
        return res

    def on_set_settings(self, req, res):
        # TODO: 작업 중이면 거부
        self.tolerance_cents = req.tolerance_cents
        self.max_adjust = req.max_adjust
        self.measure_timeout = req.measure_timeout_sec
        res.accepted = True; res.message = "설정 갱신"
        return res


def main():
    rclpy.init()
    node = TaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
