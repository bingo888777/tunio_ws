"""
task_manager — 작업 순서, 센트 판정, 재시도, 최종 검증, 상태 발행.
기획서 4·5절의 흐름을 상태기계로 구현할 자리.

TODO(정동휘):
  - Phase 1 조율 루프: 줄 선택 -> 측정 Action -> play_string -> 판정 -> adjust_peg
  - 측정 재시도(max_measure_retry) / 페그 조절(max_adjust_count) 카운터 분리 관리
  - Phase 2 최종 검증 패스: 조절 없이 4줄 재측정, 틀어진 줄 FAIL_DETUNED_ON_VERIFY
  - pause/resume/cancel 처리, /tuning/state 주기 발행
  - 정상 종료 시 result_store에 SaveJob 요청
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

from tunio_interfaces.action import MeasurePitch, PlayString, AdjustPeg
from tunio_interfaces.msg import TuningSettings, TuningState
from tunio_interfaces.srv import CommandTuning, GetSettings, SetSettings

from tunio_server import tuning_logic as L

# 명세 5절: /tuning/state 는 reliable, depth 1, transient local.
# 늦게 켠 UI도 마지막 상태를 즉시 받는다. hmi_node 쪽 값과 같아야 매칭된다.
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        # 설정 (명세 5절 TuningSettings. 작업 시작 전에만 변경)
        self.settings = TuningSettings()
        self.settings.tolerance_cents = 5.0      # 임시값 (명세 권장 시작값 10.0)
        self.settings.max_adjust_count = 5
        self.settings.max_measure_retry = 3
        self.settings.measure_timeout_sec = 10.0
        self.settings.onset_timeout_sec = 10.0

        # Action 클라이언트
        self.cli_measure = ActionClient(self, MeasurePitch, "/audio/measure_pitch")
        self.cli_play = ActionClient(self, PlayString, "/robot/play_string")
        self.cli_adjust = ActionClient(self, AdjustPeg, "/robot/adjust_peg")

        # 상태 발행
        self.pub_state = self.create_publisher(TuningState, "/tuning/state", STATE_QOS)

        # 명령 서비스. 네 개가 CommandTuning 타입을 공유하며
        # 어느 이름으로 불렸는지가 곧 명령이다 (명세 6절)
        self.create_service(CommandTuning, "/tuning/start", self.on_start)
        self.create_service(CommandTuning, "/tuning/pause", self.on_pause)
        self.create_service(CommandTuning, "/tuning/resume", self.on_resume)
        self.create_service(CommandTuning, "/tuning/cancel", self.on_cancel)
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
        res.settings = self.settings
        return res

    def on_set_settings(self, req, res):
        # TODO: 작업 중이면 거부, 입력 범위 검증
        self.settings = req.settings
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
