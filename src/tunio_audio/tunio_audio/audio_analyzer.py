"""
audio_analyzer — measure_pitch Action 서버.
요청 시에만 녹음/분석. 녹음 준비 완료를 Feedback으로 알림(기획서 4.3).

TODO(정정인):
  - Goal 수신 -> ready Feedback -> 녹음 -> pitch.estimate -> Result
  - job_id/string_number/measurement_id를 Result에 그대로 실어 요청과 대조
  - target_hz 기준 +-2반음으로 탐색 범위 제한(옥타브 오인 차단)
  - onset(무음 -> 소리) 검출: ready 발행 직후부터 입력을 감시한다.
    onset 시점이 timeout_sec 카운트의 기준이다. 서버·로봇 신호에 의존하지 않는다
  - onset_timeout_sec 안에 onset이 없으면 FAIL_LOW_SIGNAL. 발현 자체가 실패한 경우다
  - timeout_sec 초과 시 FAIL_TIMEOUT 반환. 녹음 길이가 아니라 응답 제한 시간
  - 실패 사유: FAIL_LOW_SIGNAL / FAIL_NO_PITCH / FAIL_TIMEOUT / FAIL_CANCELED
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from tunio_interfaces.action import MeasurePitch
from tunio_audio import pitch, recorder   # noqa: F401


class AudioAnalyzer(Node):
    def __init__(self):
        super().__init__("audio_analyzer")
        self.srv = ActionServer(
            self, MeasurePitch, "/audio/measure_pitch", self.on_measure)
        self.get_logger().info("audio_analyzer 시작")

    def on_measure(self, goal):
        req = goal.request
        fb = MeasurePitch.Feedback(); fb.ready = True
        goal.publish_feedback(fb)
        # TODO: 녹음 -> pitch.estimate
        goal.succeed()
        res = MeasurePitch.Result()
        # ID 3개는 Goal 값을 그대로 되돌려준다.
        # 늦게 도착한 이전 결과를 걸러내는 유일한 수단이라 생략 불가(명세 4절)
        res.job_id = req.job_id
        res.string_number = req.string_number
        res.measurement_id = req.measurement_id
        res.success = False
        # success=False면 frequency_hz는 읽지 않는다. 0을 유효값으로 쓰지 않는다
        res.failure_reason = MeasurePitch.Result.FAIL_NO_PITCH   # TODO: 미구현
        return res


def main():
    rclpy.init()
    node = AudioAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
