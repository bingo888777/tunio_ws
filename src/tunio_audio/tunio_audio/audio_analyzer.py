"""
audio_analyzer — measure_pitch Action 서버.
요청 시에만 녹음/분석. 녹음 준비 완료를 Feedback으로 알림(기획서 4.3).

TODO(정정인):
  - Goal 수신 -> ready Feedback -> 녹음 -> pitch.estimate -> Result
  - job_id/string_no/measure_id를 Result에 그대로 실어 요청과 대조
  - 실패 사유: 소리부족 / 판별불가 / 시간초과
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
        res.job_id = req.job_id
        res.string_no = req.string_no
        res.measure_id = req.measure_id
        res.success = False
        res.fail_reason = "미구현"
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
