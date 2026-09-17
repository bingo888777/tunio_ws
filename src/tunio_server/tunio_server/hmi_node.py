"""
hmi_node — PyQt UI와 ROS2를 잇는다. 버튼 -> Service, /tuning/state -> 화면.
탭 3개: 실시간 작업 / 작업 이력 / 설정 (기획서 9.1)
TODO(정동휘): PyQt 이벤트 루프와 rclpy spin을 QTimer로 통합.
"""
import rclpy
from rclpy.node import Node
from tunio_interfaces.msg import TuningState


class HmiNode(Node):
    def __init__(self):
        super().__init__("hmi_node")
        self.create_subscription(TuningState, "/tuning/state", self.on_state, 10)
        self.get_logger().info("hmi_node 시작")

    def on_state(self, msg):
        pass   # TODO: UI 갱신


def main():
    rclpy.init()
    node = HmiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
