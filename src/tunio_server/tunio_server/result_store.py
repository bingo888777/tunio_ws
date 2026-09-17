"""
result_store — SQLite 저장/조회 서비스.
정상 종료 시 일괄 저장, 취소·부분 결과는 저장 안 함(기획서 9.2).
TODO(정동휘): GetHistory 서비스 구현, 저장 요청 수신 방식 확정.
"""
import rclpy
from rclpy.node import Node
from tunio_interfaces.srv import GetHistory
from tunio_server import db


class ResultStore(Node):
    def __init__(self):
        super().__init__("result_store")
        self.con = db.connect()
        self.create_service(GetHistory, "/tuning/get_history", self.on_history)
        self.get_logger().info("result_store 시작. DB=%s" % db.DB_PATH)

    def on_history(self, req, res):
        res.history_json = "[]"   # TODO
        return res


def main():
    rclpy.init()
    node = ResultStore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
