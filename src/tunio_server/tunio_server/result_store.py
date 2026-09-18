"""
result_store — SQLite 저장/조회 서비스.
정상 종료 시 일괄 저장, 취소·부분 결과는 저장 안 함(기획서 9.2).
TODO(정동휘): GetHistory / SaveJob 구현, db.py 스키마를 새 인터페이스에 맞추기.
"""
import rclpy
from rclpy.node import Node
from tunio_interfaces.srv import GetHistory, SaveJob
from tunio_server import db


class ResultStore(Node):
    def __init__(self):
        super().__init__("result_store")
        self.con = db.connect()
        self.create_service(GetHistory, "/tuning/get_history", self.on_history)
        self.create_service(SaveJob, "/tuning/save_job", self.on_save_job)
        self.get_logger().info("result_store 시작. DB=%s" % db.DB_PATH)

    def on_history(self, req, res):
        # job_id가 비면 최신순 목록(jobs), 채우면 그 작업의 줄별 상세(strings/settings)
        res.found = False   # TODO
        return res

    def on_save_job(self, req, res):
        # 정상 종료 건만 들어온다. 취소·부분 결과는 task_manager가 보내지 않는다
        res.saved = False   # TODO
        res.message = "미구현"
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
