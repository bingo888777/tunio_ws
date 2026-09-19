"""
result_store — SQLite 저장/조회 서비스.

정상 종료 시 일괄 저장, 취소·부분 결과는 저장 안 함(기획서 10.2).
무엇을 저장할지는 task_manager 가 판단한다. 여기는 받은 것을 그대로 넣는다.

메시지 <-> DB 행 변환이 이 파일의 일이다. SQL 은 db.py 에 있다.

단일 스레드로 spin 한다. MultiThreadedExecutor + ReentrantCallbackGroup 조합에서
서비스 콜백이 중복 실행되는 것을 통합 테스트에서 확인했고, 여기는 동시 처리가
필요할 만큼 무겁지 않다. save_job 은 그래도 같은 job_id 재저장에 안전하게 짜 뒀다.
"""
import rclpy
from rclpy.node import Node

from tunio_interfaces.msg import JobSummary, StringResult, TuningSettings
from tunio_interfaces.srv import GetHistory, SaveJob

from tunio_server import db

# GetHistory.limit 이 0 으로 오면 쓰는 기본 개수. UI 가 값을 안 채웠을 때의 대비다.
DEFAULT_HISTORY_LIMIT = 50


def string_result_to_row(msg) -> dict:
    """
    메시지 하나를 string_results 행으로 바꾼다.

    has_*_hz 가 false 면 주파수를 NULL 로 넣는다. 0Hz 를 유효값으로 쓰지 않는다
    (CLAUDE.md 4절).
    """
    return {
        'string_number': int(msg.string_number),
        'target_hz': float(msg.target_hz),
        'tuned_hz': float(msg.tuned_hz) if msg.has_tuned_hz else None,
        'verified_hz': float(msg.verified_hz) if msg.has_verified_hz else None,
        'final_error_cents': float(msg.final_error_cents),
        'adjust_count': int(msg.adjust_count),
        'result': int(msg.result),
        'failure_reason': int(msg.failure_reason),
    }


def row_to_string_result(row: dict):
    """DB 행 -> StringResult. NULL 은 has_*_hz=false 로 되돌린다."""
    msg = StringResult()
    msg.string_number = int(row['string_number'])
    msg.target_hz = float(row['target_hz'] or 0.0)
    msg.has_tuned_hz = row['tuned_hz'] is not None
    msg.tuned_hz = float(row['tuned_hz']) if msg.has_tuned_hz else 0.0
    msg.has_verified_hz = row['verified_hz'] is not None
    msg.verified_hz = float(row['verified_hz']) if msg.has_verified_hz else 0.0
    msg.final_error_cents = float(row['final_error_cents'] or 0.0)
    msg.adjust_count = int(row['adjust_count'] or 0)
    msg.result = int(row['result'] or 0)
    msg.failure_reason = int(row['failure_reason'] or 0)
    return msg


def settings_to_row(msg) -> dict:
    return {f: getattr(msg, f) for f in db.SETTINGS_FIELDS}


def row_to_settings(row):
    msg = TuningSettings()
    if row is None:
        return msg
    msg.tolerance_cents = float(row['tolerance_cents'] or 0.0)
    msg.max_adjust_count = int(row['max_adjust_count'] or 0)
    msg.max_measure_retry = int(row['max_measure_retry'] or 0)
    msg.measure_timeout_sec = float(row['measure_timeout_sec'] or 0.0)
    msg.onset_timeout_sec = float(row['onset_timeout_sec'] or 0.0)
    return msg


def row_to_job_summary(row: dict):
    msg = JobSummary()
    msg.job_id = str(row['job_id'])
    sec, nanosec = db.text_to_time(row['finished_at'])
    msg.finished_at.sec = sec
    msg.finished_at.nanosec = nanosec
    msg.overall_result = int(row['overall_result'] or 0)
    return msg


class ResultStore(Node):

    def __init__(self):
        super().__init__('result_store')
        # 기본값은 ~/.tunio/tunio.db. 테스트나 여러 대를 한 PC 에서 돌릴 때 갈아끼운다
        self.declare_parameter('db_path', str(db.DB_PATH))
        path = self.get_parameter('db_path').value
        self.con = None
        self.db_error = ''
        try:
            self.con = db.connect(path)
            self.get_logger().info('result_store 시작. DB=%s' % path)
        except db.SchemaMismatch as exc:
            # 옛 스키마 파일이 남아 있으면 INSERT 가 조용히 실패한다.
            # 노드를 죽이는 대신 사유를 들고 살아 있게 해서, UI 가 이유를 보게 한다.
            self.db_error = str(exc)
            self.get_logger().error('DB 를 열지 못했다. 저장·조회가 모두 실패한다:')
            for line in self.db_error.splitlines():
                self.get_logger().error('  %s' % line)

        self.create_service(GetHistory, '/tuning/get_history', self.on_history)
        self.create_service(SaveJob, '/tuning/save_job', self.on_save_job)

    def on_history(self, req, res):
        # job_id가 비면 최신순 목록(jobs), 채우면 그 작업의 줄별 상세(strings/settings)
        res.found = False
        res.jobs = []
        res.strings = []
        res.settings = TuningSettings()
        if self.con is None:
            self.get_logger().error('이력 조회 실패: %s' % self.db_error)
            return res

        try:
            if not req.job_id:
                limit = int(req.limit) or DEFAULT_HISTORY_LIMIT
                rows = db.list_jobs(self.con, limit)
                res.jobs = [row_to_job_summary(r) for r in rows]
                res.found = bool(rows)
                return res

            job, strings, settings = db.get_job(self.con, req.job_id)
            if job is None:
                return res
            res.strings = [row_to_string_result(r) for r in strings]
            res.settings = row_to_settings(settings)
            res.found = True
            return res
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error('이력 조회 중 오류: %r' % exc)
            res.found = False
            return res

    def on_save_job(self, req, res):
        # 정상 종료 건만 들어온다. 취소·부분 결과는 task_manager가 보내지 않는다
        if self.con is None:
            res.saved = False
            res.message = self.db_error or 'DB 가 열려 있지 않다'
            return res
        if not req.job_id:
            res.saved = False
            res.message = 'job_id 가 비어 있다'
            return res

        try:
            db.save_job(
                self.con,
                req.job_id,
                db.time_to_text(req.finished_at.sec, req.finished_at.nanosec),
                req.overall_result,
                [string_result_to_row(s) for s in req.strings],
                settings_to_row(req.settings))
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().error('저장 실패 (%s): %r' % (req.job_id, exc))
            res.saved = False
            res.message = '저장 실패: %r' % exc
            return res

        self.get_logger().info('작업 %s 저장 (줄 %d개, 전체판정 %d)'
                               % (req.job_id, len(req.strings), req.overall_result))
        res.saved = True
        res.message = ''
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


if __name__ == '__main__':
    main()
