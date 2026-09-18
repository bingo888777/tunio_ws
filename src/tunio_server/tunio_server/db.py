"""
SQLite 접근. 테이블 3개: jobs / string_results / settings_snapshot.
기획서 9.2. 컬럼은 tunio_interfaces 의 JobSummary / StringResult / TuningSettings 에 맞춘다.

열거값은 정수로 저장한다. 의미는 메시지 정의가 기준:
  result          -> StringResult.RESULT_*
  failure_reason  -> StringResult.FAIL_*
  overall_result  -> TuningState.OVERALL_*
번호를 바꾸면 기존 행이 조용히 재해석되므로 상수는 뒤에 추가만 한다.
"""
import sqlite3
from pathlib import Path

DB_PATH = Path.home() / ".tunio" / "tunio.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    job_id         TEXT PRIMARY KEY,   -- "20260922_143015" 형식. task_manager가 생성
    finished_at    TEXT,               -- 작업 종료 시각
    overall_result INTEGER             -- TuningState.OVERALL_*
);
CREATE TABLE IF NOT EXISTS string_results (
    id                INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id            TEXT,
    string_number     INTEGER,         -- 실제 줄 번호 (6, 5, 2, 1)
    target_hz         REAL,
    tuned_hz          REAL,            -- has_tuned_hz=false면 NULL (0 사용 금지)
    verified_hz       REAL,            -- has_verified_hz=false면 NULL
    final_error_cents REAL,
    adjust_count      INTEGER,
    result            INTEGER,         -- StringResult.RESULT_*
    failure_reason    INTEGER,         -- StringResult.FAIL_*
    FOREIGN KEY (job_id) REFERENCES jobs(job_id)
);
CREATE TABLE IF NOT EXISTS settings_snapshot (
    job_id              TEXT PRIMARY KEY,
    tolerance_cents     REAL,
    max_adjust_count    INTEGER,
    max_measure_retry   INTEGER,
    measure_timeout_sec REAL,
    onset_timeout_sec   REAL,
    FOREIGN KEY (job_id) REFERENCES jobs(job_id)
);
"""


def connect():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(DB_PATH)
    con.executescript(SCHEMA)
    return con
