"""
SQLite 접근. 테이블 3개: jobs / string_results / settings_snapshot.
기획서 9.2. 상세 스키마는 구현 시 확정.
"""
import sqlite3
from pathlib import Path

DB_PATH = Path.home() / ".tunio" / "tunio.db"

SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    job_id     INTEGER PRIMARY KEY AUTOINCREMENT,
    started_at TEXT,
    overall    TEXT
);
CREATE TABLE IF NOT EXISTS string_results (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id       INTEGER,
    string_no    INTEGER,
    target_hz    REAL,
    final_hz     REAL,          -- 유효값 없으면 NULL (0 사용 금지)
    verify_hz    REAL,          -- 최종 검증 패스 측정값
    error_cents  REAL,
    adjust_count INTEGER,
    status       INTEGER,
    fail_reason  TEXT,
    FOREIGN KEY (job_id) REFERENCES jobs(job_id)
);
CREATE TABLE IF NOT EXISTS settings_snapshot (
    job_id           INTEGER PRIMARY KEY,
    tolerance_cents  REAL,
    max_adjust       INTEGER,
    measure_timeout  REAL,
    FOREIGN KEY (job_id) REFERENCES jobs(job_id)
);
"""


def connect():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(DB_PATH)
    con.executescript(SCHEMA)
    return con
