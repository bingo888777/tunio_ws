"""
SQLite 접근. 테이블 3개: jobs / string_results / settings_snapshot.

기획서 10.2. 컬럼은 tunio_interfaces 의 JobSummary / StringResult / TuningSettings 에 맞춘다.
이 모듈은 rclpy 와 메시지 타입에 의존하지 않는다. 메시지 <-> dict 변환은 result_store 가 한다.
로봇·마이크 없이 테스트하기 위한 분리다.

열거값은 정수로 저장한다. 의미는 메시지 정의가 기준:
  result          -> StringResult.RESULT_*
  failure_reason  -> StringResult.FAIL_*
  overall_result  -> TuningState.OVERALL_*
번호를 바꾸면 기존 행이 조용히 재해석되므로 상수는 뒤에 추가만 한다.

시각 표기: builtin_interfaces/Time 은 sec(int32) + nanosec(uint32) 인데 DB 는 TEXT 다.
ISO 8601 UTC 에 소수점 이하 9자리를 붙여 넣는다 ('2026-09-22T14:30:15.123456789Z').
  - sec/nanosec 를 손실 없이 왕복한다
  - 문자열 사전순 정렬이 곧 시간순이라 ORDER BY 에 그대로 쓴다
  - 사람이 읽을 수 있다
"""
from datetime import datetime, timezone
from pathlib import Path
import sqlite3

DB_PATH = Path.home() / '.tunio' / 'tunio.db'

# 스키마를 바꾸면 이 번호를 올린다. 기존 파일은 자동으로 갱신되지 않는다.
SCHEMA_VERSION = 1

SCHEMA = """
CREATE TABLE IF NOT EXISTS jobs (
    job_id         TEXT PRIMARY KEY,   -- "20260922_143015" 형식. task_manager가 생성
    finished_at    TEXT,               -- ISO 8601 UTC, 소수점 이하 9자리
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

# 스키마가 맞는지 확인할 때 쓰는 기대 컬럼. 순서는 보지 않는다.
EXPECTED_COLUMNS = {
    'jobs': {'job_id', 'finished_at', 'overall_result'},
    'string_results': {'id', 'job_id', 'string_number', 'target_hz', 'tuned_hz',
                       'verified_hz', 'final_error_cents', 'adjust_count',
                       'result', 'failure_reason'},
    'settings_snapshot': {'job_id', 'tolerance_cents', 'max_adjust_count',
                          'max_measure_retry', 'measure_timeout_sec', 'onset_timeout_sec'},
}

STRING_FIELDS = ('string_number', 'target_hz', 'tuned_hz', 'verified_hz',
                 'final_error_cents', 'adjust_count', 'result', 'failure_reason')
SETTINGS_FIELDS = ('tolerance_cents', 'max_adjust_count', 'max_measure_retry',
                   'measure_timeout_sec', 'onset_timeout_sec')


class SchemaMismatch(Exception):
    """기존 DB 파일의 스키마가 지금 코드와 다르다. 조용히 쓰면 안 된다."""


# --------------------------------------------------------------------- 시각 변환

def time_to_text(sec: int, nanosec: int) -> str:
    """builtin_interfaces/Time 의 sec·nanosec -> ISO 8601 UTC 문자열."""
    base = datetime.fromtimestamp(int(sec), tz=timezone.utc)
    return '%s.%09dZ' % (base.strftime('%Y-%m-%dT%H:%M:%S'), int(nanosec))


def text_to_time(text: str) -> tuple:
    """ISO 8601 UTC 문자열 -> (sec, nanosec). 빈 값이나 형식이 틀리면 (0, 0)."""
    if not text:
        return 0, 0
    try:
        head, frac = text.rstrip('Z').split('.')
        base = datetime.strptime(head, '%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc)
        return int(base.timestamp()), int(frac.ljust(9, '0')[:9])
    except (ValueError, AttributeError):
        return 0, 0


# --------------------------------------------------------------------- 연결

def _tables(con) -> set:
    rows = con.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name NOT LIKE 'sqlite_%'")
    return {r[0] for r in rows}


def _columns(con, table: str) -> set:
    return {r[1] for r in con.execute('PRAGMA table_info(%s)' % table)}


def connect(path=None):
    """
    DB 를 열고 스키마를 확인한다.

    CREATE TABLE IF NOT EXISTS 만으로는 컬럼이 바뀐 옛 파일이 그대로 살아남아
    INSERT 가 조용히 실패한다. 그래서 PRAGMA user_version 으로 버전을 박아 두고,
    다르면 SchemaMismatch 를 던진다. 자동 마이그레이션은 하지 않는다 —
    시연용 데이터라 잃어도 되고, 잘못 옮기는 쪽이 더 위험하다.
    """
    p = Path(path) if path is not None else DB_PATH
    p.parent.mkdir(parents=True, exist_ok=True)
    con = sqlite3.connect(str(p))
    con.row_factory = sqlite3.Row
    con.execute('PRAGMA foreign_keys = ON')

    existing = _tables(con)
    version = con.execute('PRAGMA user_version').fetchone()[0]

    if not existing:
        con.executescript(SCHEMA)
        con.execute('PRAGMA user_version = %d' % SCHEMA_VERSION)
        con.commit()
        return con

    if version != SCHEMA_VERSION:
        con.close()
        raise SchemaMismatch(
            '기존 DB 스키마 버전이 %d 인데 코드는 %d 를 쓴다: %s\n'
            '자동 변환하지 않는다. 파일을 옮기거나 지우고 다시 실행해라.'
            % (version, SCHEMA_VERSION, p))

    for table, expected in EXPECTED_COLUMNS.items():
        if table not in existing:
            con.close()
            raise SchemaMismatch('DB 에 %s 테이블이 없다: %s' % (table, p))
        actual = _columns(con, table)
        if actual != expected:
            con.close()
            raise SchemaMismatch(
                '%s 테이블 컬럼이 다르다 (없음 %s / 남음 %s): %s'
                % (table, sorted(expected - actual), sorted(actual - expected), p))
    return con


# --------------------------------------------------------------------- 저장·조회

def save_job(con, job_id, finished_at, overall_result, strings, settings):
    """
    작업 한 건을 한 트랜잭션으로 저장한다.

    같은 job_id 를 다시 저장하면 기존 행을 지우고 새로 넣는다. job_id 가 초 단위
    타임스탬프라 같은 값이 다시 올 일은 사실상 없지만, 재시도로 중복 저장됐을 때
    줄 기록이 두 벌로 쌓이는 쪽보다 낫다.
    """
    with con:
        con.execute('DELETE FROM string_results WHERE job_id = ?', (job_id,))
        con.execute('DELETE FROM settings_snapshot WHERE job_id = ?', (job_id,))
        con.execute('DELETE FROM jobs WHERE job_id = ?', (job_id,))
        con.execute('INSERT INTO jobs (job_id, finished_at, overall_result) VALUES (?, ?, ?)',
                    (job_id, finished_at, int(overall_result)))
        con.executemany(
            'INSERT INTO string_results '
            '(job_id, string_number, target_hz, tuned_hz, verified_hz, final_error_cents,'
            ' adjust_count, result, failure_reason) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)',
            [(job_id,) + tuple(s[f] for f in STRING_FIELDS) for s in strings])
        con.execute(
            'INSERT INTO settings_snapshot '
            '(job_id, tolerance_cents, max_adjust_count, max_measure_retry,'
            ' measure_timeout_sec, onset_timeout_sec) VALUES (?, ?, ?, ?, ?, ?)',
            (job_id,) + tuple(settings[f] for f in SETTINGS_FIELDS))


def list_jobs(con, limit):
    """최신순 작업 목록. finished_at 이 ISO 8601 이라 문자열 정렬이 곧 시간순이다."""
    rows = con.execute(
        'SELECT job_id, finished_at, overall_result FROM jobs '
        'ORDER BY finished_at DESC, job_id DESC LIMIT ?', (int(limit),))
    return [dict(r) for r in rows]


def get_job(con, job_id):
    """작업 한 건의 상세. (job, strings, settings). 없으면 (None, [], None)."""
    job = con.execute(
        'SELECT job_id, finished_at, overall_result FROM jobs WHERE job_id = ?',
        (job_id,)).fetchone()
    if job is None:
        return None, [], None
    strings = [dict(r) for r in con.execute(
        'SELECT * FROM string_results WHERE job_id = ? ORDER BY id', (job_id,))]
    settings = con.execute(
        'SELECT * FROM settings_snapshot WHERE job_id = ?', (job_id,)).fetchone()
    return dict(job), strings, (dict(settings) if settings is not None else None)
