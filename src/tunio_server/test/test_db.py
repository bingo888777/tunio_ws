"""
db 저장·조회 테스트 (ROS 없이 실행 가능).

관심사는 셋이다.
  - 시각이 손실 없이 왕복하는가 (builtin_interfaces/Time <-> TEXT)
  - 측정 못 한 주파수가 NULL 로 남는가 (0Hz 를 유효값으로 쓰지 않는다)
  - 옛 스키마 파일을 조용히 쓰지 않고 막는가
"""
import sqlite3

import pytest

from tunio_server import db


def _con(tmp_path):
    return db.connect(tmp_path / 'test.db')


def _strings():
    return [
        {'string_number': 6, 'target_hz': 82.4069, 'tuned_hz': 82.41,
         'verified_hz': 82.40, 'final_error_cents': -0.2, 'adjust_count': 1,
         'result': 1, 'failure_reason': 0},
        # 측정 자체를 못 한 줄 — 주파수는 NULL 로 들어가야 한다
        {'string_number': 5, 'target_hz': 110.0, 'tuned_hz': None,
         'verified_hz': None, 'final_error_cents': 0.0, 'adjust_count': 0,
         'result': 2, 'failure_reason': 1},
    ]


def _settings():
    return {'tolerance_cents': 10.0, 'max_adjust_count': 5, 'max_measure_retry': 3,
            'measure_timeout_sec': 10.0, 'onset_timeout_sec': 10.0}


def test_time_roundtrip():
    for sec, nsec in ((1789783795, 315481978), (0, 0), (1700000000, 999999999)):
        assert db.text_to_time(db.time_to_text(sec, nsec)) == (sec, nsec)


def test_time_text_sorts_chronologically():
    """문자열 정렬이 곧 시간순이어야 ORDER BY 가 성립한다."""
    early = db.time_to_text(1700000000, 0)
    late = db.time_to_text(1700000001, 0)
    assert early < late


def test_bad_time_text_is_zero():
    assert db.text_to_time('') == (0, 0)
    assert db.text_to_time('뭔가 이상한 값') == (0, 0)


def test_save_and_get(tmp_path):
    con = _con(tmp_path)
    db.save_job(con, '20260922_143015', db.time_to_text(1789783795, 0), 2,
                _strings(), _settings())
    job, strings, settings = db.get_job(con, '20260922_143015')
    assert job['overall_result'] == 2
    assert len(strings) == 2
    assert settings['tolerance_cents'] == pytest.approx(10.0)


def test_unmeasured_frequency_is_null(tmp_path):
    """0Hz 를 유효값으로 쓰지 않는다 (CLAUDE.md 4절)."""
    con = _con(tmp_path)
    db.save_job(con, 'j1', db.time_to_text(1789783795, 0), 2, _strings(), _settings())
    _, strings, _ = db.get_job(con, 'j1')
    by_num = {s['string_number']: s for s in strings}
    assert by_num[5]['tuned_hz'] is None
    assert by_num[5]['verified_hz'] is None
    assert by_num[6]['tuned_hz'] is not None


def test_list_jobs_newest_first(tmp_path):
    con = _con(tmp_path)
    for i, job_id in enumerate(('20260922_100000', '20260922_120000', '20260922_110000')):
        db.save_job(con, job_id, db.time_to_text(1789780000 + i * 3600, 0), 1,
                    _strings(), _settings())
    rows = db.list_jobs(con, 10)
    assert [r['job_id'] for r in rows] == \
        ['20260922_110000', '20260922_120000', '20260922_100000']


def test_list_jobs_limit(tmp_path):
    con = _con(tmp_path)
    for i in range(5):
        db.save_job(con, 'j%d' % i, db.time_to_text(1789780000 + i, 0), 1,
                    _strings(), _settings())
    assert len(db.list_jobs(con, 2)) == 2


def test_missing_job(tmp_path):
    con = _con(tmp_path)
    assert db.get_job(con, '없는작업') == (None, [], None)


def test_resave_replaces_rows(tmp_path):
    """같은 job_id 를 다시 저장해도 줄 기록이 두 벌로 쌓이지 않는다."""
    con = _con(tmp_path)
    for _ in range(3):
        db.save_job(con, 'j1', db.time_to_text(1789783795, 0), 1, _strings(), _settings())
    _, strings, _ = db.get_job(con, 'j1')
    assert len(strings) == 2


def test_old_schema_is_refused(tmp_path):
    """
    구 스키마 파일이 남아 있으면 조용히 쓰지 않고 막는다.

    CREATE TABLE IF NOT EXISTS 만 있던 시절의 파일이 다른 PC 에 남아 있을 수 있다.
    """
    path = tmp_path / 'old.db'
    con = sqlite3.connect(str(path))
    con.executescript('CREATE TABLE jobs (job_id TEXT PRIMARY KEY, pin_check_done INTEGER);')
    con.commit()
    con.close()
    with pytest.raises(db.SchemaMismatch):
        db.connect(path)


def test_version_mismatch_is_refused(tmp_path):
    """컬럼이 같아도 버전이 다르면 막는다."""
    path = tmp_path / 'v9.db'
    con = db.connect(path)
    con.execute('PRAGMA user_version = 9')
    con.commit()
    con.close()
    with pytest.raises(db.SchemaMismatch):
        db.connect(path)
