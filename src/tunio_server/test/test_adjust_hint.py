"""관제 화면의 '예상 페그 회전' 이 task_manager 가 실제로 돌릴 각도와 같은지 본다.

web_node 는 따로 공식을 두지 않고 tuning_logic.adjust_plan 을 그대로 부른다.
그 약속이 깨지면 화면은 '28°' 라고 하는데 로봇은 다른 각도를 돌린다.
"""
from pathlib import Path
from types import SimpleNamespace

import pytest

from tunio_server import peg_calibration as PC
from tunio_server import tuning_logic as L
from tunio_server.web_node import WebNode

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'peg_calibration.yaml'


def msg(string_number, measured_hz, tol=10.0, valid=True):
    target = L.TARGETS[string_number][1]
    return SimpleNamespace(current_string=string_number, measured_hz=measured_hz,
                           target_hz=target, measurement_valid=valid,
                           active_settings=SimpleNamespace(tolerance_cents=tol))


def node(cal):
    return SimpleNamespace(_cal=cal, _cal_note='캘리브레이션을 아직 읽지 못했다')


def hz_at(string_number, cents):
    return L.TARGETS[string_number][1] * 2 ** (cents / 1200.0)


@pytest.fixture
def cal():
    return PC.load(str(CONFIG))


@pytest.mark.parametrize('string_number,cents', [(2, 15.1), (5, -44.6), (6, 35.0), (1, -12.0)])
def test_hint_matches_task_manager_plan(cal, string_number, cents):
    hint = WebNode.adjust_hint(node(cal), msg(string_number, hz_at(string_number, cents)))
    expected = L.adjust_plan(L.error_cents(hz_at(string_number, cents),
                                           L.TARGETS[string_number][1]), cal.get(string_number))
    assert [(s['direction'], pytest.approx(s['deg'])) for s in hint['steps']] == expected
    for step in hint['steps']:
        assert step['turns'] == pytest.approx(step['deg'] / 360.0)
        assert step['label'] == ('조임' if step['direction'] > 0 else '풀기')


def test_sharp_needs_release_then_tighten(cal):
    """음이 높으면 백래시 때문에 풀었다가 조인다 (CLAUDE.md 3.2)."""
    hint = WebNode.adjust_hint(node(cal), msg(2, hz_at(2, 15.1)))
    assert [s['label'] for s in hint['steps']] == ['풀기', '조임']


@pytest.mark.parametrize('cents,valid', [(3.0, True), (40.0, False)])
def test_no_hint_inside_tolerance_or_without_measurement(cal, cents, valid):
    assert WebNode.adjust_hint(node(cal), msg(2, hz_at(2, cents), valid=valid)) is None


def test_missing_calibration_shows_reason_instead_of_numbers():
    hint = WebNode.adjust_hint(node(None), msg(2, hz_at(2, 20.0)))
    assert hint['steps'] == [] and hint['note']
