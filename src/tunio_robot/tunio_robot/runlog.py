"""
실행 기록 저장 — 도구가 잰 값을 파일로 떨군다.

왜 필요한가
  2026-09-19 까지는 도구가 화면에만 찍고 아무것도 안 남겼다. 사람이 눈으로 보고
  문서에 옮겨 적는 방식이라 실제로 놓친 것이 있다 — 순회에서 나온 5·2·1번의
  팔 모양(sol)이 화면에만 뜨고 사라져 yaml 이 비어 있다.
  정정인 쪽 tools/peg_cal.py 는 docs/peg_data/ 에 원자료를 남긴다. 같은 방식을 쓴다.

어디에 남기나
  docs/robot_data/<날짜>_<시각>_<이름>.json
  원자료 그대로 남긴다. 요약은 나중에 얼마든지 다시 낼 수 있지만
  원자료는 그 순간을 놓치면 끝이다.

비교
  latest() 로 조건에 맞는 가장 최근 기록을 찾는다. 파지한 회전과 허공 회전처럼
  두 번 재서 견줘야 하는 항목에 쓴다.
"""
import json
import os
import time


def save(name, payload, out_dir):
    """
    기록 하나를 json 으로 떨구고 경로를 낸다.

    저장에 실패해도 예외를 올리지 않는다. 로그 때문에 실측이 날아가면 안 된다.
    실패하면 None 을 낸다.
    """
    try:
        os.makedirs(out_dir, exist_ok=True)
        stamp = time.strftime("%Y-%m-%d_%H%M%S")
        path = os.path.join(out_dir, f"{stamp}_{name}.json")
        body = dict(payload)
        body["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(body, f, ensure_ascii=False, indent=2)
        return path
    except OSError:
        return None


def session_path(name, out_dir):
    """
    이번 실행이 계속 덮어쓸 파일 경로 하나를 잡는다 (노드용).

    save() 는 부를 때마다 시각이 붙어 새 파일이 된다. 도구는 한 번 재고 끝이라
    그게 맞지만, 노드는 goal 을 수십 번 받으므로 그대로 쓰면 파일이 쏟아진다.
    시작할 때 경로를 하나 잡아 두고 goal 이 끝날 때마다 write() 로 덮어쓴다.
    """
    return os.path.join(out_dir, f"{time.strftime('%Y-%m-%d_%H%M%S')}_{name}.json")


def write(path, payload):
    """
    session_path() 로 잡은 파일에 덮어쓴다. 성공하면 경로, 실패하면 None.

    save() 와 같은 이유로 예외를 안 올린다 — 기록 때문에 실기가 날아가면 안 된다.
    goal 이 끝날 때마다 통째로 다시 쓴다. 중간에 노드가 죽어도 그때까지가 남는다.
    """
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        body = dict(payload)
        body["saved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(body, f, ensure_ascii=False, indent=2)
        return path
    except OSError:
        return None


def load(path):
    """기록 하나를 읽는다. 못 읽으면 None."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def latest(out_dir, match=None):
    """
    조건에 맞는 가장 최근 기록을 (경로, 내용) 으로 낸다. 없으면 (None, None).

    match — 내용을 받아 참/거짓을 내는 함수. 없으면 아무거나 최근 것.
    파일명이 날짜_시각으로 시작하므로 이름순 정렬이 곧 시간순이다.
    """
    try:
        names = sorted(os.listdir(out_dir), reverse=True)
    except OSError:
        return None, None
    for name in names:
        if not name.endswith(".json"):
            continue
        path = os.path.join(out_dir, name)
        body = load(path)
        if body is None:
            continue
        if match is None or match(body):
            return path, body
    return None, None
