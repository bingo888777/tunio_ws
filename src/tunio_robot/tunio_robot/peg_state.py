"""
페그 손잡이가 지금 몇 도 돌아가 있는지 기억한다 (이슈 #31).

왜 필요한가
  로봇은 페그 손잡이의 현재 각을 모른다. 사람이 마지막으로 돌려놓은 상태라
  매번 다르다(CLAUDE.md 5절 Phase 0). 그래서 작업 시작 전에 줄마다 한 번씩
  잡아보고 각을 기록한다.

  그런데 **조절을 하면 그 각이 또 바뀐다.** adjust_peg 가 60도를 돌리면 손잡이도
  60도 돌아간 채로 남는다. 그 상태를 안 들고 있으면 다음에 교시된 원래 각으로
  들어가 헛문다. 2026-09-21 이전 코드가 그랬다.

무엇을 저장하나
  **교시 자세 기준 상대 각(roll_deg)** 이다. 절대 방위가 아니다.
  교시할 때의 손목 자세를 0 으로 보고, 거기서 얼마나 더 돌려야 지금 손잡이와
  맞는지를 적는다. 이 값을 파지 자세의 C(툴 Z 둘레 회전)에 더하면 된다 —
  intrinsic ZYZ 에서 마지막 Rz(C) 가 툴 Z 둘레 회전이고, 툴 Z 자체는 안 바뀌므로
  접근축은 그대로다 (teach_points.approach_axis 주석 참조).

왜 파일로 남기나
  노드가 죽었다 살아나도 Phase 0 를 다시 돌지 않게 하려는 것이다. 등록은
  줄마다 잡아보는 과정이라 싸지 않다.

  대신 **오래된 값은 위험하다.** 사람이 그사이 페그를 손으로 돌렸으면 기록은
  거짓말이 된다. 그래서 등록 시각을 같이 적고, max_age_sec 를 넘으면 미등록으로
  본다. 미등록이면 Phase 0 를 다시 돌아야 한다.
"""
from contextlib import contextmanager
import json
import os
import time

# 페그 손잡이는 납작한 막대라 180도 대칭이다. θ 로 잡으나 θ+180 으로 잡으나 같다.
# 그래서 [-90, 90) 으로 접어 손목이 덜 돌게 한다
_PERIOD_DEG = 180.0

DEFAULT_PATH = os.path.expanduser("~/.tunio/peg_state.json")

# 등록값을 믿는 기본 시간. 작업 한 판보다 넉넉하고 하루보다는 짧게 잡는다.
# 사람이 밤새 페그를 만졌을 수 있다
DEFAULT_MAX_AGE_SEC = 4 * 3600.0


def fold(deg):
    """손잡이 각을 [-90, 90) 으로 접는다. 180도 대칭이라 같은 자세다."""
    x = (float(deg) + _PERIOD_DEG / 2.0) % _PERIOD_DEG
    if x < 0:
        x += _PERIOD_DEG
    return x - _PERIOD_DEG / 2.0


class PegState:
    """페그별 손잡이 각과 등록 시각. 파일로 남는다."""

    def __init__(self, path=DEFAULT_PATH, max_age_sec=DEFAULT_MAX_AGE_SEC, clock=time.time):
        self.path = path
        self.max_age_sec = float(max_age_sec)
        self._clock = clock
        self._rolls = {}        # {줄번호: 손잡이 각(도)}
        self._stamps = {}       # {줄번호: 등록·갱신 시각(epoch)}

    # ------------------------------------------------------------ 읽기
    def roll_of(self, string_number):
        """
        손잡이 각(도). 등록 안 됐거나 너무 오래됐으면 None.

        None 이면 **그 각으로 들어가면 안 된다.** Phase 0 를 먼저 돌아야 한다.
        """
        no = int(string_number)
        if no not in self._rolls:
            return None
        if self.is_stale(no):
            return None
        return self._rolls[no]

    def is_stale(self, string_number):
        """등록 시각이 max_age_sec 를 넘었나. 기록이 없으면 오래된 것으로 본다."""
        no = int(string_number)
        stamp = self._stamps.get(no)
        if stamp is None:
            return True
        return (self._clock() - stamp) > self.max_age_sec

    def registered(self):
        """지금 믿을 수 있는 줄 번호들."""
        return sorted(no for no in self._rolls if not self.is_stale(no))

    # ------------------------------------------------------------ 쓰기
    def register(self, string_number, roll_deg):
        """Phase 0 가 찾아낸 각을 기록한다. 시각도 같이 찍힌다."""
        no = int(string_number)
        self._rolls[no] = fold(roll_deg)
        self._stamps[no] = self._clock()
        return self._rolls[no]

    def add_turn(self, string_number, delta_deg):
        """
        조절한 만큼 손잡이 각을 누적한다.

        **등록 안 된 줄에는 쓰지 않는다.** 기준이 없는데 누적하면 0 에서 시작한
        것처럼 보여 더 위험하다. 부르는 쪽이 등록을 먼저 확인해야 한다.
        """
        no = int(string_number)
        if no not in self._rolls:
            raise KeyError(f"{no}번 페그가 등록되지 않았다. Phase 0 를 먼저 돌려라")
        self._rolls[no] = fold(self._rolls[no] + float(delta_deg))
        self._stamps[no] = self._clock()
        return self._rolls[no]

    def forget(self, string_number=None):
        """등록을 지운다. 인자가 없으면 전부. 헛잡은 게 확인되면 부른다."""
        if string_number is None:
            self._rolls.clear()
            self._stamps.clear()
            return
        no = int(string_number)
        self._rolls.pop(no, None)
        self._stamps.pop(no, None)

    @contextmanager
    def turning(self, string_number, delta_deg):
        """움직이기 전에 디스크까지 무효화. 후퇴까지 성공해야 새 각을 확정한다 (#70)."""
        before = self.roll_of(string_number)
        if before is None:
            raise ValueError(f"{string_number}번 페그 등록이 필요하다")
        self.forget(string_number)
        self.save()             # 실패하면 아직 로봇은 안 움직였다
        yield                   # 중간 실패·프로세스 종료 시 디스크도 미등록으로 남는다
        if delta_deg is None:   # J6 외의 관절은 손잡이 각을 추적할 수 없다
            return
        self.register(string_number, before + delta_deg)
        try:
            self.save()
        except Exception:
            self.forget(string_number)
            raise

    # ------------------------------------------------------------ 파일
    def load(self):
        """파일에서 읽는다. 없거나 깨졌으면 빈 상태로 둔다 — 그러면 Phase 0 를 돈다."""
        try:
            with open(self.path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return self
        pegs = data.get("pegs") if isinstance(data, dict) else None
        if not isinstance(pegs, dict):
            return self
        for key, item in pegs.items():
            try:
                no = int(key)
                self._rolls[no] = fold(float(item["roll_deg"]))
                self._stamps[no] = float(item["at"])
            except (TypeError, ValueError, KeyError):
                continue          # 한 줄이 깨져도 나머지는 살린다
        return self

    def save(self):
        """파일로 남긴다. 디렉터리가 없으면 만든다."""
        data = {
            "pegs": {str(no): {"roll_deg": self._rolls[no], "at": self._stamps[no]}
                     for no in sorted(self._rolls)},
            "saved_at": self._clock(),
            "note": "페그 손잡이 각(교시 자세 기준 상대, 도). tunio_robot/peg_state.py",
        }
        d = os.path.dirname(self.path)
        if d:
            os.makedirs(d, exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.path)    # 쓰다 죽어도 이전 파일이 남는다
        return self.path
