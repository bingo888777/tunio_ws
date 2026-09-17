"""
두산 API를 감싸는 저수준 동작 함수. robot_controller가 호출.
DRL/DRFL 함수는 펌웨어 버전 확인 후 채운다.

TODO(정묵·성현):
  - set_tcp(피크 장착 기준), 순응/힘 제어 진입·해제
  - pluck_string: 힘 제어로 일정 세기 타현 + 발현 직후 5mm 후퇴
  - adjust_peg: 조이는 방향 마무리(백래시 대응)
"""


def pluck_string(string_no: int):
    """줄 튕기기. 힘 제어로 일정한 세기 유지 후 즉시 후퇴."""
    raise NotImplementedError


def adjust_peg(string_no: int, raise_pitch: bool, angle_deg: float):
    """페그 조절. 최종 접근은 항상 조이는 방향."""
    raise NotImplementedError


def transfer(direction: int):
    """이송(후순위). 0=작업대로, 1=원위치로."""
    raise NotImplementedError
