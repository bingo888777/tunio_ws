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


def press_pin(string_no: int) -> bool:
    """브릿지핀을 힘 제어로 눌러 안착 확인. 반환: 안착 여부."""
    raise NotImplementedError


def strum_all(strings: list):
    """전체 줄을 훑어 화음. 측정 없는 완료 연출. 비우면 전체."""
    raise NotImplementedError
