"""
두산 API를 감싸는 저수준 동작 함수. robot_controller가 호출.

DRL/DRFL 함수는 펌웨어 버전 확인 후 채운다.

TODO(정묵·성현):
  - set_tcp(피크 장착 기준), 순응/힘 제어 진입·해제
  - pluck_string: 힘 제어로 일정 세기 타현 + 발현 직후 5mm 후퇴
  - adjust_peg: 받은 방향·각도만 수행. 백래시 대응(풀었다 조이기)은 서버가 결정한다
"""


def pluck_string(string_number: int):
    """줄 튕기기. 힘 제어로 일정한 세기 유지 후 즉시 후퇴."""
    raise NotImplementedError


def adjust_peg(string_number: int, direction: int, angle_deg: float):
    """
    페그 조절. 받은 방향과 각도를 그대로 수행한다.

    direction: +1 조임(음 높이기) / -1 풀림. AdjustPeg.Goal.direction 과 같은 규약.
    angle_deg: 항상 양수 크기.
    줄별 실제 모터 회전 방향 변환은 로봇측 YAML이 담당한다.

    백래시 대응(목표보다 높으면 아래로 풀었다 조여 올리기)은 task_manager가
    결정한다(명세 9.1 확정). 여기서 임의로 조이는 방향 마무리를 덧붙이면
    서버가 계산한 각도와 이중으로 겹쳐 결과가 어긋난다.
    """
    raise NotImplementedError


def strum_all(string_numbers: list):
    """전체 줄을 훑어 화음. 측정 없는 완료 연출. 비우면 전체."""
    raise NotImplementedError
