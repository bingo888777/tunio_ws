"""
그리퍼가 지금 얼마나 벌어져 있는지 Modbus 로 읽는다 (이슈 #31).

왜 이 경로인가
  제어는 디지털 I/O + WebLogic 으로 한다(gripper.py). 그런데 **상태는 그 길로 못
  읽는다.** Compute Box 가 Modbus 홀딩 레지스터로 내주는 것을 따로 읽어야 한다.
  제어와 읽기의 경로가 다르다. 헷갈리지 말 것.

  읽기 전용이다. 기능 코드 3(읽기)만 보내므로 구조적으로 아무것도 못 쓴다.

왜 폭이어야 하나
  페그를 제대로 물었는지 판정할 수단이 이것뿐이다.
    · 회전 토크 — 2026-09-21 실측으로 부정됐다. 웜기어 14:1 을 지나면 줄 장력 몫이
      손잡이에서 0.015 Nm 라 센서 잡음 0.11 Nm 에 묻힌다. 파지·허공이 구분 안 된다
    · 파지 감지 플래그(레지스터 11) — **빈손일 때도 '있음' 으로 나온다.** 못 쓴다
  폭만 믿는다. 2026-09-21 6번 줄 실측에서 19.6 / 32.8 / 13.0 으로 깨끗이 갈렸다.
"""
import socket
import struct

BASE = 258          # 상태 레지스터 시작 주소
COUNT = 18          # onrobot_rg_control 의 comModbusTcp.getStatus 가 읽는 개수
WIDTH_REG = 9       # 손가락 사이 폭. 0.1mm 단위. 2026-09-21 실물 확인
DEFAULT_HOST = "192.168.1.1"
DEFAULT_PORT = 502
DEFAULT_UNIT = 65   # 퀵 체인저 주소


class WidthError(Exception):
    """폭을 못 읽었을 때. 판정을 못 하므로 부르는 쪽이 멈춰야 한다."""


def read_registers(host, port, unit, address, count, timeout=2.0):
    """Modbus/TCP 기능 코드 3 으로 홀딩 레지스터를 읽는다. 쓰기는 안 한다."""
    req = struct.pack(">HHHBBHH", 1, 0, 6, unit, 3, address, count)
    try:
        with socket.create_connection((host, port), timeout=timeout) as sock:
            sock.sendall(req)
            head = b""
            while len(head) < 9:
                chunk = sock.recv(9 - len(head))
                if not chunk:
                    raise WidthError("응답이 끊겼다 (헤더)")
                head += chunk
            nbytes = head[8]
            body = b""
            while len(body) < nbytes:
                chunk = sock.recv(nbytes - len(body))
                if not chunk:
                    raise WidthError("응답이 끊겼다 (본문)")
                body += chunk
    except OSError as e:
        raise WidthError(f"{host}:{port} 에 못 붙었다 ({e}). "
                         "PC 가 로봇에 유선으로 물려 있는지 확인하라")
    if len(body) < count * 2:
        raise WidthError(f"레지스터를 {len(body) // 2}개만 받았다 (요청 {count})")
    return list(struct.unpack(f">{count}H", body[:count * 2]))


def make_reader(host=DEFAULT_HOST, port=DEFAULT_PORT, unit=DEFAULT_UNIT,
                width_reg=WIDTH_REG):
    """
    폭(mm)을 돌려주는 함수를 만든다.

    함수로 넘기는 이유 — motion_primitives 가 Modbus 를 직접 알면 로봇 없이
    테스트할 수 없다. 부르는 쪽이 이걸 주입한다.
    """
    def read():
        regs = read_registers(host, port, unit, BASE, COUNT)
        if width_reg >= len(regs):
            raise WidthError(f"레지스터 {width_reg} 번이 응답에 없다 (총 {len(regs)}개)")
        return regs[width_reg] / 10.0
    return read


def classify(width_mm, thresholds):
    """
    폭 하나로 세 경우를 가른다 (이슈 #31).

    thresholds — {'empty_below': …, 'dead_above': …}. teach_points 의 peg_handle 절.

      'hold'   제대로 물렸다. 손잡이가 손가락과 나란하다
      'dead'   사점. 손잡이가 비스듬해 못 닫고 멈췄다. 열고 돌려서 다시
      'empty'  빈손. 페그를 빗나갔다
    """
    if width_mm is None:
        raise WidthError("폭이 없다")
    if width_mm < thresholds["empty_below"]:
        return "empty"
    if width_mm > thresholds["dead_above"]:
        return "dead"
    return "hold"
