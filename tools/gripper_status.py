#!/usr/bin/env python3
"""
그리퍼 상태 읽기 — RG2 가 지금 얼마나 벌어져 있고 뭘 물었는지 본다.

**읽기만 한다.** Modbus 기능 코드 3(홀딩 레지스터 읽기) 외에는 보내지 않는다.
구조적으로 아무것도 못 쓴다. 디지털 I/O 제어와 같이 켜 둬도 된다.

왜 이게 생겼나
  CLAUDE.md 3.6 에 "그리퍼 자체의 파지 신호(grip detected)는 못 읽는다" 고
  적어 뒀는데 **틀렸다**(2026-09-19 정정). 토픽으로 내보내는 노드가 없을 뿐,
  Compute Box 는 Modbus 로 폭과 파지 감지를 그대로 내준다.
  onrobot_rg_control/onrobot_rg_communication/comModbusTcp.py 의 getStatus 가
  레지스터 258~275 를 읽어 쓰고 있다. 그 읽기만 떼어냈다.

  pymodbus 는 이 PC 에 없다. 설치 없이 돌게 소켓으로 직접 짰다.

레지스터 뜻
  OnRobot 드라이버 코드가 추정한 배치를 같이 찍지만 **원본 18개를 통째로**
  보여준다. 배치가 맞는지는 그리퍼를 여닫으며 어느 칸이 변하는지로 확인하라.
  그게 확실하다.

어디에 붙나
  로봇 컨트롤러가 192.168.1.100 이다(2026-09-19 확인). Compute Box 는 별도
  주소이고, 예전 화면에서 192.168.1.1 로 적어 뒀으나 확실하지 않다.
  --scan 이 흔한 조합을 훑어 어디가 답하는지 찾는다. 읽기만 보낸다.

쓰는 법
  python3 tools/gripper_status.py --scan     # 어디에 붙는지 먼저 찾는다
  python3 tools/gripper_status.py            # 한 번 읽는다
  python3 tools/gripper_status.py --watch    # 계속 읽는다 (여닫으며 보기)
  python3 tools/gripper_status.py --host 192.168.1.1 --unit 65
"""
import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..",
                                "src", "tunio_robot"))

LOG_DIR = os.path.join(os.path.dirname(__file__), "..", "docs", "robot_data")

BASE = 258          # 상태 레지스터 시작 주소
COUNT = 18          # comModbusTcp 가 읽는 개수

# comModbusTcp.getStatus 가 쓰는 배치. **추정이다.** 실물로 확인할 것
GUESS = {
    0: ("손끝 오프셋", "mm", 10.0),
    5: ("깊이", "mm", 10.0),
    6: ("상대 깊이", "mm", 10.0),
    9: ("손가락 사이 폭", "mm", 10.0),
    10: ("동작 중(busy)", "", None),
    11: ("파지 감지(grip)", "", None),
    12: ("안전 스위치1 눌림", "", None),
    13: ("안전 회로1 작동", "", None),
    14: ("안전 스위치2 눌림", "", None),
    15: ("안전 회로2 작동", "", None),
    17: ("오프셋 반영 폭", "mm", 10.0),
}


def read_registers(host, port, unit, address, count, timeout=2.0):
    """Modbus/TCP 기능 코드 3 으로 홀딩 레지스터를 읽는다. 쓰기는 안 한다."""
    req = struct.pack(">HHHBBHH", 1, 0, 6, unit, 3, address, count)
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(req)
        head = b""
        while len(head) < 8:
            chunk = sock.recv(8 - len(head))
            if not chunk:
                raise IOError("응답이 끊겼다")
            head += chunk
        _, _, length, _, func = struct.unpack(">HHHBB", head)
        body = b""
        need = length - 2
        while len(body) < need:
            chunk = sock.recv(need - len(body))
            if not chunk:
                raise IOError("응답이 끊겼다")
            body += chunk
        if func & 0x80:
            raise IOError(f"Modbus 예외 코드 {body[0]}")
        nbytes = body[0]
        vals = struct.unpack(f">{nbytes // 2}H", body[1:1 + nbytes])
        return list(vals)


# --scan 이 훑는 후보. 로봇 컨트롤러는 192.168.1.100 이고 Compute Box 는 별도다
SCAN_HOSTS = ["192.168.1.1", "192.168.1.100", "192.168.1.2", "192.168.1.101"]
SCAN_UNITS = [65, 66, 1, 0]


def scan(extra_host, port):
    """어느 주소·유닛이 답하는지 훑는다. 읽기만 보낸다."""
    hosts = [extra_host] + [h for h in SCAN_HOSTS if h != extra_host]
    print("읽기 요청만 보낸다. 아무것도 안 바꾼다.\n")
    found = []
    for host in hosts:
        try:
            with socket.create_connection((host, port), timeout=0.8):
                pass
        except OSError as e:
            print(f"  {host}:{port}  못 붙는다 ({e.__class__.__name__})")
            continue
        print(f"  {host}:{port}  포트 열림")
        for unit in SCAN_UNITS:
            try:
                regs = read_registers(host, port, unit, BASE, COUNT, timeout=1.0)
            except (OSError, IOError) as e:
                print(f"      유닛 {unit:3d}  응답 없음 ({e})")
                continue
            width = regs[9] / 10.0
            sane = 0.0 <= width <= 120.0 and any(regs)
            print(f"      유닛 {unit:3d}  답했다 — 폭 추정 {width:.1f}mm"
                  f"{'' if sane else '  (값이 이상하다)'}")
            found.append((host, unit, regs, sane))

    print()
    if not found:
        print("답하는 데가 없다. 볼 것 —")
        print("  · ping 192.168.1.1 / ping 192.168.1.100 이 되나")
        print("  · 그리퍼 제어 웹이 뜨는 주소가 곧 Compute Box 다. 그 주소로 --host")
        print("  · Modbus 를 웹에서 꺼 뒀을 수도 있다")
        return 1
    best = next((f for f in found if f[3]), found[0])
    print(f"쓸 만한 것 — --host {best[0]} --unit {best[1]}")
    print("이 값으로 다시 읽어 보라 —")
    print(f"  python3 tools/gripper_status.py --host {best[0]} "
          f"--unit {best[1]} --watch")
    return 0


def show(regs):
    print("  원본 레지스터 —")
    for i, v in enumerate(regs):
        tag = GUESS.get(i)
        note = ""
        if tag:
            name, unit, div = tag
            note = f"   <- {name}"
            if div:
                note += f" {v / div:.1f}{unit}"
            elif v:
                note += "  ●"
        print(f"    [{i:2d}] 주소 {BASE + i}  = {v:6d}{note}")
    width = regs[9] / 10.0 if len(regs) > 9 else None
    grip = regs[11] if len(regs) > 11 else None
    busy = regs[10] if len(regs) > 10 else None
    print()
    print(f"  추정 폭 {width:.1f}mm   파지 {'물었다' if grip else '빈손'}"
          f"   {'동작 중' if busy else '멈춤'}")
    return {"width_mm": width, "grip": grip, "busy": busy}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.1", help="Compute Box 주소")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--unit", type=int, default=65, help="퀵 체인저 주소")
    ap.add_argument("--watch", action="store_true", help="계속 읽는다")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--save", action="store_true", help="읽은 값을 기록으로 남긴다")
    ap.add_argument("--scan", action="store_true",
                    help="흔한 주소 조합을 훑어 어디가 답하는지 찾는다")
    args = ap.parse_args()

    if args.scan:
        return scan(args.host, args.port)

    print()
    print("=" * 62)
    print("그리퍼 상태 읽기")
    print("=" * 62)
    print(f"  {args.host}:{args.port}  퀵체인저 {args.unit}")
    print("  읽기만 한다. 그리퍼를 움직이지 않는다.")
    print()

    def once():
        try:
            return read_registers(args.host, args.port, args.unit, BASE, COUNT)
        except (OSError, IOError) as e:
            print(f"  못 읽는다: {e}")
            print("  · Compute Box 가 이 PC 에서 보이나? ping 192.168.1.1")
            print("  · 로봇 컨트롤러를 거쳐야 하는 망이면 라우팅이 필요하다")
            return None

    samples = []
    try:
        if args.watch:
            print("여닫으면서 어느 칸이 변하는지 보라. Ctrl+C 로 끝낸다.\n")
            while True:
                regs = once()
                if regs is None:
                    return 1
                samples.append(show(regs))
                print()
                time.sleep(args.period)
        else:
            regs = once()
            if regs is None:
                return 1
            samples.append(show(regs))
            if args.save:
                from tunio_robot import runlog
                path = runlog.save("gripper_status",
                                   {"tool": "gripper_status", "host": args.host,
                                    "registers": regs, "decoded": samples[-1]},
                                   LOG_DIR)
                if path:
                    print(f"\n  기록 저장: {os.path.relpath(path)}")
    except KeyboardInterrupt:
        print("\n끝냈다.")
        if len(samples) > 1:
            widths = [s["width_mm"] for s in samples if s["width_mm"] is not None]
            if widths:
                print(f"  본 폭 범위 {min(widths):.1f} ~ {max(widths):.1f} mm")
            grips = {s["grip"] for s in samples}
            print(f"  파지 신호가 가진 값 {sorted(grips)}"
                  f"{'  — 안 변했다. 이 칸이 아닐 수 있다' if len(grips) < 2 else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
