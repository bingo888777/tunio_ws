# 페그 실측 — PC 한 대로 전부 돌리기

내 PC를 로봇에 **유선으로 직접 물려서** 두산 브링업 · `robot_controller` · 마이크 측정을
**혼자 다 띄우는** 절차다. 2026-09-21 에 실제로 성공한 구성을 그대로 적었다.

관련 문서 — 손으로 돌리는 절차는 `페그_캘리브레이션_가이드.md`,
로봇 앞에서 할 점검은 `로봇점검_체크리스트.md`. 이 문서는 **실행 환경** 쪽이다.

> ⚠️ **PC 한 대로 돈다고 해서 원격으로 혼자 하는 게 아니다.** 팔이 움직이고
> 사람이 줄을 뜯어야 하므로 **로봇 옆에 있어야 한다.** 비상정지를 손 닿는 곳에.

---

## 0. 왜 이 구성인가

원래는 3 PC 분산이다(서버·로봇·마이크). 그런데 페그 실측은

- 로봇이 쓰는 좌표가 **페그 4개뿐**이고 전부 `GripperDA_v1` 기준이다
- 발현은 **사람이 손으로** 한다
- 측정은 **내 마이크**로 한다

이라서 **TCP 기준이 하나로 통일**돼 있고, 다른 PC가 할 일이 없다.
로봇 PC 담당자를 기다릴 필요 없이 혼자 진행할 수 있다.

**로봇 PC 담당자에게는 한 가지만 알리면 된다** — "제가 유선으로 직접 붙습니다.
그쪽에서 브링업 띄우지 말아주세요." 두산 컨트롤러는 클라이언트를 하나만 받는다.

---

## 1. 확인된 환경 (2026-09-21)

### 네트워크

```
내 PC 유선   enx00e04c680085   192.168.1.103/24   (USB 랜 어댑터)
두산 컨트롤러                   192.168.1.100      포트 12345
그리퍼 Compute Box              192.168.1.1        502(Modbus) · 80(WebLogic)
```

`192.168.1.1` 의 MAC 이 `b8:27:eb:*` 다 — Raspberry Pi 계열. OnRobot Compute Box 가 맞다.

**Compute Box 의 Modbus 502 가 내 PC 에서 열려 있다.** PR #28 이 "로봇 제어 PC 에서
보이는지 확인 대상"으로 남긴 미결 항목인데, 유선으로 붙으면 보인다.
`tools/gripper_status.py` 를 이 PC 에서 바로 돌릴 수 있다.

### 로봇 상태

```
robot_mode  1 = AUTONOMOUS
tcp         'GripperDA_v1'     ← 펜던트에서 켜져 있음
tool        'Tool Weight'      ← 펜던트에서 켜져 있음
```

**코드가 `set_tool`/`set_tcp` 를 시도하고 `get` 으로 판정한다** (CLAUDE.md 3.5.1).
배포용이라 펜던트 설정에 기대지 않고, 이미 맞아 보여도 `set` 을 생략하지 않는다.
다만 **이 컨트롤러는 `set` 을 `-1` 로 거부하므로**(이슈 #23) 실제로는 펜던트에
켜져 있어야 `get` 이 맞아 통과한다.

이름이 정확히 일치해야 `robot_controller` 가 출발한다 — `_ensure_named` 가 `==` 로
비교하므로 공백·대소문자 하나도 틀리면 안 된다.

### 마이크

```
USB Audio Device   ALSA card 2   ← 이게 우리 마이크
PipeWire 기본 입력 = 이 장치      ← 그래서 sounddevice 의 `default` 가 USB 다
하드웨어 게인       Capture 16/16 = 100% = 23.81dB (이미 최대, 더 못 올림)
```

---

## 2. 시작 절차

### 터미널 1 · 두산 브링업

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```

`mode` 기본값이 `virtual`, `host` 기본값이 `127.0.0.1` 이다.
**둘 다 반드시 넘겨야 한다.** 안 주면 가짜로 떠서 팔이 안 움직인다.

### 터미널 2 · 확인 → `robot_controller`

**먼저 확인부터.** 로봇을 전혀 움직이지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 service list | grep -c dsr01        # 220 개쯤 나와야 한다
```

그 다음 TCP/Tool 확인 — **아래 스크립트를 쓴다.** `DSR_ROBOT2` 를 그냥 부르면
디스커버리가 안 끝나 **영원히 멈추므로**(이슈 #21), 먼저 기다려야 한다.

```bash
python3 - <<'EOF'
import time, rclpy, DR_init
DR_init.__dsr__id, DR_init.__dsr__model = "dsr01", "m0609"
rclpy.init()
node = rclpy.create_node("tcp_probe", namespace="dsr01")
DR_init.__dsr__node = node

# 반드시 DSR 이 쓰는 그 노드로 기다려야 한다 (다른 노드로 기다리면 소용없다)
import dsr_msgs2.srv as srv
for tn, name in [("GetCurrentTool", "dsr_controller2/tool/get_current_tool"),
                 ("GetCurrentTcp",  "dsr_controller2/tcp/get_current_tcp"),
                 ("GetCurrentPosj", "dsr_controller2/aux_control/get_current_posj")]:
    cli = node.create_client(getattr(srv, tn), name)
    print("  %-50s %s" % (name, "보임" if cli.wait_for_service(timeout_sec=40.0) else "없음"))
    node.destroy_client(cli)
time.sleep(1.0)

import DSR_ROBOT2 as api
print("robot_mode =", api.get_robot_mode(), " (1=AUTONOMOUS)")
print("tcp   =", repr(api.get_tcp()))
print("tool  =", repr(api.get_tool()))
EOF
```

**나와야 하는 것:**

```
robot_mode = 1  (1=AUTONOMOUS)
tcp   = 'GripperDA_v1'
tool  = 'Tool Weight'
```

빈 문자열(`''`)이나 다른 이름이면 **펜던트에서 켜야 한다.**
코드가 켜려 하지만 이 컨트롤러가 거부한다(#23).

확인됐으면 `robot_controller`:

```bash
source ~/tunio_ws/install/setup.bash
ros2 run tunio_robot robot_controller --ros-args -p dry_run:=false
```

**이렇게 떠야 정상이다:**

```
서비스 디스커버리 확인 (5개)
tool='Tool Weight' 확인 (set 반환 -1, 직전 'Tool Weight')
tcp='GripperDA_v1' 확인 (set 반환 -1, 직전 'GripperDA_v1')
두산 API 연결: dsr01 / m0609 tool='Tool Weight' tcp='GripperDA_v1'
```

**`set 반환 -1` 이 정상이다.** 이 컨트롤러가 설정 변경을 거부하는 것이고(#23),
판정은 반환값이 아니라 `get` 결과로 한다. `직전` 이 이미 원하는 이름이면
펜던트에 켜져 있었다는 뜻이다. 셋째 줄까지 나오면 출발한 것이다.

### 터미널 3 · 측정

```bash
cd ~/tunio_ws
source /opt/ros/jazzy/setup.bash && source ~/tunio_ws/install/setup.bash
```

---

## 3. 살아 있는지 확인하는 한 줄

```bash
ros2 action list | grep /robot/
```

`play_string` · `adjust_peg` · `strum_all` **셋 다** 나와야 한다.
하나만 나오면 `robot_controller` 가 죽고 **디스커버리 잔상만 남은 것**이다.

```bash
ros2 node list | grep -E "robot_controller|tunio_dsr"
```

- `/robot_controller` — 우리 노드
- `/dsr01/tunio_dsr` — `connect()` 가 두산 API 에 붙으면서 만든 노드

**둘 다 있어야** 실제로 로봇에 붙은 것이다.

---

## 4. 복구 절차

### 랜 케이블이 빠졌다 / 뽑았다 다시 꽂았다

**브링업은 대개 살아남고 `robot_controller` 만 죽는다.** (2026-09-21 실제 사례)

```bash
# 1) 네트워크부터
ping -c1 192.168.1.100
timeout 3 bash -c "echo > /dev/tcp/192.168.1.100/12345" && echo "포트 열림"

# 2) 브링업이 살아있나
ros2 node list | grep dsr01 | wc -l         # 10 개쯤

# 3) 로봇이 실제로 붙어있나 — 2절의 확인 스크립트를 다시 돌린다
#    robot_mode=1, tcp/tool 이 그대로면 브링업은 멀쩡하다

# 4) robot_controller 만 다시 띄운다 (터미널 2)
ros2 run tunio_robot robot_controller --ros-args -p dry_run:=false
```

**브링업까지 죽었으면** 터미널 1 부터 순서대로 다시.

### `robot_controller` 가 `connect()` 에서 멈춘다

디스커버리가 안 끝난 것이다. **최소 30초는 기다려라.**
3초에 포기했다가 틀린 적이 있고, 10초 넘게 걸린 적도 있다.

30초 넘게 안 되면 디스커버리 서버 탓인지 가른다:

```bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
# 다시 시도
```

- **껐더니 붙는다** → 원인은 디스커버리 서버. 로봇·페그는 멀쩡
- **꺼도 안 붙는다** → 원인은 로봇/브링업 쪽. 통신 문제 아님

> `unset` 은 **설정이 아니라 진단 도구다.** 평소엔 켜고 쓴다 —
> 통합할 때 어차피 디스커버리 서버를 거치므로, 같은 조건에서 검증해두는 게 이득이다.
> 2026-09-21 실측에서는 **켠 채로 잘 붙었다** (서비스 220개, 기다리니 1.0초).

### `tcp 를 'GripperDA_v1' 로 못 맞췄다` 로 멈춘다

로그 문구는 `motion_primitives.py` 의 `_ensure_named` 가 낸다.

```
tcp 를 'GripperDA_v1' 로 못 맞췄다 (set 반환 -1, 현재 ''). 펜던트에 그 이름이
등록돼 있는지, 설정 변경 제어권이 로봇에 있는지 확인하라.
```

**가드가 제대로 일한 것이다.** 펜던트의 TCP/Tool 이 YAML 과 다르다는 뜻.
펜던트에서 `GripperDA_v1` / `Tool Weight` 를 켜면 된다. 코드는 고치지 말 것.

---

## 5. 측정 절차

### 페그를 직접 돌린다 (목표 근처로 맞출 때)

```bash
# 풀기 (음 낮추기)
ros2 action send_goal /robot/adjust_peg tunio_interfaces/action/AdjustPeg \
  "{string_number: 6, direction: -1, angle_deg: 80.0}"

# 조이기 (음 높이기)
ros2 action send_goal /robot/adjust_peg tunio_interfaces/action/AdjustPeg \
  "{string_number: 6, direction: 1, angle_deg: 10.0}"
```

`direction` — **`+1` 조임(음 높이기), `-1` 풀림(음 낮추기).** `angle_deg` 는 항상 양수.

**시간이 오래 걸린다.** `turn_vel: 2.0 deg/s` 라 회전만 `각도 ÷ 2` 초다.
80도면 40초, 여기에 접근·파지·놓기·후퇴가 붙어 **1분쯤**. 안 끝난 것처럼 보여도 기다려라.

### 지금 음 확인

```bash
python3 tools/pitch_test.py --string 6
```

### 캘리브레이션 도구

```bash
# 흐름만 — 팔은 진짜 움직이고 소리만 가짜
python3 tools/peg_cal.py --string 6 --min-angle --robot --simulate 0.2

# 백래시
python3 tools/peg_cal.py --string 6 --backlash --robot \
    --save docs/peg_data/$(date +%Y-%m-%d)_peg6_backlash.json

# 최소 유효 각도
python3 tools/peg_cal.py --string 6 --min-angle --robot \
    --save docs/peg_data/$(date +%Y-%m-%d)_peg6_minangle.json
```

**`--device` 는 붙이지 마라.** 기본값이 `default` 이고 그게 USB 마이크다.
PipeWire 가 장치를 잡았다 놨다 하면서 **번호가 계속 밀린다**(6절 참조).

타임아웃이 나면 `--robot-timeout 180`. 기본 60초는 `turn_vel 2.0` 에서 아슬아슬하다.

---

## 6. 함정 모음 — 전부 2026-09-21 에 실제로 겪은 것

### `DSR_ROBOT2` 를 바로 부르면 영원히 멈춘다

`set_tcp` 등이 `spin_until_future_complete` 를 **타임아웃 없이** 부른다.
디스커버리가 안 끝났으면 영영 안 돌아온다. **먼저 `wait_for_service` 로 기다려라.**
반드시 **DSR 이 쓰는 그 노드**로 기다려야 한다 — 다른 노드로 기다리면 그 노드만
디스커버리가 끝나고 DSR 노드는 여전히 못 찾는다.

기다려주면 **1.0초**에 붙는다. 안 기다리면 **90초에도 안 붙는다.**

### `-1` 은 에러 코드가 아니다

`set_tcp` 가 `-1` 을 줘도 **이유는 어디에도 없다.** 층을 따라가면 이렇다.

```
DSR_ROBOT2.py       ret = 0 if result.success else -1
dsr_msgs2/srv/      SetCurrentTcp.srv 의 응답이 `bool success` 하나뿐
                    → 실패 이유를 담을 필드가 아예 없다
dsr_controller2.cpp res->success = Drfl->set_tcp(req->name);  (로그도 주석 처리)
libDRFL.a           닫힌 바이너리. 진짜 이유는 여기만 안다
```

**ROS 쪽은 더 팔 게 없다.** 원인은 두산 API 문서와 펜던트 쪽에 있다.

### `get_tcp()` 는 실패를 구분하지 못한다

`get_current_tcp_cb` 가 `res->success = true` 를 **하드코딩**한다.
그래서 `''` 가 나와도 "TCP 가 안 켜짐"인지 "못 읽음"인지 알 수 없다.
**펜던트 화면을 눈으로 확인하는 게 확실하다.**

### 마이크 장치 번호가 계속 밀린다

PipeWire 가 `hw:2,0` 을 배타적으로 잡으면 PortAudio 목록에서 **USB 장치가 사라지고**,
뒤 번호가 한 칸씩 당겨진다. `--device 14` 가 `IndexError` 를 낸 원인이 이것이다.

```bash
# 지금 누가 잡고 있나
fuser -v /dev/snd/pcmC2D0c

# 카드는 멀쩡한가
cat /proc/asound/cards
arecord -l
```

**해결은 `--device` 를 안 쓰는 것이다.** `default` 가 PipeWire 를 거쳐 같은 USB
마이크로 간다. 직행(`hw:2,0`)이 막혀도 소리는 똑같이 온다.

### AGC 가 켜져 있으면 측정이 흔들린다

USB 마이크에 하드웨어 AGC 가 있다. 켜져 있으면 뜯는 세기에 따라 게인이 제멋대로
바뀌어 **같은 조건을 반복할 수 없다.** CLAUDE.md 3.5 위반이다.

```bash
amixer -c 2 sset 'Auto Gain Control' off
amixer -c 2 sget 'Auto Gain Control' | tail -1      # [off] 확인
```

**끄면 레벨이 20dB 쯤 떨어진다.** 그게 진짜 값이다. AGC 가 부풀리던 건 가짜다.

하드웨어 게인은 이미 최대(`Capture 16/16 = 23.81dB`)라 더 못 올린다.
모자라면 **마이크를 기타에 더 가까이** (거리 절반 = +6dB) 하거나 더 세게 뜯어라.

### RMS 가 낮아도 괜찮다 — 판정은 **돌출**이다

```python
MIN_PROMINENCE_DB = 20.0        # pitch.py:52
```

`-35 dBFS` 여도 **돌출이 +55dB** 면 통과한다. 방 소음 RMS 가 실제 연주 RMS 와
완전히 겹쳐서 RMS 로는 못 가르기 때문에 돌출을 쓴다(`pitch.py` 주석).

**`-18 dBFS` 는 로봇이 튕길 때 얘기다**(이슈 #6). 로봇이 돌면 배경이 24dB 올라가
돌출이 3~8dB 깎인다. 사람이 손으로 뜯고 로봇이 멈춰 있으면 해당 없다.

### 저음현은 최대 피크가 기본파에 안 선다

6번 줄(82Hz)은 **2배음이 기본파보다 +12dB 세다.** `mic_level.py` 는 범위 제한 없이
최대 피크만 찍으므로 165~180Hz(2배음)나 270Hz(3배음)가 나온다. **정상이다.**

`pitch.py` 는 탐색 범위를 **목표 ±2반음**으로 묶어서 배음에 안 속는다.

### `mic_level.py` 의 장치별 RMS 는 서로 비교하면 안 된다

장치를 **하나씩 순서대로** 녹음한다. 기타는 뜯고 나서 계속 감쇠하므로
`[11]` 과 `[14]` 는 **다른 순간의 다른 뜯기**를 잰 것이다.
게인을 볼 때는 `--loop` 으로 **한 장치만 반복**해서 봐라.

### ALSA `paInvalidSampleRate` 도배는 무시해도 된다

44100 → 장치 기본값 → 48000 순으로 재시도하는 과정이다. 결과에 영향 없다.

---

## 7. 측정할 때 지킬 것

- **항상 조이는 방향으로 마무리.** 방향을 바꾸면 백래시로 헛돈다 (CLAUDE.md 3.2)
- **목표를 넘어가도 되돌려 풀지 마라.** 그 지점부터 백래시가 또 끼어 데이터가 오염된다
- **로봇이 완전히 멈춘 뒤에 뜯어라.** 구동음이 20dB 관문을 흐린다
- **손 뗀 뒤 3초 대기.** 도구가 자동으로 기다린다
- 6번 줄은 **녹음 2.0초**가 필요하다. 뜯고 바로 줄에 손대지 말 것
- **Phase 0(페그 초기 등록)이 아직 미구현이다.** 로봇이 손잡이 방향을 스스로 못 맞춘다.
  **지금 서 있는 각도로 그냥 문다.** 시작 전에 눈으로 방향을 확인할 것

---

## 8. 2026-09-21 실측 기록

### 각도-센트 (6번 줄, 로봇 파지 21mm, turn_vel 2.0 deg/s)

| 동작 | 전 | 후 | 센트 | 센트/도 |
|---|---|---|---|---|
| 풀기 80도 | 89.601 Hz | 81.324 Hz | -167.8 | **2.098** |

`docs/페그_실측표.md` 의 6번 줄이 전 구간 2.025 / 목표 ±30센트 2.219 인데
**2.098 이 그 사이에 들어와 일관된다.** 게다가 이 구간(+145센트대)은
고장력 구간이라 지금까지 데이터가 없었다. **표본 보강에 해당한다.**

### 확인된 사실

- 펜던트 TCP/Tool 이 **맞게 켜져 있다.** `set_tcp` 를 안 고쳐도 진행 가능하다는 실증
- 디스커버리 서버를 **켠 채로** 잘 붙는다 (서비스 220개, 대기 1.0초)
- `wait_for_service` 를 넣으면 90초 행이 **1.0초**로 바뀐다 — PR #28 의 `_wait_services` 가 유효함
- Compute Box 의 Modbus 502 가 **내 PC 에서 보인다** — PR #28 의 미결 항목 해소

### 아직 안 잰 것

- 백래시 (`--backlash`)
- 최소 유효 각도 (`--min-angle`) — 이슈 #8 의 빈 칸
- 5 · 2 · 1 번 줄
