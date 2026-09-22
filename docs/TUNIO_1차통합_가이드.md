# TUNIO 1차 통합 가이드 — PC 2대

> 2026-09-22. 구성은 `docs/TUNIO_시스템아키텍처_v3.md`.
> 이 문서는 **그날 순서대로 칠 것**만 담는다.

## 절반은 이미 끝나 있다

`docs/통합_1차_서버DB관제.md` (2026-09-21, 검증 정정인) 가 **로봇을 뺀 절반**을
이미 통과시켰다. 이 문서는 **거기에 로봇을 붙이는 절차**다. 겹치는 내용은 안 쓴다.

거기서 이미 확인된 것 —

- **인터페이스 정의 13/13** 이 명세와 일치. 서비스 8 · Action 4 · 토픽 1 이 명세
  이름·QoS 그대로 뜬다
- **조율 전 과정이 두 구성에서 완주**하고 DB 저장·이력 조회까지 통과
- **통신 지연이 예산 안에 든다** — 원격 서비스 왕복 중앙 7.4ms, Action 왕복 추정
  16ms(최악 30ms). 응답 목표 2.20초에 6번 줄 실측 2.05초라 **여유 150ms** 다.
  PC 2대가 되면서 마이크↔서버 왕복은 아예 없어졌으니 더 유리하다
- ⚠ **ping 으로 지연을 재면 안 된다.** 같은 시각 ping 이 7배 나쁘게 나온다
  (무선 절전 + AP 의 ICMP 후순위). `tools/ros_latency.py` · `action_latency.py` 를 쓴다
- ⚠ **디스커버리에 15초쯤 걸린다.** 런치 직후 `ros2 service list` 가 비어 보인다고
  실패로 판단하지 말 것

그 문서의 §6(로봇이 빠진 이유 — TCP #23·#26)과 §8 의 캘리브레이션 항목은
**2026-09-22 에 해소됐다.** 그래서 이번에 로봇을 붙인다.

## 0. 이 회차에서 어디까지 갈 수 있나 — 먼저 읽을 것

**페그 조절은 아직 못 한다.** `peg_calibration.yaml` 의 `measured` 가 `false` 라
task_manager 가 **조절이 필요해지는 순간 `PHASE_ERROR` 로 멈춘다.** 실측 전에
지어낸 각도로 페그를 돌리지 않으려고 일부러 막아 둔 것이다 (CLAUDE.md 11절).

그래서 **조절을 안 거치고 한 바퀴를 도는 것**을 이번 목표로 잡는다.

> **사람이 네 줄을 미리 허용 센트(10센트) 안으로 맞춰 두면 조절이 안 일어난다.**
> 그러면 캘리브레이션 없이도 완주한다 —
> 시작 → 피크 집기 → 발현 → 측정 → 판정(통과) → 다음 줄 → 검증 패스 →
> 화음 연출 → 대기 복귀 → DB 저장.

이 한 바퀴가 돌면 **배선은 전부 검증된 것**이다. 남는 것은 페그 조절 하나뿐이다.

```
A  마른 시험    dry_run=true    노드·상태·웹·DB 배선만
B  젖은 시험    dry_run=false   조절 없이 완주  ← 이번 목표
C  조절 포함    캘리브레이션 끝난 뒤              ← 다음
```

**C 로 가기 전에 막혀 있는 것 두 개**

- **1번 E4 백래시·최소 유효 각도 미측정**, 5번은 `cents_per_deg` 불일치로 재측정
  필요 (#8). `docs/페그실측_줄별_명령어.md`
- **#70 손목 감김** — 페그를 한 방향으로 여러 번 돌리면 J6 가 ±360도 한계까지
  감겨 `movel` 이 거부된다. **조율 루프도 같은 경로다** — 줄당 최대 5회 ×
  `max_angle_deg` 60도면 한 방향 300도다. C 에서 만난다

---

## 1. 준비 — 양쪽 PC 에서 각각

### 1.1 빌드 (⚠ 이걸 빼먹어서 두 시간을 썼다)

```bash
cd ~/tunio_ws
git pull
colcon build
source install/setup.bash
```

`robot_controller` 는 소스가 아니라 **설치본**을 읽는다. 2026-09-21 에 로봇 PC
빌드를 빼먹어 설치본의 `pegs.*.sol` 이 `null` 로 남았고, 5번은 허공을 헤매고
2번은 빈손 16.2mm 가 났다.

**로봇 PC 에서 확인:**

```bash
python3 -c "
import yaml
d=yaml.safe_load(open('install/tunio_robot/share/tunio_robot/config/teach_points.yaml'))
print({n: d['pegs'][n]['sol'] for n in (6,5,2,1)})
"
```

`{6: 7, 5: 7, 2: 2, 1: 2}` 가 나와야 한다. `None` 이 있으면 빌드가 안 먹었다.

### 1.2 환경 변수 — 양쪽 다

```bash
export ROS_DOMAIN_ID=10
```

**두 PC 가 같아야 한다.** 다르면 서로 안 보인다.

### 1.3 마이크 (정정인 PC)

OS 의 **노이즈 제거·자동 음량(AGC)·에코 제거를 전부 끈다.** 하나라도 켜져 있으면
측정이 조용히 틀어진다 (CLAUDE.md 3.5).

### 1.4 기타·페그 (로봇 PC 쪽 작업대)

- **페그 손잡이 4개를 손으로 눕혀 둔다.** 열기 폭 38mm 로는 선 손잡이를 못 지난다
- **피크가 거치대에 있는지** 확인한다. 놓친 피크는 비전이 없어 못 찾는다
- **네 줄(6·5·2·1)을 미리 맞춰 둔다** — 0절대로 이번 회차는 조절을 안 한다.
  10센트 안으로. 튜너 앱이든 `tools/pitch_test.py` 든 상관없다

```bash
python3 tools/pitch_test.py --string 6      # 줄마다 확인
```

---

## 2. 띄우기 — 순서대로

### 터미널 R1 · 로봇 PC — 두산 브링업

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash

ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100
```

`mode` 와 `host` **둘 다 반드시 넘긴다.** 기본값이 `virtual` / `127.0.0.1` 이라
안 주면 가짜로 떠서 팔이 안 움직인다.

### 터미널 R2 · 로봇 PC — robot_controller

먼저 브링업이 붙었는지 본다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 service list | grep -c dsr01        # 220 개쯤
```

**A 단계(마른 시험)** — 기본값이 `dry_run:=true` 라 인자 없이 띄운다.

```bash
source ~/tunio_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch tunio_robot robot.launch.py
```

**B 단계(젖은 시험)** — 실제로 움직인다.

```bash
ros2 launch tunio_robot robot.launch.py dry_run:=false
```

이 세 줄이 나와야 출발한 것이다.

```
tool='Tool Weight' 확인 (set 반환 -1, 직전 'Tool Weight')
tcp='GripperDA_v1' 확인 (set 반환 -1, 직전 'GripperDA_v1')
두산 API 연결: dsr01 / m0609 tool='Tool Weight' tcp='GripperDA_v1'
```

**`set 반환 -1` 은 정상이다.** 이 컨트롤러가 설정 변경을 거부하는 것이고(#23),
판정은 반환값이 아니라 `get` 결과로 한다.

`connect()` 에서 멈추면 **최소 30초는 기다린다.** 디스커버리가 10초 넘게 걸린
적이 있다.

ℹ **`gripper:=rg2` 는 주지 않는다.** 노드가 그 파라미터를 선언하지도 읽지도
않아 아무 일도 하지 않는다. 그리퍼 한계·프리셋은 `teach_points.yaml` 의
`gripper` 절에서 온다.

### 터미널 S1 · 정정인 PC — 마이크 + 서버 + 관제 웹

```bash
source /opt/ros/jazzy/setup.bash && source ~/tunio_ws/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch tunio_bringup station.launch.py
```

옵션 — `device:=2`(마이크 장치) · `port:=9000` · `open_browser:=false` · `db:=<경로>`.

---

## 3. 붙었는지 확인 — 정정인 PC 에서

```bash
ros2 node list
```

다섯 개가 다 나와야 한다.

```
/task_manager  /result_store  /web_node  /audio_analyzer  /robot_controller
```

```bash
ros2 action list | grep -E "/robot/|/audio/"
```

**넷 다** 나와야 한다 — `play_string` · `adjust_peg` · `strum_all` · `measure_pitch`.

`/robot/*` 가 하나도 안 보이면 **두 PC 가 서로를 못 보는 것**이다 → 6절.
하나만 보이면 `robot_controller` 가 죽고 디스커버리 잔상만 남은 것이다.

```bash
ros2 topic echo /tuning/state --once
```

**늦게 켜도 마지막 상태가 온다** (transient local). 안 오면 task_manager 가 없다.

브라우저에서 **`http://localhost:8080`** — 진행 상황·설정·기록 **세 곳이 다**
채워지는지 본다. 기록만 뜨면 ROS 쪽이 안 붙은 것이다.

---

## 4. A 단계 · 마른 시험 (로봇 안 움직임)

`dry_run:=true` 인 채로 관제 웹에서 **시작**을 누른다.

**보는 것은 배선뿐이다.** 로봇이 안 튕기니 마이크에 소리가 안 들어오고, 측정은
재시도를 소진한 뒤 그 줄을 실패로 넘긴다. **그게 정상이다.**

확인할 것 —

- [ ] 웹의 **시작** 버튼이 먹고 상태가 `대기` 에서 바뀌는가
- [ ] 단계 줄이 6 → 5 → 2 → 1 순서로 넘어가는가
- [ ] **일시정지 · 재개 · 취소**가 먹는가. 작업 중 설정 변경이 거부되는가
- [ ] 끝나고 **기록 탭에 job 이 생기는가**

> **측정까지 진짜로 보려면 두 가지 길이 있다.**
>
> 하나는 **사람이 큐에 맞춰 줄을 뜯는 것.** 로봇은 안 움직이고 마이크만 진짜라
> `sim_robot.launch.py` 와 같은 조건이 된다.
>
> 다른 하나는 **녹음을 재생하는 것** — 기타도 사람도 필요 없다.
>
> ```bash
> ros2 param set /audio_analyzer replay_wav "$PWD/tools/g{string_number}_*.wav"
> ```
>
> `replay_wav` 는 goal 마다 읽히는 값이라 **노드를 띄운 뒤 런타임에 바꿔도 먹는다.**
> `{string_number}` 는 줄 번호로 치환되고 와일드카드는 회차마다 다른 파일로 돈다.
> 녹음 파일은 `.gitignore` 의 `*.wav` 대상이라 저장소에 없다 —
> `docs/통합_1차_서버DB관제.md` §4·§9 참조.

---

## 5. B 단계 · 젖은 시험 (이번 회차 목표)

⚠ **비상정지에 손이 닿는 자리에 선다.** 처음 도는 조합이다.

로봇 PC 의 R2 를 `dry_run:=false` 로 다시 띄우고 웹에서 **시작**.

### 순서대로 이게 나와야 한다

```
피크 집기 (거치대)  →  발현 자세  →  6번 튕김  →  측정  →  판정 통과
  →  피크 반환  →  5번 …  →  2번 …  →  1번 …
  →  검증 패스 (조절 없이 측정만)  →  화음 연출  →  대기 자세 복귀  →  저장
```

### 확인할 것

- [ ] **피크를 제대로 집는가.** 놓치면 비전이 없어 못 찾는다 — 바로 중단
- [ ] **발현이 측정을 통과하는가.** 돌출 20dB 를 넘어야 한다.
      `FAIL_LOW_SIGNAL` 이 잦으면 **로봇 타현이 약한 것**이다 (이슈 #32)
- [ ] **로봇 구동음이 측정을 오염시키지 않는가.** 회전이 끝난 뒤 재는 구조지만,
      PC 배치가 바뀌어 **마이크 위치도 바뀌었다.** 돌출 문턱 20dB 근거
      (`docs/로봇구동음_측정결과.md`)가 그대로인지 본다
- [ ] **발현 자세와 페그 자세를 오갈 때 팔이 꼬이거나 기타를 쓸지 않는가**
      (CLAUDE.md 13절 미검증 항목)
- [ ] **`PHASE_ERROR` 가 나는가.** 나면 그 줄이 10센트 밖이라 조절이 필요해진
      것이다. 그 줄을 사람이 다시 맞추고 재시작한다

### 같이 재 둘 것 — 오늘이 사이클 타임 실측일이다

CLAUDE.md 에 **9/22 사이클 타임 실측**이 잡혀 있다. 완주한 job 에서 뽑는다.

```bash
python3 - <<'EOF'
import sqlite3, datetime as dt
c = sqlite3.connect("file:/home/rokey/.tunio/tunio.db?mode=ro", uri=True)
for jid, fin, res in c.execute(
        "select job_id, finished_at, overall_result from jobs order by job_id desc limit 5"):
    if not fin:
        print(jid, "미완료"); continue
    t0 = dt.datetime.strptime(jid[:15], "%Y%m%d_%H%M%S")
    t1 = dt.datetime.fromisoformat(fin.replace("Z", "+00:00")).replace(tzinfo=None)
    print(f"{jid}  {(t1 - t0).total_seconds():7.1f}초  결과 {res}")
EOF
```

`job_id` 가 시작 시각이고 `finished_at` 이 끝난 시각이다. **줄별 시간은 DB 에
안 남으므로** 필요하면 `task_manager` 터미널 로그의 타임스탬프를 쓴다.

**측정 응답 시간도 본다.** 목표 2.2초다. PC 2대가 되면서 `/audio/measure_pitch`
가 한 PC 안으로 들어와 Wi-Fi 왕복이 없어졌는데, 대신 정정인 PC 한 대에서 노드
4개가 돈다. **넘으면 그게 이번 구성의 대가다.**

### 끝나고

```bash
python3 - <<'EOF'
import sqlite3
c = sqlite3.connect("file:/home/rokey/.tunio/tunio.db?mode=ro", uri=True)
jid = c.execute("select job_id from jobs order by job_id desc limit 1").fetchone()[0]
print("job", jid)
for r in c.execute(
        "select string_number, target_hz, tuned_hz, verified_hz, final_error_cents,"
        " adjust_count, result, failure_reason from string_results"
        " where job_id=? order by id", (jid,)):
    print(" ", r)
EOF
```

`adjust_count` 가 **전부 0** 이어야 이번 회차 전제(조절 없음)가 지켜진 것이다.

---

## 6. 두 PC 가 서로를 못 볼 때

같은 `ROS_DOMAIN_ID` 를 export 했는지부터 본다. 그래도 안 보이면 Wi-Fi 가
멀티캐스트를 막는 것이다. Fast DDS Discovery Server 를 쓴다 —
**서버는 정정인 PC 에서 띄운다** (예전에는 서버 PC 였다).

```bash
# 정정인 PC
fastdds discovery -i 0 -l 0.0.0.0 -p 11811
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# 로봇 제어 PC
export ROS_DISCOVERY_SERVER=<정정인 PC IP>:11811
```

**클라이언트가 서버에 자기 주소를 등록하므로 고정 IP 가 필요한 것은 정정인 PC
하나뿐이다.** 노드를 띄운 **모든 터미널**에서 export 해야 한다.

⚠ `peg_cal.py` 처럼 **직접 두산 API 에 붙는 도구는 디스커버리 서버를 끄고** 써야
붙는 경우가 있다 — `unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT`.

---

## 7. 자주 나는 것

**`빈손 <폭>mm` / 팔이 하늘을 본다**
빌드를 안 했거나 `sol` 이 비었다(1.1절). 에러 문구의 "기타가 밀렸거나 교시 좌표가
어긋났다"는 **오진이다** (`docs/robot_data/README.md`). 기타를 손대지 말 것.
손잡이는 축에 대칭이라 어느 각도로도 폭이 19.6~32.8mm 다 — 그보다 작으면 빈손이다.

**`movel 가 거부됐다 (-1)`**
손목이 감긴 것이다(#70). `ros2 topic echo /dsr01/joint_states --once --field position`
으로 J6 를 본다. ±360도에 가까우면 펜던트로 되감고 재등록한다.

**측정이 `돌출 부족` 으로 실패**
로봇 타현이 약하다. 이슈 #32 에 기록하고, 급하면 사람이 대신 뜯어 배선만 본다.

**`측정 불가` 만 뜬다**
마이크 향상 기능이 켜져 있거나 장치가 바뀌었다. `device:=<번호>` 로 지정한다.

**웹에 기록만 뜨고 진행 상황·설정이 비어 있다**
ROS 쪽이 안 붙었다. 3절을 다시 본다.

---

## 8. 끝나고 남길 것

- **완주 여부**와 실패한 줄·사유
- **사이클 타임**과 측정 응답 시간 (5절)
- **`adjust_count` 가 0 이었는지**
- 로봇 구동음이 측정을 오염시켰는지 → 이슈 #32
- 발현↔페그 자세 간섭이 있었는지 → CLAUDE.md 13절 미확정 항목
- `~/.tunio/robot_data/` 의 회차 로그 중 남길 만한 것은 `docs/robot_data/` 로

C 단계(조절 포함)는 **1번 측정과 5번 재측정이 끝나야** 연다.
`docs/페그실측_줄별_명령어.md`.
