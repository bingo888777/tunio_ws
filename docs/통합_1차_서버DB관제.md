# 1차 통합 결과 — 서버 · DB · 관제 웹 + 마이크

검증: 정정인 / 2026-09-21
대상: `tunio_interfaces` · `tunio_server` (task_manager · result_store · web_node) · `tunio_audio`
빠진 것: 실제 로봇 (TCP 문제로 보류 — 6절)

---

## 1. 결론

**서버 · DB · 관제 웹 · 마이크 네 가지가 인터페이스 명세대로 붙어서 전 구간이 돌았다.**

- 인터페이스 정의 **13/13** 이 명세와 일치
- 서비스 8개 · Action 4개 · 토픽 1개가 명세 이름·QoS 그대로 뜬다
- 조율 전 과정이 **두 가지 구성**에서 완주하고 DB 저장·이력 조회까지 통과
- 통신 지연은 응답 목표 예산 안에 들어온다

로봇만 붙이면 된다. **로봇을 뺀 부분은 통합이 끝났다고 봐도 된다.**

---

## 2. 인터페이스 정합성

### 2.1 정의 대조 — 13/13

기준본 `docs/TUNIO_INTERFACE_SPEC.md` 의 각 인터페이스 코드블록을 뽑아, 실제
`src/tunio_interfaces/` 파일과 **주석을 제거한 필드 단위**로 비교했다.

```
✓ action/AdjustPeg.action      (7줄 일치)     ✓ msg/TuningSettings.msg   (5줄 일치)
✓ action/MeasurePitch.action  (20줄 일치)     ✓ msg/TuningState.msg     (28줄 일치)
✓ action/PlayString.action     (5줄 일치)     ✓ srv/CommandTuning.srv    (3줄 일치)
✓ action/StrumAll.action       (5줄 일치)     ✓ srv/GetHistory.srv       (7줄 일치)
✓ msg/JobSummary.msg           (3줄 일치)     ✓ srv/GetSettings.srv      (2줄 일치)
✓ msg/StringResult.msg        (21줄 일치)     ✓ srv/SaveJob.srv          (8줄 일치)
                                              ✓ srv/SetSettings.srv      (4줄 일치)
결과: 13/13 일치
```

### 2.2 실제로 뜬 엔드포인트

```
서비스 8개   /tuning/start · pause · resume · cancel
             /tuning/get_settings · set_settings · get_history · save_job
Action 4개   /audio/measure_pitch
             /robot/play_string · adjust_peg · strum_all
토픽         /tuning/state
```

`/tuning/state` 의 QoS 는 **RELIABLE + TRANSIENT_LOCAL + depth 1** 로,
`task_manager` · `web_node` · `mock_state` 세 곳이 같은 값을 쓴다.

> **주의 — 디스커버리에 15초쯤 걸린다.** `ROS_DISCOVERY_SERVER` 가 걸려 있으면
> 같은 PC 안의 노드끼리도 서버를 거친다. 런치 직후 `ros2 service list` 가 비어
> 보인다고 실패로 판단하면 안 된다. `ros2 daemon stop` 후 15초 기다린다.

---

## 3. 1단계 — 전부 가짜 (`sim_all.launch.py`)

```bash
ros2 launch tunio_bringup sim_all.launch.py speed:=fast port:=9123 db:=<경로>
ros2 service call /tuning/start tunio_interfaces/srv/CommandTuning "{}"
```

```
6번 줄 페그 조임 280.85도 → 조율 성공 82.27Hz (-3.0센트, 조절 1회)
5번 줄 페그 조임 236.97 / 67.45도 → 조율 성공 109.88Hz (-2.0센트, 조절 2회)
2번 줄 페그 조임 117.89 / 60.59 / 33.52도 → 조율 성공 246.07Hz (-6.1센트, 조절 3회)
1번 줄 페그 조임 86.63 / 50.34 / 33.69도 → 조율 성공 327.96Hz (-8.8센트, 조절 3회)
작업 20260921_092814 저장 (줄 4개, 전체판정 1)
```

**조절 횟수가 늘수록 오차가 줄어드는 수렴 거동이 화면과 로그에 그대로 나타난다.**

> ⚠ 이때 쓰는 각도-센트 상수는 `peg_calibration.sim.yaml` 의 **지어낸 값**이다.
> 여기서 수렴한다고 실제로 수렴한다는 뜻이 아니다. 흐름만 본다.

---

## 4. 2단계 — 진짜 마이크 + 진짜 분석 (`sim_robot.launch.py`)

로봇만 가짜고 `audio_analyzer` 는 실제 노드다. **기타도 사람도 없이** 돌렸다 —
`replay_wav` 에 2026-09-19·20 녹음을 걸었다.

```bash
ros2 launch tunio_bringup sim_robot.launch.py port:=9124 db:=<경로>
ros2 param set /audio_analyzer replay_wav "<경로>/tools/g{string_number}_*.wav"
ros2 service call /tuning/start tunio_interfaces/srv/CommandTuning "{}"
```

`replay_wav` 는 goal 마다 읽히는 값이라 **노드를 띄운 뒤 런타임에 바꿔도 먹는다.**
`{string_number}` 는 줄 번호로 치환되고 와일드카드는 회차마다 다른 파일로 돈다.

```
6번 → g6_e2_1.wav   81.972Hz  -9.2센트  돌출 32.7dB  일치 2.59센트  성공
5번 → g5_a2_1.wav  109.906Hz  -1.5센트  돌출 44.1dB  일치 2.31센트  성공
2번 → g2_b3_1.wav  247.004Hz  +0.4센트  돌출 42.3dB  일치 0.74센트  성공
1번 → g1_e4_1.wav  326.502Hz -16.5센트  → 페그 41.23도 조임 → 재측정 …
1번 줄 검증에서 틀어짐 326.38Hz (-17.2센트)
작업 20260921_092953 저장 (줄 4개, 전체판정 2)
```

### 실패 경로도 같이 검증됐다

1번 줄 녹음이 실제로 **−17센트 틀어진 상태**라, 페그를 돌려도 재생 데이터는 음이
변하지 않는다. 그래서 의도치 않게 아래가 전부 확인됐다.

- 조절 5회 소진 → 그 줄 실패 판정
- 최종 검증 패스에서 틀어짐 감지 → 실패로 재기록
- 4줄 완주했으므로 **PARTIAL(전체판정 2)도 저장** — CLAUDE.md 13절 규칙대로

---

## 5. DB 와 조회

`result_store` 가 만든 SQLite 에 테이블 3개가 생긴다.

```
jobs               job_id · finished_at · overall_result
string_results     job_id · string_number · target_hz · tuned_hz · verified_hz
                   final_error_cents · adjust_count · result · failure_reason
settings_snapshot  (작업 시점 설정 보존)
```

`GetHistory` 두 모드가 명세 규칙대로 동작한다.

| 호출 | jobs | strings | settings |
|---|---|---|---|
| `job_id: ''` (목록) | 1건 | **빈다** | 빈 값 |
| `job_id: '2026…'` (상세) | **빈다** | 4건 | 채워짐 |

관제 웹은 `http://localhost:<port>` 에서 HTTP 200 으로 응답한다.

---

## 6. 로봇이 빠진 이유

**`robot_controller` 를 `dry_run:=false` 로 띄우면 안 되는 상태다.**

- 컨트롤러가 `set_tool` / `set_tcp` / `add_tcp` 를 전부 `-1` 로 거부한다 (#23)
- `motion_primitives.connect()` 가 그 반환값을 **버리고** "두산 API 연결" 을 찍는다 (#26)
- 그러면 TCP 가 안 켜진 채(플랜지 기준) `GripperDA_v1`(툴 Z 208mm) 기준으로 교시된
  페그 좌표를 명령해 **약 208mm 어긋난 곳으로 직선 진입**한다

2026-09-21 09:xx 기준 `get_current_tcp` / `get_current_tool` 둘 다 빈 문자열이다.
**펜던트에서 사람이 직접 켜는 것**이 1순위 해법이다 (#23 · #26).

`robot_controller` 는 `dry_run` 기본값으로는 정상 동작을 확인했다 — goal 23회
전부 처리, 회전 부호 변환(`direction(-1) × turn_sign(-1) = J6 +`)도 의도대로다.

---

## 7. 통신 지연 — 예산 안에 들어온다

도구: `tools/ros_latency.py` · `tools/action_latency.py`

```
서비스 왕복  원격(마이크 PC ↔ 로봇 PC)  중앙 7.4ms  p99 14.3  최대 14.3  실패 0/30
             로컬(같은 PC)              중앙 0.6ms
Action 왕복  로컬 robot_controller       중앙 2.4ms  최대 2.9
             원격 추정 = 7.4×2 + 처리 1.3 ≈ 16ms, 최악 약 30ms
```

```
응답 목표      2.20초
6번 줄 실측    2.05초
남은 여유      0.15초 = 150ms   ←  30ms 는 넉넉히 들어온다
```

> **ping 으로 재면 안 된다.** 같은 시각 ping 은 최악 106~153ms 로 7배 나쁘게 나온다.
> ping 은 간격이 넓어 그 사이 무선이 절전으로 들어가고, AP 가 ICMP 를 후순위로
> 돌린다. **CLI 로도 못 잰다** — `ros2 service call` 은 매번 노드를 새로 만들어
> 디스커버리가 섞인다. 두 도구 모두 노드를 한 번만 만들고 루프로 부른다.

측정 중 로봇 PC IP 가 172.24.3.122 → 172.24.1.117 로 바뀌었는데 **ROS 는 멀쩡했다**
(ping 은 100% 손실). 클라이언트가 Discovery Server 에 자기 주소를 등록하므로
**고정이 필요한 것은 서버 PC 하나뿐**이다.

---

## 8. 남은 것

- **로봇 붙이기** — TCP 복구가 선행 (#23 · #26)
- **`peg_calibration.yaml` 나머지 3칸** — 안 차면 `measured: true` 로 못 올리고,
  그러면 실제 로봇에서는 페그가 아예 안 돈다. 지금 sim 파일로만 도는 이유다 (#8 · #13)
- **task_manager ↔ robot_controller 실제 Action 체인 지연** — 서버·로봇이 같이 떠야 잰다
- **돌출 문턱 확정** — 로봇 타현 세기가 정해져야 한다 (#6 · #7)

---

## 9. 재현 절차

```bash
cd ~/tunio_ws && colcon build && source install/setup.bash

# 1단계 — 전부 가짜
ros2 launch tunio_bringup sim_all.launch.py speed:=fast port:=9123 db:=/tmp/t1.db

# 2단계 — 진짜 마이크 (기타 없이)
ros2 launch tunio_bringup sim_robot.launch.py port:=9124 db:=/tmp/t2.db
ros2 param set /audio_analyzer replay_wav "$PWD/tools/g{string_number}_*.wav"

# 둘 다 — 15초 기다린 뒤
ros2 service call /tuning/start tunio_interfaces/srv/CommandTuning "{}"
ros2 service call /tuning/get_history tunio_interfaces/srv/GetHistory "{job_id: ''}"
```

`replay_wav` 에 쓰는 녹음 파일은 `.gitignore` 의 `*.wav` 대상이라 저장소에 없다.
정정인에게 요청하면 된다. 없으면 2단계는 사람이 직접 줄을 튕기면 된다.
