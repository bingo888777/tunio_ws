> AI agent로 작업할 경우 먼저 **CLAUDE.md** 를 읽으세요.

# TUNIO — 협동로봇 기타 자동 조율 시스템

로봇팔(두산 M0609)이 기타 줄을 튕겨 주파수를 측정하고 페그를 조절해 자동 조율한다.
비전 미사용. 조율 판정은 마이크(주파수)로만 하고, 힘/토크 센서는 타현 세기 유지·
접촉 탐색·파지 확인·과토크 안전정지에만 쓴다 (조율 판정에는 쓰지 않는다).

## 구성 (ROS 2 Jazzy / Ubuntu 24.04)

2대의 PC가 동일 Wi-Fi에서 ROS 2로 통신한다 (2026-09-22 변경, 이전에는 3대).
마이크와 서버를 한 대로 합쳤다. 자세한 것은 `docs/TUNIO_시스템아키텍처_v3.md`.

| 패키지 | 실행 PC | 담당 | 역할 |
|---|---|---|---|
| `tunio_interfaces` | 공용 | 전원 | msg / srv / action 정의 |
| `tunio_server` | 정정인 PC | 정동휘 | task_manager, web_node(관제 웹), result_store(SQLite) |
| `tunio_robot` | 로봇 제어 PC | 정묵·성현 | robot_controller (두산 API) |
| `tunio_audio` | 정정인 PC | 정정인 | audio_analyzer (녹음·주파수 분석) |
| `tunio_bringup` | - | 공용 | 통합 런치 (station / 단일 PC 테스트) |

## 노드 관계

```
task_manager ─ measure_pitch Action ─ audio_analyzer
      │
      ├─ play_string / adjust_peg / strum_all Action ─ robot_controller
      │
      ├─ /tuning/state (Topic) ─ web_node ─ 브라우저
      └─ save_job / get_history (Service) ─ result_store ─ SQLite

설정 조회·변경 Service는 task_manager 자신이 제공한다.
```

## 인터페이스 (tunio_interfaces)

15개 (action 6 / msg 4 / srv 5). 세부 정의의 기준본은 `docs/TUNIO_INTERFACE_SPEC.md`다.

- **Action**: MeasurePitch, PlayString, AdjustPeg, StrumAll, RegisterPeg, ReturnHome
- **Service**: CommandTuning(start/pause/resume/cancel 공유), GetSettings, SetSettings, GetHistory, SaveJob
- **Topic/msg**: TuningState (`/tuning/state`), StringResult, TuningSettings, JobSummary

확인:

```bash
ros2 interface list | grep tunio_interfaces
ros2 interface show tunio_interfaces/action/MeasurePitch
```

열거 상수는 뒤에 추가만 한다. 기존 번호를 바꾸면 노드 간 불일치가 감지되지 않고
DB에 쌓인 기록까지 재해석된다.

## 빌드

```bash
cd tunio_ws
# 1) 인터페이스 먼저 빌드 (다른 패키지가 의존)
colcon build --packages-select tunio_interfaces
source install/setup.bash
# 2) 전체 빌드
colcon build
source install/setup.bash
```

## 실행

PC별로 각자 실행:

```bash
# 정정인 PC — 마이크 + 서버 + 관제 웹
ros2 launch tunio_bringup station.launch.py
# 로봇 제어 PC
ros2 launch tunio_robot robot.launch.py dry_run:=false
```

⚠ `robot.launch.py` 는 `dry_run` 기본값이 true 다. 안 주면 로봇이 안 움직인다.

단일 PC 테스트:

```bash
ros2 launch tunio_bringup all_in_one.launch.py
```

두 PC는 같은 `ROS_DOMAIN_ID`를 export 해야 서로 보인다. 강의실 Wi-Fi가
멀티캐스트를 막으면 Fast DDS Discovery Server를 쓴다 — 서버는 **정정인 PC**에서
띄운다 (`docs/TUNIO_시스템아키텍처_v3.md`).

## 테스트

```bash
# 로봇·마이크 없이 내부 로직과 대역 ROS 통신 검증
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=117 colcon test --packages-select tunio_robot tunio_server tunio_audio
```

피크 확인 정책과 검증 범위는 `docs/피크_확인_동작.md`를 참고하세요.

## 작업 순서 (권장)

1. **전원**: `tunio_interfaces`의 msg/srv/action 필드를 먼저 합의·고정한다. 이게 정해져야 각자 병렬 작업 가능.
2. 각 담당이 자기 패키지의 `TODO` 채우기.
3. `tunio_audio`는 별도 검증 스크립트(`tools/pitch_test.py`)로 마이크·알고리즘 먼저 확정 후 `pitch.py`에 이식.
4. 9/21 통합.

## 개발 상태

기획 기준본 v1.2 기반.

*(2026-09-22 기준)*

- **tunio_interfaces** — 15개 정의 완료 (빌드·검증 통과). 기준본은 `docs/TUNIO_INTERFACE_SPEC.md`
- **tunio_audio** — measure_pitch 노드 구현 완료. onset 검출·안정화 판정·유효 측정 기준 확정
- **tunio_server** — task_manager 상태기계·관제 웹·SQLite 저장 동작. 1차 통합 완료
- **tunio_robot** — 발현·페그 등록·조절·복귀 Action 구현. 발현·페그 조절·피크 집기·반환이 실기에서 돈다.
  Phase 0는 서버가 RegisterPeg를 요청하고 로봇 내부에서 수행하며, 페그·피크 파지는 Modbus 폭으로 판정한다.
  남은 것은 `strum_all`(화음)과 실기 반복 검증 — `TODO.md` 4절

**아직 임시값인 것이 많다.** 속도 배율·경유 문턱·재시도 한도·허용 센트·파지 문턱이
전부 실측 전이거나 표본이 작다. 무엇이 임시값인지는 `CLAUDE.md` 13절에 모여 있다.

기타 이송과 브릿지핀 안착 확인은 범위에서 제외했다.
