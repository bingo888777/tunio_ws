> AI agent로 작업할 경우 먼저 **CLAUDE.md** 를 읽으세요.

# TUNIO — 협동로봇 기타 자동 조율 시스템

로봇팔(두산 M0609)이 기타 줄을 튕겨 주파수를 측정하고 페그를 조절해 자동 조율한다.
비전 미사용. 조율 판정은 마이크(주파수)로만 하고, 힘/토크 센서는 타현 세기 유지·
접촉 탐색·파지 확인·과토크 안전정지에만 쓴다 (조율 판정에는 쓰지 않는다).

## 구성 (ROS 2 Jazzy / Ubuntu 24.04)

3대의 PC가 동일 Wi-Fi에서 ROS 2로 통신한다.

| 패키지 | 실행 PC | 담당 | 역할 |
|---|---|---|---|
| `tunio_interfaces` | 공용 | 전원 | msg / srv / action 정의 |
| `tunio_server` | 서버 | 정동휘 | task_manager, hmi_node(PyQt), result_store(SQLite) |
| `tunio_robot` | 로봇 제어 | 정묵·성현 | robot_controller (두산 API) |
| `tunio_audio` | 마이크 | 정정인 | audio_analyzer (녹음·주파수 분석) |
| `tunio_bringup` | - | 공용 | 통합 런치 (단일 PC 테스트용) |

## 노드 관계

```
task_manager ─ measure_pitch Action ─ audio_analyzer
      │
      ├─ play_string / adjust_peg / strum_all Action ─ robot_controller
      │
      ├─ /tuning/state (Topic) ─ hmi_node ─ PyQt
      └─ save_job / get_history (Service) ─ result_store ─ SQLite

설정 조회·변경 Service는 task_manager 자신이 제공한다.
```

## 인터페이스 (tunio_interfaces)

13개 (action 4 / msg 4 / srv 5). 세부 정의의 기준본은 `docs/TUNIO_INTERFACE_SPEC.md`다.

- **Action**: MeasurePitch, PlayString, AdjustPeg, StrumAll
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
# 서버 PC
ros2 launch tunio_server server.launch.py
# 로봇 제어 PC
ros2 launch tunio_robot robot.launch.py
# 마이크 PC
ros2 launch tunio_audio audio.launch.py
```

단일 PC 테스트:

```bash
ros2 launch tunio_bringup all_in_one.launch.py
```

세 PC는 같은 `ROS_DOMAIN_ID`를 export 해야 서로 보인다.

## 테스트

```bash
# 로봇·마이크 없이 조율 판단 로직만 검증
cd src/tunio_server && python3 -m pytest test/ -q
```

`colcon test`는 이 저장소에서 테스트를 0개 실행한다(pytest 대신 unittest를 돌린다).
"NO TESTS RAN"이 통과처럼 보이니 위 명령으로 직접 돌릴 것.

## 작업 순서 (권장)

1. **전원**: `tunio_interfaces`의 msg/srv/action 필드를 먼저 합의·고정한다. 이게 정해져야 각자 병렬 작업 가능.
2. 각 담당이 자기 패키지의 `TODO` 채우기.
3. `tunio_audio`는 별도 검증 스크립트(`tools/pitch_test.py`)로 마이크·알고리즘 먼저 확정 후 `pitch.py`에 이식.
4. 9/21 통합.

## 개발 상태

기획 기준본 v1.2 기반. **인터페이스 13개는 정의 완료**(빌드·검증 통과)이며,
각 노드의 실제 동작은 아직 뼈대다. 담당별 할 일은 각 파일 `TODO` 주석 참고.

기타 이송과 브릿지핀 안착 확인은 범위에서 제외했다.
