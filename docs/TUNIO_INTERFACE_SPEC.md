# tunio_interfaces 명세 — 검증용

이 문서는 `tunio_ws/src/tunio_interfaces` 패키지가 의도대로 만들어졌는지 대조하기 위한 기준본이다.
아래 정의와 실제 파일이 다르면 **이 문서 쪽이 기준**이다.

TUNIO는 협동로봇(두산 M0609 + RG2)이 기타 줄을 튕겨 주파수를 측정하고 페그를 돌려 조율하는 시스템이다.
PC 3대가 ROS 2 Jazzy로 통신한다. 서버(task_manager, hmi_node, result_store) / 로봇(robot_controller) / 마이크(audio_analyzer).

---

## 1. 검증 절차

```bash
cd ~/tunio_ws
colcon build --packages-select tunio_interfaces
source install/setup.bash

ros2 interface list | grep tunio_interfaces
ros2 interface show tunio_interfaces/action/MeasurePitch
ros2 interface show tunio_interfaces/action/PlayString
ros2 interface show tunio_interfaces/action/AdjustPeg
ros2 interface show tunio_interfaces/action/StrumAll
ros2 interface show tunio_interfaces/msg/TuningState
ros2 interface show tunio_interfaces/msg/StringResult
ros2 interface show tunio_interfaces/msg/TuningSettings
ros2 interface show tunio_interfaces/msg/JobSummary
ros2 interface show tunio_interfaces/srv/CommandTuning
ros2 interface show tunio_interfaces/srv/GetSettings
ros2 interface show tunio_interfaces/srv/SetSettings
ros2 interface show tunio_interfaces/srv/GetHistory
ros2 interface show tunio_interfaces/srv/SaveJob
```

빌드가 통과하고 13개 인터페이스가 모두 보이면 통과다.

---

## 2. 기대 파일 목록

```
src/tunio_interfaces/
├── action/
│   ├── MeasurePitch.action
│   ├── PlayString.action
│   ├── AdjustPeg.action
│   └── StrumAll.action
├── msg/
│   ├── TuningSettings.msg
│   ├── StringResult.msg
│   ├── JobSummary.msg
│   └── TuningState.msg
├── srv/
│   ├── CommandTuning.srv
│   ├── GetSettings.srv
│   ├── SetSettings.srv
│   ├── GetHistory.srv
│   └── SaveJob.srv
├── CMakeLists.txt
├── package.xml
└── README.md
```

이 패키지에는 노드 구현 코드가 들어가지 않는다. 인터페이스 정의만 둔다.

`PressPin.action`이 있으면 **삭제**한다. 브릿지핀 안착 확인은 프로젝트 범위에서 빠졌다.

---

## 3. 공통 규칙

- `job_id` — `string`. task_manager가 `"20260922_143015"` 형식으로 생성
- `measurement_id` — `uint32`. 작업 안에서 1부터 증가, 작업이 바뀌면 초기화
- `string_number` — 배열 인덱스가 아니라 **실제 줄 번호** (6, 5, 2, 1)
- 실패 표현 — `bool success` + `string message`. 단, `MeasurePitch`만 `failure_reason` 상수를 쓴다
- 열거 상수를 나중에 바꿀 때 기존 번호를 재사용하거나 당기지 않는다. 뒤에 추가만 한다
- Feedback은 `MeasurePitch`에만 있다

---

## 4. Action

### action/MeasurePitch.action

`/audio/measure_pitch` · task_manager → audio_analyzer
마이크는 상시 분석하지 않고 이 Goal을 받았을 때만 녹음/분석한다.

```
# --- Goal ---
string job_id
uint8 string_number
uint32 measurement_id
float32 target_hz
float32 timeout_sec
float32 onset_timeout_sec
---
# --- Result ---
string job_id
uint8 string_number
uint32 measurement_id
bool success
float32 frequency_hz
uint8 failure_reason
uint8 FAIL_NONE=0
uint8 FAIL_LOW_SIGNAL=1
uint8 FAIL_NO_PITCH=2
uint8 FAIL_TIMEOUT=3
uint8 FAIL_CANCELED=4
---
# --- Feedback ---
bool ready
```

- `target_hz` — 마이크가 이 값 ±2반음으로 탐색 범위를 제한한다. 옥타브 오인 차단용. 목표 주파수의 단일 출처는 서버다
- `timeout_sec` — 결과 응답 제한 시간이지 녹음 길이가 아니다. 카운트는 audio_analyzer가 오디오 onset(무음 → 소리)을 스스로 검출한 시점부터 시작하며, 초과 시 마이크가 `FAIL_TIMEOUT`을 반환한다. 로봇 접근·파지 시간이 타임아웃에 먹히지 않고, 서버나 로봇의 신호에 의존하지 않는다
- `onset_timeout_sec` — onset 대기 상한. 마이크가 `ready`를 발행해 듣기 시작한 시점부터 센다. 이 안에 onset이 들어오지 않으면 발현 자체가 실패한 것이므로 `FAIL_LOW_SIGNAL`을 반환한다. onset이 영영 오지 않을 때 Action이 매달리는 것을 막는 유일한 장치다
- Result의 ID 3개는 Goal 값을 그대로 되돌려준다. 늦게 도착한 이전 결과를 걸러내는 유일한 수단이라 생략 불가
- `success=false`면 `frequency_hz`는 읽지 않는다. 0Hz를 유효값으로 쓰지 않는다
- Feedback `ready=true`를 받은 뒤에 서버가 `play_string`을 요청한다

### action/PlayString.action

`/robot/play_string` · task_manager → robot_controller

```
# --- Goal ---
uint8 string_number
---
# --- Result ---
bool success
string message
---
# --- Feedback ---
# 사용하지 않음
```

- 발현 직후 5mm 이상 후퇴(버징 방지)는 로봇 내부에서 처리하며 인터페이스에 드러나지 않는다

### action/AdjustPeg.action

`/robot/adjust_peg` · task_manager → robot_controller

```
# --- Goal ---
uint8 string_number
int8 direction
float32 angle_deg
---
# --- Result ---
bool success
string message
```

- `direction` — `+1` 음 높이기(조임), `-1` 음 낮추기(풀림)
- `angle_deg` — 항상 양수 크기
- 줄별 실제 모터 회전 방향 변환은 로봇측 YAML이 담당한다
- 백래시 대응(목표보다 높으면 아래로 풀었다 조여 올리기)은 **task_manager가 결정한다.** 로봇은 시킨 방향과 각도만 수행한다. 따라서 조절 1회가 goal 2번이 될 수 있다

### action/StrumAll.action

`/robot/strum_all` · task_manager → robot_controller
완료 알림용 연출 동작. 측정하지 않는다.

```
# --- Goal ---
uint8[] string_numbers
---
# --- Result ---
bool success
string message
```

- 배열이 비면 교시된 전체 줄

---

## 5. Message

### msg/TuningSettings.msg

UI 설정 탭, 상태 Topic, DB 저장에 공통으로 쓴다.
작업 시작 시점의 값을 그대로 저장해, 이후 설정을 바꿔도 과거 작업의 기준이 바뀌지 않게 한다.

```
float32 tolerance_cents
uint8 max_adjust_count
uint8 max_measure_retry
float32 measure_timeout_sec
float32 onset_timeout_sec
```

- `tolerance_cents` — |1200 × log2(측정/목표)| 가 이 값 이내면 성공. 기본값 미정, 시작값 ±10 권장
- `max_adjust_count` — 줄당 최대 페그 조절 횟수 (임시 5)
- `max_measure_retry` — 측정 실패 시 페그를 돌리지 않고 다시 튕기는 최대 횟수 (임시 3)
- `measure_timeout_sec` — 임시 10.0
- `onset_timeout_sec` — onset 대기 상한. 임시 10.0. 로봇 접근·파지·발현에 걸리는 시간을 덮을 만큼 넉넉해야 한다

### msg/StringResult.msg

줄 하나의 조율 결과. 조율 직후 값과 최종 검증 값을 함께 보존한다.

```
uint8 string_number
float32 target_hz
float32 tuned_hz
bool has_tuned_hz
float32 verified_hz
bool has_verified_hz
float32 final_error_cents
uint8 adjust_count
uint8 result
uint8 failure_reason

uint8 RESULT_PENDING=0
uint8 RESULT_SUCCESS=1
uint8 RESULT_FAILED=2

uint8 FAIL_NONE=0
uint8 FAIL_LOW_SIGNAL=1
uint8 FAIL_NO_PITCH=2
uint8 FAIL_TIMEOUT=3
uint8 FAIL_CANCELED=4
uint8 FAIL_ADJUST_EXHAUSTED=5
uint8 FAIL_ROBOT_ERROR=6
uint8 FAIL_DETUNED_ON_VERIFY=7
```

- `has_tuned_hz` / `has_verified_hz` — DB의 NULL에 대응한다. false면 해당 주파수를 읽지 않는다. 0Hz를 유효값으로 쓰지 않으려는 장치라 생략 불가
- `adjust_count` — 실제 수행한 페그 조절 횟수. 최초 측정은 포함하지 않는다
- `FAIL_DETUNED_ON_VERIFY` — 조율은 성공했으나 최종 검증 패스에서 허용 센트를 벗어난 경우
- `FAIL_*` 0~4는 `MeasurePitch`의 `failure_reason`과 번호·의미가 같다. 측정 실패 사유를 그대로 옮겨 담을 수 있다. 5~7은 `StringResult`에만 있다

### msg/JobSummary.msg

이력 목록 한 줄에 표시할 요약 정보.

```
string job_id
builtin_interfaces/Time finished_at
uint8 overall_result
```

- `overall_result`는 `TuningState`의 `OVERALL_*` 값과 같은 체계를 쓴다

### msg/TuningState.msg

`/tuning/state` · task_manager → hmi_node
UI는 이 메시지 하나만 보고 실시간 탭을 그린다.
QoS는 reliable, depth 1, transient local. 늦게 켠 UI도 마지막 상태를 즉시 받는다.

```
builtin_interfaces/Time stamp
string job_id
uint8 phase
uint8 current_string
float32 target_hz
float32 measured_hz
bool measurement_valid
float32 error_hz
uint8 adjust_count
StringResult[] results
uint8 overall_result
TuningSettings active_settings

uint8 PHASE_IDLE=0
uint8 PHASE_MEASURE_READY=1
uint8 PHASE_PLUCK=2
uint8 PHASE_ANALYZE=3
uint8 PHASE_ADJUST_PEG=4
uint8 PHASE_VERIFY=5
uint8 PHASE_STRUM=6
uint8 PHASE_RETURN=7
uint8 PHASE_DONE=8
uint8 PHASE_PAUSED=9
uint8 PHASE_CANCELED=10
uint8 PHASE_ERROR=11

uint8 OVERALL_PENDING=0
uint8 OVERALL_SUCCESS=1
uint8 OVERALL_PARTIAL=2
uint8 OVERALL_FAILED=3
```

- `error_hz` — 화면 표시용 오차 = 측정 − 목표. 양수면 목표보다 높은 음. **판정은 센트, 표시는 Hz**
- `measurement_valid=false`면 화면에 '측정 불가' 표시
- `current_string` — 해당 없으면 0
- phase 상수에 `PHASE_PRESS_PIN`이 있으면 안 된다. 브릿지핀 제외로 빠졌고 뒤 번호를 당겼다

---

## 6. Service

### srv/CommandTuning.srv

`/tuning/start`, `/tuning/pause`, `/tuning/resume`, `/tuning/cancel` 네 개가 이 타입을 공유한다.
요청에 필드가 없다. 어느 이름으로 불렸는지가 곧 명령이다.

```
---
bool accepted
string message
```

- 파일 첫 줄이 `---`로 시작한다. 주석만 있고 요청 필드가 없는 형태다
- 접수 여부만 응답하고 실제 진행은 `/tuning/state`로 알린다

### srv/GetSettings.srv

```
---
TuningSettings settings
```

### srv/SetSettings.srv

작업 시작 전에만 허용하며 작업 중에는 거부한다.

```
TuningSettings settings
---
bool accepted
string message
```

### srv/GetHistory.srv

```
string job_id
uint16 limit
---
bool found
JobSummary[] jobs
StringResult[] strings
TuningSettings settings
```

- `job_id`가 비면 최신순 목록 조회, 채우면 그 작업의 줄별 상세 조회
- 목록 조회 시 `strings`는 비고, 상세 조회 시 `jobs`가 빈다

### srv/SaveJob.srv

task_manager → result_store
정상 종료 시에만 호출한다. 취소 작업과 부분 결과는 저장하지 않는다.

```
string job_id
builtin_interfaces/Time finished_at
uint8 overall_result
StringResult[] strings
TuningSettings settings
---
bool saved
string message
```

- `pin_check_done` 필드가 있으면 삭제한다. 브릿지핀 제외로 빠졌다

---

## 7. 빌드 파일

### package.xml 필수 항목

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>builtin_interfaces</depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

`<build_type>ament_cmake</build_type>`이 `<export>` 안에 있어야 한다.

### CMakeLists.txt 필수 항목

```cmake
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MeasurePitch.action"
  "action/PlayString.action"
  "action/AdjustPeg.action"
  "action/StrumAll.action"
  "msg/TuningSettings.msg"
  "msg/StringResult.msg"
  "msg/JobSummary.msg"
  "msg/TuningState.msg"
  "srv/CommandTuning.srv"
  "srv/GetSettings.srv"
  "srv/SetSettings.srv"
  "srv/GetHistory.srv"
  "srv/SaveJob.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
```

13개 파일이 모두 등록돼 있어야 한다. 하나라도 빠지면 그 인터페이스는 생성되지 않는다.

---

## 8. 흔한 실수 점검

- `rosidl_generate_interfaces`에 파일이 빠짐 → 빌드는 통과하는데 `ros2 interface show`에서 안 보임
- `CommandTuning.srv`에 `---` 누락 → 빌드 실패
- `member_of_group` 누락 → 다른 패키지에서 import 실패
- 같은 패키지 안의 메시지를 참조할 때는 패키지명 없이 `StringResult[] results`로 쓴다
- `builtin_interfaces/Time`을 쓰면서 `DEPENDENCIES builtin_interfaces`를 빠뜨림 → 빌드 실패
- `.gitignore`에 `build/ install/ log/`가 없으면 빌드 산출물이 커밋됨

---

## 9. 아직 확정되지 않은 것

담당자와 1:1 합의 후 바뀔 수 있는 부분이다. 지금 구현에 반영돼 있지만 논의 대상이다.

- **백래시 마무리 주체** — 현재는 task_manager가 결정(풀기 goal + 조이기 goal 두 번). 기획서 9.1절은 로봇이 마무리한다고 적혀 있어 문서 간 불일치가 있다
- **타임아웃 주체** — 현재는 마이크가 직접 재고 `FAIL_TIMEOUT` 반환. 서버가 Action을 취소하는 방식이면 실패 사유가 뭉개진다
- **설정 Service 제공 노드** — 현재 전제는 result_store. hmi_node가 제공하는 안도 가능
- **허용 센트 기본값** — 미정. 첫 통합 테스트는 ±10센트로 시작 권장
