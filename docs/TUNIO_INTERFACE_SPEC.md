# tunio_interfaces 명세 — 검증용

이 문서는 `tunio_ws/src/tunio_interfaces` 패키지가 의도대로 만들어졌는지 대조하기 위한 기준본이다.
아래 정의와 실제 파일이 다르면 **이 문서 쪽이 기준**이다.

TUNIO는 협동로봇(두산 M0609 + RG2)이 기타 줄을 튕겨 주파수를 측정하고 페그를 돌려 조율하는 시스템이다.
PC 2대가 ROS 2 Jazzy로 통신한다(2026-09-22, PC 3대에서 축소). 정정인 PC(task_manager,
web_node, result_store, audio_analyzer) / 로봇 제어 PC(robot_controller). 랜을 건너는
인터페이스는 `/robot/play_string`·`adjust_peg`·`strum_all`·`register_peg`·`return_home` 다섯이다 — `measure_pitch`는
정정인 PC 한 대 안에서 돈다. 근거는 `CLAUDE.md` 6절.

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
ros2 interface show tunio_interfaces/action/RegisterPeg
ros2 interface show tunio_interfaces/action/ReturnHome
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

빌드가 통과하고 15개 인터페이스가 모두 보이면 통과다.

---

## 2. 기대 파일 목록

```
src/tunio_interfaces/
├── action/
│   ├── MeasurePitch.action
│   ├── PlayString.action
│   ├── AdjustPeg.action
│   ├── StrumAll.action
│   ├── RegisterPeg.action
│   └── ReturnHome.action
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

> ⚠️ **빈 Feedback 섹션을 지우지 말 것.**
>
> ROS 2 action 은 Goal / Result / Feedback 3부 구조라 `---` 구분자가 **정확히 2개**여야
> 파싱된다. Feedback 을 쓰지 않는 Action 도 `---` 와 `# --- Feedback ---` 을 남겨야 한다.
> 필드가 없으니 실수처럼 보이지만 실수가 아니다. 지우면 빌드가 이렇게 죽는다.
>
> ```
> rosidl_adapter.parser.InvalidActionSpecification:
> Number of '---' separators nonconformant with action definition
> ```
>
> 아래 예시 4개는 전부 `---` 가 2개다. 3절의 "Feedback 은 MeasurePitch 에만 있다"는
> *필드* 기준 서술이며, 구분자 개수와는 별개다.
>
> (2026-09-19 수정: AdjustPeg·StrumAll 예시에 Feedback 섹션이 빠져 있어,
>  기준본을 그대로 옮기면 빌드가 깨지는 상태였다. 실제 `.action` 파일 4개는
>  처음부터 구분자가 2개로 맞아 있었다.)

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
---
# --- Feedback ---
# 사용하지 않음
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
---
# --- Feedback ---
# 사용하지 않음
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

`/tuning/state` · task_manager → web_node
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
uint8 PHASE_REGISTER_PEG=12

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
  "action/RegisterPeg.action"
  "action/ReturnHome.action"
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

15개 파일이 모두 등록돼 있어야 한다. 하나라도 빠지면 그 인터페이스는 생성되지 않는다.

---

## 8. 흔한 실수 점검

- `rosidl_generate_interfaces`에 파일이 빠짐 → 빌드는 통과하는데 `ros2 interface show`에서 안 보임
- `CommandTuning.srv`에 `---` 누락 → 빌드 실패
- `member_of_group` 누락 → 다른 패키지에서 import 실패
- 같은 패키지 안의 메시지를 참조할 때는 패키지명 없이 `StringResult[] results`로 쓴다
- `builtin_interfaces/Time`을 쓰면서 `DEPENDENCIES builtin_interfaces`를 빠뜨림 → 빌드 실패
- `.gitignore`에 `build/ install/ log/`가 없으면 빌드 산출물이 커밋됨

---

## 9. 확정 사항과 미확정 사항

### 9.1 확정된 것

담당자 합의를 마쳤다. 바꾸려면 다시 합의가 필요하다.

- **백래시 마무리 주체 — 서버(task_manager)**
  목표보다 높으면 아래로 풀었다 조여 올리는 판단을 task_manager가 한다.
  로봇은 시킨 방향과 각도만 수행하므로 조절 1회가 goal 2번이 될 수 있다.
  기획서 9.1절이 로봇 기준으로 적혀 있던 것은 서버 기준으로 수정했다.
- **타임아웃 주체 — 마이크(audio_analyzer)**
  마이크가 직접 재고 `FAIL_TIMEOUT`을 반환한다.
  서버가 Action을 취소하는 방식이면 실패 사유가 뭉개지므로 쓰지 않는다.
  카운트 시작은 마이크가 스스로 검출한 오디오 onset 시점이다.
- **설정 Service 제공 노드 — task_manager**
  `/tuning/get_settings`, `/tuning/set_settings` 둘 다 task_manager가 제공한다.
  설정을 보유한 노드이자 작업 진행 여부를 아는 유일한 노드라서,
  "작업 중이면 SetSettings 거부"를 다른 노드에 물어보지 않고 판단할 수 있다.
  `TuningState.active_settings` 발행 주체와도 일치한다.

### 9.2 아직 미확정

- **허용 센트 기본값** — 코드에 10.0으로 넣었다. 첫 통합 테스트를 이 값으로 시작하고
  실측 후 확정한다. 5로 시작하면 수렴 실패 시 기준이 빡빡한 탓인지
  회전량 계산이 틀린 탓인지 구분되지 않는다.
- **`onset_timeout_sec` 기본값** — 임시 10.0. 로봇 접근·파지·발현 실측 후 조정한다.


## 시작 전 페그 초기 등록 (2026-09-22)

`/robot/register_peg` — `RegisterPeg.action`, task_manager → robot_controller.

```text
uint8 string_number
---
bool success
string message
---
```

시작 시 6→5→2→1 순서로 요청하고 전부 성공한 뒤 마이크 측정과 발현을 시작한다.
로봇은 저장된 유효 각도가 있으면 움직이지 않고 성공하며 미등록 줄만 등록한다.
실패/시간 초과 시 작업을 PHASE_ERROR로 종료하고 발현하지 않는다.
줄별 대기시간은 서버 `register_timeout_sec`(180초, 실측 전 임시값)으로 설정한다.
등록 중 상태는 PHASE_REGISTER_PEG=12로 표시한다. 기존 번호는 바꾸지 않는다.
AdjustPeg는 등록을 수행하지 않는다. 미등록이면 이동 전에 실패를 반환한다.
등록 무효화 후에도 RegisterPeg를 명시적으로 완료한 뒤 AdjustPeg를 요청한다.
서버·로봇 PC 모두 tunio_interfaces와 사용하는 노드를 재빌드/재시작해야 한다.


## 피크 반환·복귀 및 시간 초과 (2026-09-22)

`/robot/return_home` — `ReturnHome.action`, task_manager/단독 조율 시험 → robot_controller.
빈 Goal, Result는 `bool success`와 `string message`, 빈 Feedback이다.
피크를 쥐고 있으면 반환한 뒤 `frames.standby`로 간다. 화음은 연주하지 않는다.
반환 실패 시 복귀 성공으로 보고하지 않는다. StrumAll의 기존 복귀 동작은 호환을 위해 유지한다.

`adjust_timeout_sec`는 수락 후 요청 1회 기준 120초(임시값)이다.
로봇 Action 시간 초과 시 `robot_completion_wait_sec`(120초, 임시값)만큼 같은 결과를
추가로 기다린다. 추가 대기 중에도 성공 Result를 받으면 정상 성공으로 반영한다.
모든 로봇 Action은 ROS 상태 SUCCEEDED와 Result.success=true를 모두 확인한 뒤
다음 로봇 Action을 요청한다. 실패 Result, 취소 상태, Goal 거부는 PHASE_ERROR로 중단한다.
자동 재발현·다음 줄 이동·실패 후 자동 복귀를 요청하지 않는다.
마이크 측정 실패/조율 횟수 소진에 대한 다음 줄 정책은 별개로 유지한다.
끝내 종료 응답을 못 받거나 Goal 수락 여부를 모르면 PHASE_ERROR로 중단한다.
이 대기는 정지 명령이 아니다. 실제 움직임을 중지시켰다고 간주하지 않는다.

한 줄 시험 절차는 `docs/1번줄_ROS_통합시험.md`를 따른다.


## 화음 실행 보류 (#84, 2026-09-22 사용자 결정)

정상 조율에서는 최종 검증 → ReturnHome 성공 확인 → PHASE_DONE → 저장 순서로 진행한다.
미구현 화음은 `_run_job()`의 `_strum()` 호출만 주석 처리해 제외한다.
StrumAll Action 정의와 서버·로봇 함수는 삭제하지 않으며, 구현·실기 검증 후 호출을 복구한다.
ReturnHome 실패/거부/종료 불명은 기존 성공 Result 정책에 따라 작업을 중단한다.
