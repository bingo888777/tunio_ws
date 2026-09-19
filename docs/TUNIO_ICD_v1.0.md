# TUNIO 인터페이스 명세서

문서 번호: TUNIO-ICD-001
버전: v1.0
기준 패키지: `tunio_interfaces`
기준 브랜치: `feat/interface-migration`

---

## 0. 이 문서에 대하여

### 목적

노드를 구현하는 사람이 **다른 노드의 코드를 열어보지 않고도** 통신 규약을 알 수 있게 한다.
필드 이름, 타입, 단위, 허용 범위, 의미, 누가 보내고 누가 받는지를 모두 적는다.

### 범위

`tunio_interfaces` 패키지가 정의하는 Action 4개, Message 4개, Service 5개.
노드 내부 구현, 좌표 YAML, DB 스키마는 이 문서의 범위가 아니다.

### 우선순위

**기준본은 `docs/TUNIO_INTERFACE_SPEC.md` 다.** 이 문서와 SPEC 이 다르면 SPEC 이 기준이다.

이 문서는 필드 단위 상세(단위·허용 범위·송수신자·시퀀스)를 담는 참고 문서다.
SPEC 에 없는 상세가 여기 있으면 그대로 쓰고, 둘이 충돌하면 SPEC 을 따른 뒤
담당자에게 알려 이 문서를 고친다.
기획서는 왜 그렇게 정했는지를 담고, 이 문서는 무엇을 주고받는지를 담는다.

> 2026-09-19 변경: 이 문서가 스스로를 기준본으로 선언해, 같은 선언을 하는 SPEC 및
> SPEC 을 기준본으로 지목하는 CLAUDE.md 7절과 3중으로 충돌하던 것을 정리했다.
> 세션·담당자마다 다른 문서를 기준으로 작업하면 조용히 갈라지기 때문이다.

### 변경 절차

1. 변경이 필요하면 담당자에게 알리고 합의한다
2. 이 문서를 먼저 고친다
3. `.action` / `.msg` / `.srv` 파일을 고친다
4. 영향받는 노드 코드를 고친다
5. 문서 버전을 올리고 아래 변경 이력에 적는다

혼자 바꾸지 않는다. 인터페이스는 여러 사람이 동시에 의존하는 계약이다.

---

## 1. 공통 규약

### 1.1 식별자

**job_id** — `string`
기타 한 대에 대한 작업 단위. `task_manager`가 작업 시작 시 생성한다.
형식은 `YYYYMMDD_HHMMSS`. 예: `20260922_143015`
로그와 DB에서 사람이 바로 읽을 수 있도록 시각 문자열을 쓴다.
대기 상태에서는 빈 문자열 `""`.

**measurement_id** — `uint32`
한 작업 안에서의 측정 일련번호. `1`부터 시작해 측정할 때마다 1씩 증가한다.
작업이 바뀌면 다시 1부터 시작한다.
최초 측정, 재측정, 검증 패스의 측정이 모두 각각 번호를 받는다.

**string_number** — `uint8`
배열 인덱스가 아니라 **실제 기타 줄 번호**다.
유효값은 `6`, `5`, `2`, `1` 네 가지. 순서도 이 순이다.
해당 없음을 나타낼 때만 `0`을 쓴다 (`TuningState.current_string`).
`4`, `3`은 현재 범위 밖이지만 타입상으로는 들어올 수 있으므로,
받는 쪽에서 교시된 좌표가 없으면 실패로 응답한다.

### 1.2 단위

주파수는 `float32`, 단위는 **Hz**.
각도는 `float32`, 단위는 **도(degree)**. 라디안이 아니다.
시간은 `float32`, 단위는 **초(second)**. 밀리초가 아니다.
음정 오차는 `float32`, 단위는 **센트(cent)**.

센트 계산식:
```
센트 = 1200 × log2(측정 주파수 ÷ 목표 주파수)
```
양수면 목표보다 높은 음, 음수면 낮은 음.

### 1.3 페그 각도의 기준

`AdjustPeg.angle_deg`는 **손잡이(튜닝 버튼) 각도**다.
줄이 감기는 축(포스트) 각도가 아니다.

기타 페그는 보통 14:1로 감속되어 있어, 손잡이 14회전이 축 1회전이다.
로봇도 손잡이를 파지해 돌리므로 기준이 일치한다.
축 각도로 환산하지 않는다.

### 1.4 성공과 실패

모든 Action Result는 `bool success`를 가진다. 받는 쪽은 **항상 이 값을 먼저 확인**한다.

`success == false`일 때 다른 값은 의미가 없다.
특히 `frequency_hz`를 읽지 않는다. `0.0`을 유효한 측정값으로 쓰지 않는다.

실패 이유를 표현하는 방식은 둘로 나뉜다.

- **`uint8 failure_reason`** — `MeasurePitch`만 사용.
  서버가 값으로 분기하고 DB에 정수로 저장하기 때문에 상수로 정의한다.
- **`string message`** — 로봇 Action 3개가 사용.
  사람이 읽는 로그용이며 프로그램이 분기에 쓰지 않는다.
  성공 시 빈 문자열 `""`.

### 1.5 열거 상수 규칙

**기존 상수의 번호를 절대 바꾸지 않는다. 뒤에 추가만 한다.**

이유: 실패 사유와 결과 코드는 정수 그대로 SQLite에 저장된다.
번호를 바꾸면 과거 기록이 다른 의미로 재해석된다.

더 중요한 이유: **ROS 2는 이 변경을 감지하지 못한다.**
타입 해시(RIHS)에 상수는 포함되지 않아, 상수 번호를 바꿔 재빌드해도 해시가 동일하다.
따라서 노드 간 버전이 어긋나도 통신은 정상적으로 이루어지고 값만 조용히 오역된다.
실측으로 확인한 사실이다.

---

## 2. 노드별 제공·사용 목록

### task_manager (서버 PC · 정동휘)

제공하는 Service
- `/tuning/start` · `/tuning/pause` · `/tuning/resume` · `/tuning/cancel` (CommandTuning)
- `/tuning/get_settings` (GetSettings)
- `/tuning/set_settings` (SetSettings)

발행하는 Topic
- `/tuning/state` (TuningState)

호출하는 Action
- `/audio/measure_pitch` · `/robot/play_string` · `/robot/adjust_peg` · `/robot/strum_all`

호출하는 Service
- `/tuning/save_job`

### hmi_node (서버 PC · 정동휘)

구독하는 Topic
- `/tuning/state`

호출하는 Service
- 버튼 4개, 설정 2개, `/tuning/get_history`

### result_store (서버 PC · 정동휘)

제공하는 Service
- `/tuning/get_history` (GetHistory)
- `/tuning/save_job` (SaveJob)

### robot_controller (로봇 PC · 정묵·성현)

제공하는 Action
- `/robot/play_string` (성현)
- `/robot/adjust_peg` (정묵)
- `/robot/strum_all` (공동)

### audio_analyzer (마이크 PC · 정정인)

제공하는 Action
- `/audio/measure_pitch`

---

## 3. Action

### 3.1 MeasurePitch — `/audio/measure_pitch`

호출: `task_manager` → `audio_analyzer`
목적: 요청받은 한 줄의 기본 주파수(F0)를 1회 측정한다.

마이크는 상시 분석하지 않는다. Goal을 받았을 때만 녹음·분석한다.

#### Goal

```
string  job_id
uint8   string_number
uint32  measurement_id
float32 target_hz
float32 timeout_sec
float32 onset_timeout_sec
```

- `job_id` / `string_number` / `measurement_id` — 1.1절 참조. Result에 그대로 되돌려준다
- `target_hz` — 이 줄의 목표 주파수.
  마이크는 이 값의 ±2반음(약 ±200센트)으로 탐색 범위를 제한해 옥타브 오인을 차단한다.
  목표 주파수의 단일 출처는 서버다. 마이크는 자체 표를 두지 않는다.
  유효값: `82.4069` (6번) / `110.0` (5번) / `246.9417` (2번) / `329.6276` (1번)
- `timeout_sec` — 분석 응답 제한 시간. 녹음 길이가 아니다.
  **카운트 시작은 마이크가 오디오 onset을 검출한 시점이다.**
  초과 시 `FAIL_TIMEOUT`. 현재값 `10.0`
- `onset_timeout_sec` — onset 대기 상한.
  **카운트 시작은 `ready` Feedback을 발행한 시점이다.**
  이 안에 onset이 검출되지 않으면 `FAIL_LOW_SIGNAL`. 현재값 `10.0`

#### Result

```
string  job_id
uint8   string_number
uint32  measurement_id
bool    success
float32 frequency_hz
uint8   failure_reason
```

- 앞 세 필드는 Goal 값을 그대로 복사해 반환한다. **생략하거나 다른 값을 넣지 않는다.**
  서버는 이 값을 현재 요청과 대조해 일치하지 않으면 결과를 폐기한다.
  이것이 없으면 지연된 이전 측정값이 현재 줄의 값으로 채택되며, 어떤 오류도 발생하지 않는다.
- `success` — 유효한 주파수를 얻었는지 여부
- `frequency_hz` — 측정 주파수. `success == false`일 때 값은 정의되지 않는다.
  받는 쪽은 읽지 않는다
- `failure_reason` — 4.1절 상수. `success == true`이면 `FAIL_NONE(0)`

#### Feedback

```
bool ready
```

- `ready == true` — 녹음 준비 완료.
  **오디오 버퍼가 실제로 유입되기 시작한 뒤에 발행한다.**
  장치 오픈 명령 직후가 아니다. 그 사이에 발현이 일어나면 소리를 놓친다.
- 서버는 이 Feedback을 받은 뒤에 `play_string`을 호출한다
- 발행 횟수는 1회

#### 취소

서버가 goal을 취소하면 마이크는 `success == false`, `failure_reason == FAIL_CANCELED(4)`로
Result를 1회 반환한다. 무응답으로 종료하지 않는다.

#### onset 검출 요구사항

`ready` 발행 직후부터 입력을 감시하고, 무음에서 유음으로 전환되는 시점을 검출한다.
검출 방식은 구현자 재량이나, 짧은 프레임의 RMS가 무음 기준선 대비 일정 배수를 넘는 시점을
기준으로 삼는 방법이 있다. 임계값과 프레임 길이는 실측으로 정한다.

로봇 구동음을 발현으로 오인하지 않아야 한다. 로봇이 동작 중인 조건에서 검증한다.

---

### 3.2 PlayString — `/robot/play_string`

호출: `task_manager` → `robot_controller`
목적: 지정한 줄을 피크로 1회 튕긴다.

#### Goal

```
uint8 string_number
```

좌표와 자세는 로봇측 YAML의 교시값을 사용한다. Goal에 좌표를 싣지 않는다.

#### Result

```
bool   success
string message
```

- `message` — 실패 사유. 성공 시 빈 문자열.
  권장 어휘: `"교시 좌표 없음"` / `"모션 실패"` / `"충돌 감지"`
  (`AdjustPeg`와 동일한 어휘를 사용한다)

#### Feedback

필드 없음. Feedback 섹션은 비어 있으나 `---` 구분자는 2개를 유지한다.

> 향후 추가 예정: 힘 값 미분으로 발현 순간을 감지해 `plucked` 신호를 발행.
> 현 버전에서는 마이크가 오디오 onset으로 직접 판단하므로 불필요하다.

#### 동작 요구사항

인터페이스에 드러나지 않으나 구현에 포함되어야 하는 사항.

- 발현 직후 **5mm 이상 후퇴**한다. 피크가 진동하는 줄에 닿으면 버징이 발생해 분석이 불가능해진다
- 이웃 줄을 동시에 건드리지 않는다. 두 줄이 함께 울리면 주파수 분리가 불가능하다
- 피크는 거치대에서 집었다 반환한다 (2026-09-19 변경. v1.2까지는 손가락 외측
  부착이었으나, 피크를 쥐면 손가락이 막혀 페그를 돌릴 수 없어 성립하지 않았다)
- **집기·반환을 언제 할지는 로봇이 스스로 판단한다.** 서버는 지시하지 않으며
  이 Action 의 필드도 바뀌지 않는다. 피크를 가지러 가는 시간은 로봇 내부 사정이다
- 다만 그 시간이 `onset_timeout_sec` 예산 안에 들어간다. 마이크는 `ready` 를
  발행한 시점부터 세므로, 피크 왕복이 길어지면 발현 전에 제한 시간이 지나
  `FAIL_LOW_SIGNAL`(발현 실패)로 보고된다. 실측 후 값을 조정해야 한다

#### Result 반환 시점

후퇴 완료 후 반환한다. 후퇴 전에 반환하면 서버가 다음 동작을 요청할 수 있고,
피크가 줄에 닿은 채로 이동하게 된다.

---

### 3.3 AdjustPeg — `/robot/adjust_peg`

호출: `task_manager` → `robot_controller`
목적: 지정한 줄의 페그를 요청받은 방향과 각도로 회전시킨다.

#### Goal

```
uint8   string_number
int8    direction
float32 angle_deg
```

- `direction` — `+1`은 음 높이기(조임), `-1`은 음 낮추기(풀림).
  **줄별 실제 모터 회전 방향으로의 변환은 로봇측 YAML이 담당한다.**
  서버는 음정의 방향만 지시하며 물리적 회전 방향을 알지 못한다.
  `0`은 유효하지 않다
- `angle_deg` — 회전량. **항상 양수**로 보낸다. 방향은 `direction`이 담당한다.
  손잡이 각도 기준(1.3절)

#### Result

```
bool   success
string message
```

- `message` — 권장 어휘: `"파지 실패"` / `"과토크"` / `"걸림"` / `"모션 실패"`

#### Feedback

필드 없음.

#### 백래시 마무리 규약

페그 웜기어의 유격 때문에, 같은 각도라도 직전 회전 방향에 따라 결과가 달라진다.
**최종 접근은 항상 조이는 방향(`direction = +1`)으로 통일한다.**

**이 판단은 `task_manager`가 한다.** 로봇은 요청받은 방향과 각도를 그대로 수행할 뿐이며,
스스로 되돌리거나 보정하지 않는다.

측정값이 목표보다 높은 경우 서버는 다음과 같이 두 번 호출한다.

```
1) AdjustPeg(direction = -1, angle_deg = A)   목표 아래로 충분히 풀기
2) AdjustPeg(direction = +1, angle_deg = B)   조여 올려 마무리
```

따라서 **조절 1회가 goal 2회로 구성될 수 있다.**
`max_adjust_count`의 집계 기준은 goal 횟수가 아니라 **재측정이 발생하는 횟수**다.

#### 실패 시 상태

파지를 유지한 채 정지한다. 페그를 놓지 않는다.
재파지 동작은 추가 실패 위험이 있으며, 일시정지·취소 시의 파지 유지 정책과도 일치한다.

#### 실측이 필요한 파라미터

다음 값이 확정되어야 서버가 `angle_deg`를 계산할 수 있다. 현재 미정이다.

- 손잡이 1도당 음정 변화량 (센트/도) — 줄별
- 백래시 양 (도) — 방향 전환 후 음정이 변하기 시작하는 각도
- 1회 최대 안전 각도 (도) — 줄 파손 방지
- 최소 유효 각도 (도) — 이보다 작으면 유격에 묻힌다

---

### 3.4 StrumAll — `/robot/strum_all`

호출: `task_manager` → `robot_controller`
목적: 전체 줄을 훑어 화음을 울린다. 작업 완료를 알리는 연출 동작이다.

#### Goal

```
uint8[] string_numbers
```

- 비어 있으면 교시된 전체 줄을 대상으로 한다. **현재 운용은 항상 빈 배열이다**
- 값이 있으면 해당 줄만 대상으로 한다

#### Result

```
bool   success
string message
```

#### Feedback

필드 없음.

#### 주의

**이 동작 중에는 측정하지 않는다.** 복수의 줄이 동시에 울리면 주파수가 중첩되어
개별 줄의 F0를 분리할 수 없다. 측정은 반드시 `play_string`을 통한 단일 줄 발현으로만 한다.

---

## 4. Message

### 4.1 StringResult

줄 하나의 조율 결과. `TuningState`, `GetHistory`, `SaveJob`에서 공통으로 쓴다.

```
uint8   string_number
float32 target_hz
float32 tuned_hz
bool    has_tuned_hz
float32 verified_hz
bool    has_verified_hz
float32 final_error_cents
uint8   adjust_count
uint8   result
uint8   failure_reason
```

- `target_hz` — 목표 주파수 (A4=440Hz, 12평균율 기준 고정값)
- `tuned_hz` / `has_tuned_hz` — Phase 1 조율 루프에서 마지막으로 얻은 **유효** 측정값.
  `has_tuned_hz == false`면 유효값을 얻지 못한 것이며, `tuned_hz`를 읽지 않는다.
  DB에는 NULL로 저장한다
- `verified_hz` / `has_verified_hz` — 최종 검증 패스에서 얻은 측정값. 위와 동일한 규칙
- `final_error_cents` — 최종 판정에 사용한 센트 오차.
  검증값이 있으면 검증값 기준, 없으면 조율값 기준
- `adjust_count` — 실제 수행한 페그 조절 횟수.
  최초 측정은 포함하지 않는다. goal 2회로 구성된 조절도 1로 센다

#### result 상수

```
0  RESULT_PENDING    아직 처리 전
1  RESULT_SUCCESS    허용 센트 이내
2  RESULT_FAILED     실패
```

#### failure_reason 상수

```
0  FAIL_NONE                실패 아님
1  FAIL_LOW_SIGNAL          소리 부족 (측정 재시도 소진)
2  FAIL_NO_PITCH            주파수 판별 불가 (측정 재시도 소진)
3  FAIL_TIMEOUT             응답 시간 초과 (측정 재시도 소진)
4  FAIL_CANCELED            서버 취소로 중단
5  FAIL_ADJUST_EXHAUSTED    조절 횟수 소진 후에도 허용 센트 초과
6  FAIL_ROBOT_ERROR         로봇 동작 실패
7  FAIL_DETUNED_ON_VERIFY   검증 패스에서 틀어짐
```

`0`부터 `4`까지는 `MeasurePitch.failure_reason`과 번호·의미가 완전히 일치한다.
`task_manager`는 매핑 없이 값을 그대로 복사할 수 있다.
`5`부터 `7`은 서버가 판단하는 사유이며 마이크가 반환하지 않는다.

---

### 4.2 TuningSettings

조율 파라미터 묶음. UI 설정 탭, 상태 Topic, DB 저장에 공통으로 쓴다.

```
float32 tolerance_cents
uint8   max_adjust_count
uint8   max_measure_retry
float32 measure_timeout_sec
float32 onset_timeout_sec
```

- `tolerance_cents` — 허용 센트. `|final_error_cents| <= tolerance_cents`이면 성공.
  현재값 `10.0`. 확정값 미정
- `max_adjust_count` — 줄당 최대 페그 조절 횟수. 현재값 `5`
- `max_measure_retry` — 측정 실패 시 페그를 돌리지 않고 재측정하는 최대 횟수. 현재값 `3`
- `measure_timeout_sec` — `MeasurePitch.timeout_sec`으로 전달. 현재값 `10.0`
- `onset_timeout_sec` — `MeasurePitch.onset_timeout_sec`으로 전달. 현재값 `10.0`

작업 시작 전에만 변경 가능하다. 작업 중에는 `SetSettings`가 거부한다.
작업 시작 시점의 값을 결과와 함께 DB에 저장해, 이후 설정 변경이 과거 기록의 판정 기준을
바꾸지 않도록 한다.

목표 주파수는 이 설정에 포함되지 않는다. UI 변경 대상이 아니다.

---

### 4.3 JobSummary

이력 목록 표시용 요약.

```
string                   job_id
builtin_interfaces/Time  finished_at
uint8                    overall_result
```

`overall_result`는 `TuningState`의 `OVERALL_*` 상수와 같은 체계를 쓴다.

---

### 4.4 TuningState

`/tuning/state`로 발행하는 현재 상태 전체. UI는 이 메시지만으로 실시간 화면을 구성한다.

```
builtin_interfaces/Time  stamp
string                   job_id
uint8                    phase
uint8                    current_string
float32                  target_hz
float32                  measured_hz
bool                     measurement_valid
float32                  error_hz
uint8                    adjust_count
StringResult[]           results
uint8                    overall_result
TuningSettings           active_settings
```

- `job_id` — 진행 중인 작업. 대기 상태면 빈 문자열
- `current_string` — 현재 처리 중인 줄. 해당 없으면 `0`
- `measured_hz` / `measurement_valid` — 가장 최근 측정값과 그 유효성.
  `false`면 UI에 '측정 불가'로 표시한다
- `error_hz` — **화면 표시용** 오차 = 측정 − 목표. 양수면 목표보다 높은 음.
  판정은 센트로 하고 표시는 Hz로 한다
- `results` — 줄별 결과 배열. 처리 전인 줄은 `RESULT_PENDING`
- `active_settings` — 이번 작업에 적용 중인 설정

#### phase 상수

```
 0  PHASE_IDLE           대기
 1  PHASE_MEASURE_READY  측정 준비 (ready Feedback 대기)
 2  PHASE_PLUCK          발현
 3  PHASE_ANALYZE        분석
 4  PHASE_ADJUST_PEG     페그 조절
 5  PHASE_VERIFY         최종 검증 패스
 6  PHASE_STRUM          화음 마무리
 7  PHASE_RETURN         대기 자세 복귀
 8  PHASE_DONE           완료
 9  PHASE_PAUSED         일시정지
10  PHASE_CANCELED       취소
11  PHASE_ERROR          오류
```

#### overall_result 상수

```
0  OVERALL_PENDING   판정 전
1  OVERALL_SUCCESS   4줄 모두 성공
2  OVERALL_PARTIAL   일부 성공
3  OVERALL_FAILED    모두 실패
```

전체 판정은 **최종 검증 패스 결과**를 기준으로 한다.

#### QoS

```
reliability  RELIABLE
durability   TRANSIENT_LOCAL
depth        1
```

발행 측과 구독 측 **양쪽에 동일하게 설정한다.** 한쪽만 설정하면 연결되지 않는다.

`TRANSIENT_LOCAL`이므로 나중에 실행된 구독자도 마지막 발행 메시지를 즉시 수신한다.
UI를 작업 도중에 켜도 빈 화면이 아니라 현재 상태가 표시된다.

---

## 5. Service

### 5.1 CommandTuning

제공: `task_manager`
사용 토픽: `/tuning/start` · `/tuning/pause` · `/tuning/resume` · `/tuning/cancel`

```
---
bool   accepted
string message
```

요청 필드가 없다. **어느 이름으로 호출되었는지가 곧 명령이다.**
네 서비스가 동일한 타입을 공유한다.

- `accepted` — 명령을 접수했는지 여부.
  실제 동작의 진행은 `/tuning/state`로 확인한다. 이 응답은 접수 여부만 나타낸다
- `message` — 거부 사유.
  예: `"준비되지 않음"` / `"이미 진행 중"` / `"진행 중인 작업 없음"` / `"현재 상태에서 불가"`

#### 각 명령의 조건

- `start` — 로봇·마이크 노드 준비 완료 + 진행 중인 작업 없음
- `pause` — 작업 진행 중
- `resume` — 일시정지 상태
- `cancel` — 작업 진행 중 또는 일시정지 상태

취소된 작업은 DB에 저장하지 않으며 재개하지 않는다.
사람이 기타와 로봇을 정리한 뒤 새 작업으로 시작한다.

---

### 5.2 GetSettings

제공: `task_manager`
토픽: `/tuning/get_settings`

```
---
TuningSettings settings
```

현재 적용 중인 설정을 반환한다. UI 설정 탭 로딩 시 호출한다.

---

### 5.3 SetSettings

제공: `task_manager`
토픽: `/tuning/set_settings`

```
TuningSettings settings
---
bool   accepted
string message
```

설정 전체를 교체한다. 일부 필드만 바꾸는 방식이 아니다.
호출 측은 `GetSettings`로 현재값을 받아 필요한 필드만 수정해 전송한다.

작업 진행 중에는 거부한다. `message`에 `"작업 진행 중"`을 넣는다.
입력 범위를 벗어나면 거부한다.

**이 서비스를 `task_manager`가 제공하는 이유:**
설정을 보유하면서 작업 진행 여부를 아는 유일한 노드이기 때문이다.
다른 노드가 제공하면 진행 상태를 조회하는 왕복이 추가로 필요하다.

---

### 5.4 GetHistory

제공: `result_store`
토픽: `/tuning/get_history`

```
string  job_id
uint16  limit
---
bool            found
JobSummary[]    jobs
StringResult[]  strings
TuningSettings  settings
```

두 가지 모드로 동작한다.

**목록 조회** — `job_id`가 빈 문자열
`jobs`에 최신순으로 최대 `limit`개를 담아 반환한다. `strings`는 비어 있다.

**상세 조회** — `job_id`가 채워져 있음
해당 작업의 `strings`(줄별 기록)와 `settings`(당시 적용 설정)를 반환한다. `jobs`는 비어 있다.

`found == false`면 나머지 필드는 모두 비어 있다.

---

### 5.5 SaveJob

제공: `result_store`
호출: `task_manager`
토픽: `/tuning/save_job`

```
string                   job_id
builtin_interfaces/Time  finished_at
uint8                    overall_result
StringResult[]           strings
TuningSettings           settings
---
bool   saved
string message
```

완료된 작업 한 건을 일괄 저장한다.

**정상 종료 시에만 호출한다.** 취소된 작업과 부분 결과는 저장하지 않는다.
작업 절차의 완료와 조율 성공은 다르다. `OVERALL_PARTIAL`이나 `OVERALL_FAILED`여도
정상 종료 절차를 마쳤다면 저장한다.

`settings`는 작업 시작 시점의 값이다. 저장 시점의 값이 아니다.

---

## 6. 통신 시퀀스

### 6.1 정상 측정 1회

```
1. task_manager  →  audio_analyzer   MeasurePitch Goal
                                     (job_id, string_number, measurement_id,
                                      target_hz, timeout_sec, onset_timeout_sec)

2. audio_analyzer →  task_manager     Feedback: ready = true
                                     ← 여기서부터 onset_timeout_sec 카운트

3. task_manager  →  robot_controller  PlayString Goal (string_number)

4. robot_controller                   접근 → 발현 → 5mm 후퇴

5. audio_analyzer                     onset 검출
                                     ← 여기서부터 timeout_sec 카운트
                                       과도음 구간 제외 후 안정 구간 분석

6. robot_controller →  task_manager   PlayString Result (success)

7. audio_analyzer  →  task_manager    MeasurePitch Result
                                     (ID 3개, success, frequency_hz, failure_reason)

8. task_manager                       ID 대조 → 센트 판정
```

6번과 7번의 순서는 보장되지 않는다. 서버는 둘 다 받은 뒤 진행한다.

### 6.2 측정 실패 시

`success == false`이면 페그를 조작하지 않고 `MeasurePitch`를 다시 호출한다.
최대 `max_measure_retry`회. `measurement_id`는 매번 새로 증가한다.

소진 시 해당 줄을 실패로 기록하고 마지막 `failure_reason`을 남긴다.

### 6.3 조율 필요 시

```
1. task_manager  →  robot_controller  AdjustPeg Goal (direction, angle_deg)
2. robot_controller →  task_manager    Result
   (목표보다 높았으면 1~2를 반대 방향으로 한 번 더)
3. 6.1절 처음으로 돌아가 재측정
```

최대 `max_adjust_count`회. 소진 시 `FAIL_ADJUST_EXHAUSTED`.

### 6.4 취소

```
1. hmi_node  →  task_manager   /tuning/cancel
2. task_manager →  hmi_node     accepted = true
3. task_manager                 진행 중인 Action에 취소 요청
4. audio_analyzer →  task_manager  Result (success=false, FAIL_CANCELED)
5. robot_controller             현재 위치 정지, 파지 유지
6. task_manager                 PHASE_CANCELED 발행, DB 저장하지 않음
```

자동 복귀하지 않는다. 사람이 정리한 뒤 새 작업으로 시작한다.

### 6.5 최종 검증 패스

4줄 조율 완료 후, 페그 조작 없이 4줄을 다시 측정만 한다.
한 줄을 조이면 넥이 미세하게 변형되어 먼저 맞춘 줄이 틀어질 수 있기 때문이다.

허용 센트를 벗어난 줄은 `FAIL_DETUNED_ON_VERIFY`로 실패 재기록한다.
전체 판정은 이 결과를 기준으로 한다.

---

## 7. 확인 방법

정의 확인
```
ros2 interface list | grep tunio_interfaces
ros2 interface show tunio_interfaces/action/MeasurePitch
ros2 interface show tunio_interfaces/msg/TuningState
```

Action 직접 호출 (서버 없이 단위 테스트)
```
ros2 action send_goal /robot/play_string \
  tunio_interfaces/action/PlayString "{string_number: 6}"

ros2 action send_goal /robot/adjust_peg \
  tunio_interfaces/action/AdjustPeg \
  "{string_number: 6, direction: 1, angle_deg: 30.0}"

ros2 action send_goal -f /audio/measure_pitch \
  tunio_interfaces/action/MeasurePitch \
  "{job_id: 'test', string_number: 6, measurement_id: 1,
    target_hz: 82.4069, timeout_sec: 10.0, onset_timeout_sec: 10.0}"
```

`-f` 옵션은 Feedback을 함께 출력한다.

Service 호출
```
ros2 service call /tuning/start tunio_interfaces/srv/CommandTuning
ros2 service call /tuning/get_settings tunio_interfaces/srv/GetSettings
```

Topic 확인
```
ros2 topic echo /tuning/state
ros2 topic info /tuning/state --verbose
```

`--verbose`로 QoS 설정이 양쪽 일치하는지 확인할 수 있다.

---

## 8. 미확정 사항

실측 후 확정한다. 현재값은 임시다.

- `tolerance_cents` 확정값 — 현재 `10.0`.
  5센트로 시작하면 수렴 실패 원인이 기준 탓인지 회전량 계산 탓인지 구분되지 않는다.
  루프 안정 후 조인다
- `onset_timeout_sec` 확정값 — 로봇 접근 시간 실측 후
- `measure_timeout_sec` 확정값 — 측정 1회 소요 시간 실측 후
- 페그 각도당 센트 변화량 — **현재 최우선 실측 항목.** 이 값 없이는 회전량 계산이 불가능하다
- 대기 자세 복귀 지시 방법 — 전용 Action이 없다. 담당 미정
- 재시도 카운터 초기화 범위 — 페그 조작 후 측정 재시도 횟수를 리셋할지
- 취소 외 오류 종료 시 DB 저장 정책

---

## 9. 범위 밖

다음은 이번 개발 범위에 포함되지 않는다.

- 브릿지핀 안착 확인 — 관련 Action과 상태를 모두 제거했다
- 기타 이송 — 배치·회수는 사람이 한다
- 4번·3번 줄 — 동일 로직으로 확장 가능하나 좌표 교시가 추가로 필요하다
- 악기 진단 리포트 — 향후 과제
- Wi-Fi 단절 대응 — 연결 감시와 자동 정지는 구현하지 않는다

---

## 10. 변경 이력

**v1.0** — 최초 작성. 인터페이스 13개 확정.

이전 정의 대비 변경:
- 제거: `PressPin` `TransferGuitar` `ControlCommand` `StartTuning`
- 추가: `TuningSettings` `JobSummary` `CommandTuning` `SaveJob`
- 실패 사유: 문자열 → `uint8` 상수
- 무효값 표현: NaN → `has_*` bool 플래그
- `PlayString`의 `plucked` Feedback 제거
- `MeasurePitch`에 `onset_timeout_sec`, `FAIL_CANCELED` 추가
- 설정 Service 제공 노드를 `result_store`에서 `task_manager`로 이동
- 백래시 마무리 주체를 로봇에서 `task_manager`로 확정
