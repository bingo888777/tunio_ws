# TUNIO 인터페이스 요약

`tunio_interfaces` · Action 7 / Msg 4 / Srv 5 — 2026-09-30, main 기준.
기준본은 `TUNIO_INTERFACE_SPEC.md`. 이 문서는 필드와 상수만 한눈에 보는 용도다.

---

## 공통

```
job_id          string   "20260922_143015" 형식. task_manager 생성
measurement_id  uint32   작업 내 1부터 증가
string_number   uint8    실제 줄 번호 6/5/2/1. 해당 없음은 0
주파수          float32  Hz
각도            float32  도. 페그는 손잡이 각도 (축 아님)
시간            float32  초
```

성공 여부는 `bool success`로 판단한다. false면 다른 값을 읽지 않는다.
열거 상수는 뒤에 추가만. 기존 번호를 바꾸지 않는다.
로봇 Action은 ROS 상태 SUCCEEDED **와** `success=true` 둘 다 확인한 뒤에 다음 명령을 보낸다.

---

## Action

### MeasurePitch — `/audio/measure_pitch`
task_manager → audio_analyzer (같은 PC)

```
Goal
  string  job_id
  uint8   string_number
  uint32  measurement_id
  float32 target_hz            ±2반음으로 탐색 제한
  float32 timeout_sec          onset 검출 시점부터 카운트
  float32 onset_timeout_sec    ready 발행 시점부터 카운트

Result
  string  job_id               Goal 값 에코 (늦게 온 이전 결과를 걸러내는 기준)
  uint8   string_number        Goal 값 에코
  uint32  measurement_id       Goal 값 에코
  bool    success
  float32 frequency_hz         false면 읽지 않음
  uint8   failure_reason

Feedback
  bool    ready                녹음 준비 완료. 이걸 받은 뒤에만 발현
```

```
failure_reason
  0  FAIL_NONE
  1  FAIL_LOW_SIGNAL    소리 부족 / onset 미검출
  2  FAIL_NO_PITCH      주파수 판별 불가
  3  FAIL_TIMEOUT       분석 시간 초과
  4  FAIL_CANCELED      서버 취소
```

### PreparePluck — `/robot/prepare_pluck`
task_manager → robot_controller

```
Goal     uint8   string_number
         bool    resumed              일시정지·보충 대기 뒤 첫 요청. 로봇이 피크를 다시 확인
Result   bool    success
         string  message              실패 사유 또는 비치명적 경고
         string  preparation_id       성공 시 발급하는 일회용 식별자 → PlayString에 그대로
         bool    pick_refill_required 재집기 소진 후 홈 복귀 완료 → 서버는 보충 대기(PAUSED)
Feedback string  warning              재집기·홈 복귀·보충 안내 (발현 시점 신호 아님)
```

피크 집기(필요하면) → 줄 위 접근점까지. 접촉·스트로크는 하지 않는다.

### PlayString — `/robot/play_string`
task_manager → robot_controller

```
Goal     uint8   string_number
         bool    resumed              위와 같음
         string  preparation_id       PreparePluck 식별자. 같은 줄에 한 번만 소비
Result   bool    success
         string  message
         bool    pick_refill_required
Feedback string  warning
```

접근 → 스트로크 → 즉시 5mm 이상 후퇴. Result는 후퇴 뒤. 빈 `preparation_id`는 단독 도구 호환용이고 서버는 항상 채운다.

### AdjustPeg — `/robot/adjust_peg`
task_manager → robot_controller

```
Goal     uint8   string_number
         int8    direction     +1 조임(음↑) / -1 풀림(음↓)
         float32 angle_deg     항상 양수. 손잡이 각도
Result   bool    success
         string  message
Feedback (없음)
```

백래시 마무리(풀었다 조이기)는 task_manager가 결정. 조절 1회 = goal 2회일 수 있다.
등록된 각도가 없으면 움직이지 않고 실패한다 → 페그 재등록 먼저.

### RegisterPeg — `/robot/register_peg`
task_manager → robot_controller

```
Goal     uint8   string_number
         bool    force         true면 저장된 각도를 무효화하고 다시 등록
Result   bool    success
         string  message
Feedback (없음)
```

웹 「페그 재등록」은 force=true로 6→5→2→1. 재개 전 강제 재등록에도 쓴다. 각도는 로봇이 `peg_state.json`에 캐시.

### ReturnHome — `/robot/return_home`
task_manager → robot_controller

```
Goal     (없음)
Result   bool    success
         string  message
Feedback (없음)
```

피크를 쥐고 있으면 거치대에 반환한 뒤 대기 자세. 완료 뒤 · 취소 뒤 · 재등록 전에 쓴다. 화음은 없다.

### StrumAll — `/robot/strum_all`
정의만 유지. **정상 조율에서 호출하지 않는다** (#84). #104 넥 닦기로 대체 검토 중.

```
Goal     uint8[] string_numbers   비면 전체
Result   bool    success
         string  message
Feedback (없음)
```

---

## Message

### StringResult

```
uint8   string_number
float32 target_hz
float32 tuned_hz            조율 루프 마지막 유효 측정값
bool    has_tuned_hz
float32 verified_hz         최종 검증 패스 측정값
bool    has_verified_hz
float32 final_error_cents   검증값이 있으면 그 기준
uint8   adjust_count        실제 페그 조절 횟수 (최초 측정 제외)
uint8   result
uint8   failure_reason
```

```
result
  0  RESULT_PENDING
  1  RESULT_SUCCESS
  2  RESULT_FAILED

failure_reason              0~4는 MeasurePitch와 같은 번호
  0  FAIL_NONE
  1  FAIL_LOW_SIGNAL
  2  FAIL_NO_PITCH
  3  FAIL_TIMEOUT
  4  FAIL_CANCELED
  5  FAIL_ADJUST_EXHAUSTED  조절 소진
  6  FAIL_ROBOT_ERROR
  7  FAIL_DETUNED_ON_VERIFY 검증 시 틀어짐
```

`has_*`가 false면 해당 주파수는 무효. DB에는 NULL.

### TuningSettings

```
float32 tolerance_cents       10.0   (임시값)
uint8   max_adjust_count      5
uint8   max_measure_retry     3
float32 measure_timeout_sec   10.0
float32 onset_timeout_sec     10.0
```

작업 중 변경 불가. 작업 시작 시점 값을 결과와 함께 저장.

### JobSummary

```
string                  job_id
builtin_interfaces/Time finished_at
uint8                   overall_result
```

### TuningState — `/tuning/state`

```
builtin_interfaces/Time stamp
string                  job_id              대기면 빈 문자열
uint8                   phase
uint8                   current_string      해당 없으면 0
float32                 target_hz
float32                 measured_hz
bool                    measurement_valid
float32                 error_hz            측정 - 목표
uint8                   adjust_count
StringResult[]          results
uint8                   overall_result
TuningSettings          active_settings
string                  robot_notice        로봇이 보낸 최근 알림 (피크 없음 등). 새 작업 시 비움
string                  error_message       작업 중단 사유. ERROR에서 표시, 새 접수 시 비움
```

```
phase
   0  IDLE            5  VERIFY         10  CANCELED
   1  MEASURE_READY   6  STRUM (미사용)  11  ERROR
   2  PLUCK           7  RETURN         12  REGISTER_PEG   페그 재등록
   3  ANALYZE         8  DONE           13  PREPARE_PLUCK  피크 집기·접근
   4  ADJUST_PEG      9  PAUSED (일시정지 · 피크 보충 대기)

overall_result
  0  PENDING   1  SUCCESS   2  PARTIAL   3  FAILED
```

QoS: RELIABLE / TRANSIENT_LOCAL / depth 1. 발행·구독 양쪽 동일 설정.

---

## Service

### CommandTuning
`/tuning/start` `/tuning/pause` `/tuning/resume` `/tuning/cancel` `/tuning/reregister_pegs` — task_manager

```
Request   (없음)
Response  bool    accepted
          string  message
```

호출된 이름이 곧 명령. `accepted`는 **접수** 여부이지 완료·물리 정지가 아니다. 진행은 `/tuning/state`로.

- `start` — 이번 관제 실행에서 재등록·복귀를 마친 뒤에만 접수(허가 1회, 접수 시 소비)
- `reregister_pegs` — ReturnHome → RegisterPeg(force) 6→5→2→1 → ReturnHome → IDLE. 측정·저장 없음
- `pause` · `cancel` — 로봇 즉시 정지(MoveStop → IDLE 확인). 재개는 그 줄을 처음부터. 취소는 ReturnHome 후 CANCELED
- `resume` — 일시정지 또는 피크 보충 대기를 푼다. 보충 대기면 로봇 서비스 없이 서버에서 바로

### GetSettings — task_manager
```
Request   (없음)
Response  TuningSettings settings
```

### SetSettings — task_manager
```
Request   TuningSettings settings    전체 교체
Response  bool    accepted
          string  message
```
작업 중 거부.

### GetHistory — result_store
```
Request   string  job_id      비면 목록, 채우면 상세
          uint16  limit
Response  bool            found
          JobSummary[]    jobs      목록 조회 시
          StringResult[]  strings   상세 조회 시
          TuningSettings  settings  상세 조회 시
```

### SaveJob — result_store
```
Request   string                  job_id
          builtin_interfaces/Time finished_at
          uint8                   overall_result
          StringResult[]          strings
          TuningSettings          settings
Response  bool    saved
          string  message
```
정상 종료 시에만 호출. 4줄을 완주했으면 PARTIAL·FAILED도 저장. 취소·오류로 끊긴 작업은 저장 안 함.

---

## 측정 1회 순서

```
1  TM → RC   PreparePluck Goal           피크 집기(확인) → 줄 위 접근점 → preparation_id
2  TM → AA   MeasurePitch Goal
3  AA → TM   ready = true                onset_timeout 카운트 시작
4  TM → RC   PlayString Goal (preparation_id)
5  RC        접촉 → 스트로크 → 5mm 후퇴 → (verify_pick) 폭 확인
6  AA        onset 검출                  timeout 카운트 시작
7  RC → TM   PlayString Result
8  AA → TM   MeasurePitch Result
9  TM        ID 대조 → 센트 판정 → 필요하면 피크 반환·AdjustPeg → 1부터
```

7과 8의 순서는 보장되지 않는다. 1에서 `pick_refill_required`면 서버는 PAUSED로 기다리고, 웹 재개 뒤 1부터 `resumed=true`로 다시.

---

## 확인

```
ros2 interface list | grep tunio_interfaces
ros2 interface show tunio_interfaces/action/PlayString
ros2 action send_goal /robot/play_string \
  tunio_interfaces/action/PlayString "{string_number: 6, resumed: false, preparation_id: ''}"
ros2 topic info /tuning/state --verbose
ros2 service call /tuning/reregister_pegs tunio_interfaces/srv/CommandTuning '{}'
```
