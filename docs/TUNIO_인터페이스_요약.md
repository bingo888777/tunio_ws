# TUNIO 인터페이스 요약

`tunio_interfaces` · Action 4 / Msg 4 / Srv 5
상세는 ICD 문서 참조. 이 문서는 필드와 상수만 담는다.

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

---

## Action

### MeasurePitch — `/audio/measure_pitch`
task_manager → audio_analyzer

```
Goal
  string  job_id
  uint8   string_number
  uint32  measurement_id
  float32 target_hz            ±2반음으로 탐색 제한
  float32 timeout_sec          onset 검출 시점부터 카운트
  float32 onset_timeout_sec    ready 발행 시점부터 카운트

Result
  string  job_id               Goal 값 에코
  uint8   string_number        Goal 값 에코
  uint32  measurement_id       Goal 값 에코
  bool    success
  float32 frequency_hz         false면 읽지 않음
  uint8   failure_reason

Feedback
  bool    ready                버퍼 유입 확인 후 발행
```

```
failure_reason
  0  FAIL_NONE
  1  FAIL_LOW_SIGNAL    소리 부족 / onset 미검출
  2  FAIL_NO_PITCH      주파수 판별 불가
  3  FAIL_TIMEOUT       분석 시간 초과
  4  FAIL_CANCELED      서버 취소
```

### PlayString — `/robot/play_string`
task_manager → robot_controller

```
Goal     uint8   string_number
Result   bool    success
         string  message       성공 시 ""
Feedback (없음)
```

발현 직후 5mm 이상 후퇴. Result는 후퇴 완료 후 반환.

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

백래시 마무리는 task_manager가 결정. 조절 1회 = goal 2회일 수 있음.
실패 시 파지 유지.

### StrumAll — `/robot/strum_all`
task_manager → robot_controller

```
Goal     uint8[] string_numbers   비면 전체
Result   bool    success
         string  message
Feedback (없음)
```

측정하지 않는 연출 동작.

---

## Message

### StringResult

```
uint8   string_number
float32 target_hz
float32 tuned_hz            조율 직후
bool    has_tuned_hz
float32 verified_hz         검증 패스
bool    has_verified_hz
float32 final_error_cents
uint8   adjust_count
uint8   result
uint8   failure_reason
```

```
result
  0  RESULT_PENDING
  1  RESULT_SUCCESS
  2  RESULT_FAILED

failure_reason              0~4는 MeasurePitch와 동일
  0  FAIL_NONE
  1  FAIL_LOW_SIGNAL
  2  FAIL_NO_PITCH
  3  FAIL_TIMEOUT
  4  FAIL_CANCELED
  5  FAIL_ADJUST_EXHAUSTED  조절 소진
  6  FAIL_ROBOT_ERROR
  7  FAIL_DETUNED_ON_VERIFY 검증 시 틀어짐
```

`has_*`가 false면 해당 주파수는 무효. DB에 NULL.

### TuningSettings

```
float32 tolerance_cents       10.0
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
string                  job_id
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
```

```
phase
   0  IDLE            6  STRUM
   1  MEASURE_READY   7  RETURN
   2  PLUCK           8  DONE
   3  ANALYZE         9  PAUSED
   4  ADJUST_PEG     10  CANCELED
   5  VERIFY         11  ERROR

overall_result
  0  PENDING   1  SUCCESS   2  PARTIAL   3  FAILED
```

QoS: RELIABLE / TRANSIENT_LOCAL / depth 1. 발행·구독 양쪽 동일 설정.

---

## Service

### CommandTuning
`/tuning/start` `/tuning/pause` `/tuning/resume` `/tuning/cancel` — task_manager

```
Request   (없음)
Response  bool    accepted
          string  message
```

호출된 이름이 곧 명령. 접수 여부만 응답하고 진행은 Topic으로.

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
정상 종료 시에만 호출. 취소·부분 결과는 저장 안 함.

---

## 측정 1회 순서

```
1  TM → AA   MeasurePitch Goal
2  AA → TM   ready = true              onset_timeout 카운트 시작
3  TM → RC   PlayString Goal
4  RC        접근 → 발현 → 5mm 후퇴
5  AA        onset 검출                timeout 카운트 시작
6  RC → TM   PlayString Result
7  AA → TM   MeasurePitch Result
8  TM        ID 대조 → 센트 판정
```

6과 7의 순서는 보장되지 않는다.

---

## 확인

```
ros2 interface show tunio_interfaces/action/MeasurePitch
ros2 action send_goal /robot/play_string \
  tunio_interfaces/action/PlayString "{string_number: 6}"
ros2 topic info /tuning/state --verbose
```
