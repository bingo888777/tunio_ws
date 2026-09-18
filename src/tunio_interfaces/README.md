# tunio_interfaces

TUNIO 기타 자동 조율 시스템의 ROS 2 인터페이스 정의 패키지.
노드 구현 코드는 들어가지 않는다. 메시지 정의만 둔다.

## 구성

### Action
- `MeasurePitch` — `/audio/measure_pitch` · task_manager → audio_analyzer
- `PlayString` — `/robot/play_string` · task_manager → robot_controller
- `AdjustPeg` — `/robot/adjust_peg` · task_manager → robot_controller
- `StrumAll` — `/robot/strum_all` · task_manager → robot_controller

### Service
- `CommandTuning` — `/tuning/start`, `/tuning/pause`, `/tuning/resume`, `/tuning/cancel`
- `GetSettings` — `/tuning/get_settings`
- `SetSettings` — `/tuning/set_settings`
- `GetHistory` — `/tuning/get_history`
- `SaveJob` — task_manager → result_store 저장 요청

### Topic
- `TuningState` — `/tuning/state` (reliable, depth 1, transient local)

보조 메시지: `TuningSettings`, `StringResult`, `JobSummary`

## 빌드

```bash
cd <워크스페이스 루트>
colcon build --packages-select tunio_interfaces
source install/setup.bash
ros2 interface show tunio_interfaces/action/MeasurePitch
```

## 규칙

- `job_id`는 `"20260922_143015"` 형식, task_manager가 생성
- `measurement_id`는 작업 안에서 1부터 증가
- `string_number`는 인덱스가 아니라 실제 줄 번호 (6, 5, 2, 1)
- 열거값을 바꿀 때는 기존 상수의 번호를 바꾸지 말고 뒤에 추가한다

## 미확정 사항

- 백래시 마무리를 task_manager가 결정하는 구조로 잡았다 (풀기 goal + 조이기 goal 두 번)
- 측정 타임아웃은 audio_analyzer가 직접 재고 `FAIL_TIMEOUT`을 반환한다
- 측정 타임아웃 카운트는 audio_analyzer가 오디오 onset(무음 → 소리)을 검출한 시점부터 센다
- onset이 `onset_timeout_sec` 안에 오지 않으면 `FAIL_LOW_SIGNAL`. 대기 상한 임시 10.0
- 허용 센트 기본값 미정 (시작값 ±10센트 권장)
