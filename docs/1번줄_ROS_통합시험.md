# 1번 줄 전체 조율 — ROS 통합 시험

이 도구는 가짜 각도를 반복 회전하는 시험이 아니다. 실제 서버의 음정 판정·백래시 보정·
재시도·회전량 계산을 재사용하고, robot_controller와 audio_analyzer에 ROS Action을 보낸다.
1번 E4 목표는 329.6276Hz다. 이미 허용 오차 이내면 페그를 돌리지 않는 것이 정상이다.

순서: 1번 미등록 시 초기 등록 → 피크 집기·발현·마이크 측정 → 필요시 피크 반환·페그 조절
→ 같은 줄 재측정 → 최종 검증 → 피크 반환·대기 자세 복귀.
측정 실패/횟수 소진은 결과에 남긴다. 화음은 호출하지 않고, 일반 4줄 작업 DB에도 저장하지 않는다.

## 양쪽 PC 업데이트

같은 PR 브랜치를 양쪽 PC에 받아야 한다. 새 RegisterPeg/ReturnHome Action 때문에
서버·로봇 모두 인터페이스를 다시 빌드하고 기존 노드를 재시작한다.

```bash
cd ~/tunio_ws
source /opt/ros/jazzy/setup.bash
# 로봇 PC에서만 추가
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
colcon build --packages-select tunio_interfaces tunio_robot tunio_server
source install/setup.bash
```

마이크 PC에서는 기존 tunio_audio 설치도 필요하다. 필요하면 `colcon build --packages-select tunio_audio`.
두 PC의 모든 실행 터미널에서 다음 환경을 맞춘다. Discovery Server는 담당 PC에서 먼저 기동한다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=172.24.0.136:11811
export ROS_SUPER_CLIENT=TRUE
export ROS_DOMAIN_ID=10
```

## 로봇 PC

두산 브링업은 전체 PC 중 이 PC에서 하나만 실행한다. 이미 정상 실행 중이면 추가로 띄우지 않는다.

```bash
ros2 launch m0609_rg2_bringup bringup.launch.py \
  mode:=real host:=192.168.1.100 model:=m0609
```

다른 터미널에서 환경을 source하고:

```bash
ros2 run tunio_robot robot_controller --ros-args \
  -p dry_run:=false \
  -p teach_points:=$HOME/tunio_ws/src/tunio_robot/config/teach_points.yaml
```

피크는 거치대에 있고, 페그 손잡이는 기존 사전 정렬 조건을 만족해야 한다.
저장된 유효 페그 각도는 재사용하므로, 사람이 페그를 돌렸다면 기존 등록값을 신뢰하면 안 된다.

집기·반환 높이는 50mm, 발현 속도/가속도는 100/300(임시값)이다.
1번 페그 회전 속도/가속도는 기존 2°/s·4°/s²를 유지한다. 반대쪽 sol 적용은 보류다.
반환 로그의 `거치대 위`, `반환점`에서 목표/실제 XYZ를 확인한다.
현재 좌표 기준 상단 Z=214mm, 반환점 Z=164mm다. 오차 2mm 초과 또는 조회 실패 시 놓지 않는다.
이 검사는 피크 안착 여부를 감지하지 않는다.

## 서버·마이크 PC

기존 task_manager는 종료한다. station.launch.py로 함께 띄웠다면 그 런치를 종료하고
마이크만 아래처럼 따로 실행한다. 두 조율 제어기를 동시에 실행하지 않는다.

```bash
ros2 run tunio_audio audio_analyzer --ros-args -p device:=-1
```

`-1`은 시스템 기본 입력이다. USB 마이크가 기본이 아니면 확인한 장치 번호로 바꾼다.
이미 마이크만 정상 실행 중이면 중복 실행하지 않는다.

새 터미널에서 같은 source와 통신 환경을 적용한 후:

```bash
cd ~/tunio_ws
# 계획 확인: 로봇을 움직이지 않음
python3 tools/tune_one_string.py --string 1

# 실제 ROS Action 전송: dry_run=false 로봇이면 움직임
python3 tools/tune_one_string.py --string 1 --execute
```

기본 허용 오차 10센트, 조절 최대 5회, 측정 추가 재시도 3회는 기존 서버 기본값이다.
소리 대기는 피크 집기 시간을 포함해 시험 도구에서 40초로 둔다(실측 전 임시값).
`--onset-timeout 50`처럼 바꿀 수 있다. 일반 서버/UI의 10초 설정을 자동 변경하지 않는다.
페그 조절 요청 제한은 120초, 등록은 줄당 180초, 반환·복귀는 120초다.
시간 초과 시 로봇의 같은 결과를 추가 120초 기다린다. 종료 불명이면 후속 동작을 중단한다.

상태는 `/single_string_test/state`로 확인한다. 관제 웹의 일반 시작 버튼은 누르지 않는다.

```bash
ros2 topic echo /single_string_test/state
```

결과 JSON은 `~/.tunio/robot_data/*_single_string.json`에 남는다.
`--output /tmp/string1.json`으로 경로를 지정할 수 있다. 성공은 exit 0, 실패는 exit 1이다.
최종 검증 측정까지 성공하고 피크 반환·복귀도 성공해야 시험 성공이다.
Ctrl+C나 통신 단절 시 자동 회수하지 않는다. 로봇은 이미 수락한 동작을 계속할 수 있다.

## 로봇 없이 통신 흐름 확인

실제 장치와 분리된 도메인 101을 사용한다. 두 터미널 모두 설치 환경을 source하고
`unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT`, `export ROS_DOMAIN_ID=101`을 적용한다.

```bash
# 터미널 A
ros2 run tunio_server fake_hardware --ros-args -p mode:=sim \
  -p ready_delay_sec:=0.05 -p measure_delay_sec:=0.05 \
  -p play_delay_sec:=0.05 -p adjust_delay_sec:=0.05
```

```bash
# 터미널 B
python3 tools/tune_one_string.py --string 1 --execute \
  --calibration src/tunio_server/config/peg_calibration.sim.yaml \
  --output /tmp/string1_sim.json
```

sim 캘리브레이션은 지어낸 값이다. 통신·순서 검증일 뿐 실기 조율 성능 근거가 아니다.

### 페그 등록과 회전 분리

전체 통합은 6→5→2→1번 중 미등록 페그를 모두 등록한 뒤 피크를 집는다.
이 한 줄 시험은 1번만 등록한다. `AdjustPeg`는 등록을 하지 않으며,
미등록이면 이동 전에 실패한다. 등록 무효화 후에는 `RegisterPeg`를 다시 완료해야 한다.

등록과 회전 모두 현재 J1을 읽어 경유를 결정한다. 1·2번은 −30~+30도,
5·6번은 +150~+210도(경계 포함)에서만 생략한다. 그 밖에는 +90도 경유지를 사용한다.
−180도는 +180도로 환산하지 않는다. J1을 못 읽으면 경유를 시도하며,
이동에 필요한 관절값도 읽지 못하면 동작은 실패한다.
