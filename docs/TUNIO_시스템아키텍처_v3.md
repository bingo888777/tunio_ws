# TUNIO 시스템 아키텍처 v3 — PC 2대

> 2026-09-22. v2(PC 3대)를 대체한다. 도식은 `TUNIO_시스템아키텍처_v3.png`,
> 생성기는 `TUNIO_시스템아키텍처_v3.py` 다.

## 무엇이 바뀌었나

**PC 3대 → 2대.** 마이크 PC 와 서버 PC 를 한 대로 합쳤다.

```
v2                                v3
① 로봇 제어 PC   robot            ② 로봇 제어 PC   robot
② 서버 PC        server      →    ① 정정인 PC      server + audio
③ 마이크 PC      audio
```

**노드도 인터페이스도 안 바뀌었다.** 달라진 것은 **어느 PC 에서 도는가** 뿐이다.
`tunio_interfaces` 는 손대지 않았고 코드 변경도 없다.

## 왜 이게 통합에 유리한가

**ROS 를 실제로 건너는 것이 셋으로 줄었다.**

```
랜을 건넌다   /robot/play_string · /robot/adjust_peg · /robot/strum_all
한 PC 안      /audio/measure_pitch · /tuning/state · /tuning/* 서비스 전부
```

v2 에서는 `/audio/measure_pitch` 도 Wi-Fi 를 건넜다. 측정 응답 목표가 **2.2초**이고
그 예산에 통신 지연이 아직 안 들어가 있는데(CLAUDE.md 4절), **그 왕복이 없어졌다.**
남은 건 로봇 Action 셋뿐이고 이쪽은 원래 초 단위 동작이라 지연에 둔하다.

디스커버리가 붙어야 할 상대도 둘로 줄어 실패 지점이 하나 줄었다.

## 대신 생긴 것

- **정정인 PC 한 대에 노드 4개**가 돈다 — `audio_analyzer` · `task_manager` ·
  `result_store` · `web_node`. audio 는 실시간 녹음·분석이라 CPU 를 쓴다.
  통합 첫 회차에서 **측정 응답 시간이 2.2초를 넘는지 봐야 한다**
- **Discovery Server 를 띄우는 자리가 서버 PC 에서 정정인 PC 로 옮겨졌다**
- **로봇 구동음이 마이크에 더 가까워질 수 있다.** PC 배치가 바뀌면 마이크 위치도
  바뀐다. 돌출 문턱 20dB 근거(`docs/로봇구동음_측정결과.md`)가 그대로인지 봐야 한다

## 실행

### 정정인 PC — 마이크 + 서버 + 관제 웹

```bash
source /opt/ros/jazzy/setup.bash && source ~/tunio_ws/install/setup.bash
export ROS_DOMAIN_ID=10

ros2 launch tunio_bringup station.launch.py
```

옵션 — `device:=2`(마이크 장치) · `port:=9000`(웹 포트) · `open_browser:=false` ·
`db:=<경로>`.

이 런치는 `tunio_server/server.launch.py` 를 그대로 include 하고 `audio_analyzer`
만 따로 띄운다. **노드 정의를 복사하지 않는다** — 복사하면 원본이 바뀔 때 여기만
옛 설정으로 남는다.

### 로봇 제어 PC

```bash
# 터미널 1 — 두산 브링업
source /opt/ros/jazzy/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100

# 터미널 2 — 우리 노드
source /opt/ros/jazzy/setup.bash && source ~/tunio_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch tunio_robot robot.launch.py dry_run:=false
```

⚠ **`dry_run` 기본값이 true 다.** 안 주면 로봇이 안 움직인다. 일부러 그렇게 뒀다.

### 한 PC 에서 전부 (시험용)

```bash
ros2 launch tunio_bringup real_all.launch.py dry_run:=false gripper:=rg2
```

## 두 PC 가 서로를 못 볼 때

같은 `ROS_DOMAIN_ID` 를 export 했는지부터 본다. 그래도 안 보이면 Wi-Fi 가
멀티캐스트를 막는 것이다. Fast DDS Discovery Server 를 쓴다 —
**서버는 정정인 PC 에서 띄운다** (v2 에서는 서버 PC 였다).

```bash
# 정정인 PC
fastdds discovery -i 0 -l 0.0.0.0 -p 11811
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# 로봇 제어 PC
export ROS_DISCOVERY_SERVER=<정정인 PC IP>:11811
```

클라이언트가 서버에 자기 주소를 등록하므로 **고정 IP 가 필요한 것은 정정인 PC
하나뿐**이다.

⚠ `peg_cal.py` 처럼 **직접 두산 API 에 붙는 도구는 디스커버리 서버를 끄고** 써야
붙는 경우가 있다 (`unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT`).
`docs/페그실측_단독PC_런북.md` 4절 참조.

## 1차 통합

**절차는 `docs/TUNIO_1차통합_가이드.md` 에 있다.** 아래는 뼈대만 남긴다.

## 1차 통합 점검 순서

**두 PC 를 붙이기 전에 각자 혼자 도는지부터 본다.** 한꺼번에 켜면 안 될 때
원인이 네트워크인지 노드인지 안 갈린다.

### 0 · 빌드

```bash
cd ~/tunio_ws && git pull && colcon build && source install/setup.bash
```

⚠ **두 PC 모두에서 한다.** 2026-09-21 에 로봇 PC 빌드를 빼먹어 설치본의 `sol` 이
`null` 로 남았고, 5번은 허공을 헤매고 2번은 빈손 16.2mm 가 났다. 두 시간을 썼다.

### 1 · 정정인 PC 혼자

```bash
ros2 launch tunio_bringup station.launch.py
ros2 node list          # task_manager · result_store · web_node · audio_analyzer
```

브라우저에서 `http://localhost:8080` — 화면이 뜨고 진행 상황·설정·기록 세 곳이
다 채워지는지 본다.

### 2 · 로봇 PC 혼자

`docs/페그실측_줄별_명령어.md` 1·2절을 그대로 따른다. `set 반환 -1` 은 정상이다.

### 3 · 붙이기

```bash
# 정정인 PC 에서
ros2 node list          # 양쪽 노드가 다 보여야 한다
ros2 action list | grep -E "/robot/|/audio/"
```

`play_string` · `adjust_peg` · `strum_all` · `measure_pitch` **넷 다** 나와야 한다.

### 4 · 마른 시험 (로봇 안 움직임)

로봇 PC 를 `dry_run:=true`(기본값) 로 띄운 채 관제 웹에서 시작을 누른다.
**상태 기계가 끝까지 도는지**만 본다. 페그 캘리브레이션이 `measured: false` 라
조절 단계에서 `PHASE_ERROR` 로 멈추는데, **그게 의도한 동작이다.**

### 5 · 젖은 시험

`dry_run:=false`. **손이 비상정지에 닿는 자리에서** 한 줄만 먼저 돌린다.

## 통합 전에 막혀 있는 것

- **페그 캘리브레이션이 안 끝났다.** `peg_calibration.yaml` 의 `measured` 가
  `false` 라 조절 단계에서 멈춘다. 5·2번은 쟀고 **1번이 남았다**.
  5번은 `cents_per_deg` 불일치로 재측정이 필요하다 (#8)
- **#70 — 손목 감김.** 페그를 한 방향으로 여러 번 돌리면 J6 가 ±360도 한계까지
  감겨 `movel` 이 거부된다. **조율 루프도 같은 경로를 쓴다** — 줄당 최대 5회 ×
  `max_angle_deg` 60도면 한 방향 300도다. 통합 실기에서 만날 수 있다
- **발현과 페그 조절을 한 루프에서 번갈아 할 때의 자세·경로 간섭이 미검증**이다
  (CLAUDE.md 13절). 경유지는 페그 쪽만 검증됐다

## 문서 갱신 상태

- `CLAUDE.md` 6절 — v3 로 고쳤다
- `README.md` 구성·실행 — v3 로 고쳤다
- `docs/기획서_*_v1.2.md` — **안 고쳤다.** 버전이 박힌 산출물이라 그대로 둔다.
  거기 적힌 PC 3대는 9/21 까지의 구성이다
- `TUNIO_시스템아키텍처_v2.*` — 남겨 둔다. 비교용이다
