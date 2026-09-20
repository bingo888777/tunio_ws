# TUNIO 관제 웹

작성 2026-09-19 · 담당 정동휘
대상: 팀 전체. 웹을 직접 고치지 않더라도 **화면을 띄워 쓰게 되므로** 읽어 두면 좋다.

---

## 0. 세 줄 요약

- 기존 **PyQt 화면(`hmi_node`)이 관제 웹(`web_node`)으로 바뀌었다.**
- **`ros2 run tunio_server web_node`** 를 띄우고 브라우저로 `http://localhost:8080` 에 접속한다.
- **새로 설치할 것이 없다.** 파이썬 표준 라이브러리만 쓴다.

---

## 1. 왜 바꿨나

PyQt 화면은 서버 PC에 붙은 모니터에서만 볼 수 있다. 시연할 때 화면을 띄우려면
그 PC 앞에 서야 하고, 노트북이나 다른 화면으로 옮기기 어렵다.

웹으로 바꾸면 브라우저만 있으면 된다. 지금은 보안상 **서버 PC의 localhost 에서만**
접속되게 막아 뒀지만, 필요하면 같은 Wi-Fi 안에서 열어 줄 수 있는 구조다.

바뀐 것은 화면뿐이다. **ROS 인터페이스(msg/srv/action) 13개는 하나도 바뀌지 않았다.**
로봇·오디오 쪽 코드는 이 변경과 무관하다.

---

## 2. 화면 보는 법

### 서버가 실제로 돌 때

```bash
cd ~/tunio_ws && source install/setup.bash
ros2 launch tunio_server server.launch.py
```

`task_manager`, `result_store`, `web_node` 가 함께 뜨고 브라우저가 자동으로 열린다.

포트를 바꾸거나 브라우저 자동 실행을 끄려면:

```bash
ros2 launch tunio_server server.launch.py port:=9000 open_browser:=false
```

### 로봇도 마이크도 없이 화면만 볼 때

```bash
ros2 run tunio_server mock_state &
ros2 run tunio_server web_node
```

`mock_state` 가 조율이 진행되는 척 상태를 흘려준다. 화면 배치나 표시 내용을 확인할 때 쓴다.
**판정식도 수렴 규칙도 진짜가 아니다.** 화면 확인 전용이다.

---

## 3. 대역 노드 두 개 — 헷갈리지 말 것

테스트용 가짜 노드가 두 개 있는데 **서로 반대 방향**이다.

**`mock_state`** — `task_manager` 를 대체한다. 위에서 화면용 상태를 흉내 낸다.
- 쓰는 곳: 화면 배치·표시 확인
- 진짜가 아닌 것: 조율 판정, 수렴 규칙
- **`task_manager` 와 같이 띄우면 안 된다.** 둘이 같은 토픽을 발행한다

**`fake_hardware`** — `robot_controller` 와 `audio_analyzer` 를 대체한다. 아래에서 하드웨어를 받친다.
- 쓰는 곳: 로봇·마이크 없이 **진짜 `task_manager` 와 진짜 `result_store`** 를 돌려보기
- 진짜인 것: 조율 판정, 재시도, 백래시 판단, DB 저장이 전부 실제 코드로 돈다
- **진짜 `robot_controller` / `audio_analyzer` 와 같이 띄우면 안 된다.** 같은 Action 이름을 둘이
  제공하면 어느 쪽이 받을지 알 수 없다

서버 흐름 전체를 검증하려면 이렇게 한다.

```bash
ros2 run tunio_server fake_hardware &
ros2 run tunio_server result_store &
ros2 run tunio_server task_manager &
ros2 run tunio_server web_node
```

`fake_hardware` 는 `mode` 파라미터로 상황을 바꾼다 — `on_target`(정상), `detuned`(음이 벗어남),
`fail`(측정 실패). 동작 지연도 파라미터로 조절한다.

---

## 4. 화면 구성

기획서 10.1 의 탭 3개를 그대로 옮겼다.

**실시간 작업**
- 명령 버튼 넷: 시작 / 일시정지 / 재개 / 취소
- 현재 단계 배지, 전체 판정 배지
- 지금 조율 중인 줄의 목표 Hz, 측정 Hz, 오차, 조절 횟수
- 줄별 결과 카드 (6·5·2·1)

**작업 이력**
- 최근 20건 목록. 클릭하면 그 작업의 줄별 상세가 펼쳐진다
- 그 작업에 적용됐던 설정 스냅샷도 같이 보인다

**설정**
- 허용 센트, 조절 횟수, 재측정 횟수, 측정 타임아웃, onset 제한 시간
- **작업 중에는 잠긴다.** `task_manager` 가 작업 중 변경을 거부하고, 화면도 그 상태를 표시한다

판정은 센트로 하지만 **화면 표시는 Hz** 다 (기획 2.2).

---

## 5. 담당별로 알아 둘 것

### 로봇 담당 (정묵·성현)

화면에 로봇 실패 사유가 그대로 뜬다. `AdjustPeg.Result.message` 와 `PlayString.Result.message` 에
담은 문구가 **사람이 읽는 화면 문구가 된다.** "파지 실패", "과토크", "걸림" 처럼 사람 말로 남겨 달라.
예외 클래스 이름만 올라오면 화면에서 원인을 알 수 없다.

DB에도 그대로 저장돼 나중에 이력에서 보인다.

### 오디오 담당 (정정인)

측정 실패 사유가 화면에 이름으로 뜬다. `FAIL_LOW_SIGNAL` 은 "소리 부족",
`FAIL_TIMEOUT` 은 "시간 초과" 같은 식이다.

오디오 파형·스펙트럼 시각화는 **지금 범위에 없다.** 넣으려면 `tunio_audio` 에 새 토픽이
필요하고, 그건 인터페이스 변경이라 팀 합의가 필요하다.

### 공통

`/tuning/state` 는 **변화가 있을 때만** 발행된다. 주기 발행이 아니다.
그래서 대기 중에는 화면의 갱신 시각이 멈춰 있는데 **정상이다.**

---

## 6. 구조 (고칠 사람만 보면 된다)

### 스레드 두 개

```
메인 스레드      MultiThreadedExecutor.spin()
                 ├─ /tuning/state 구독
                 └─ Service 클라이언트 7개

daemon 스레드    ThreadingHTTPServer.serve_forever()
                 └─ 브라우저 요청 처리
```

**웹서버를 executor 와 같은 스레드에 두면 안 된다.** 블로킹 서버가 executor 를 막으면
`/tuning/state` 수신이 멈춘다.

반대로 HTTP 핸들러 스레드에서 ROS Service 를 부를 때는 **거기서 `spin()` 을 부르면 안 된다**
(메인이 이미 돌고 있다). `call_async` 로 던지고 `threading.Event` 로 Future 완료를 기다린다.
`task_manager` 가 Action 을 기다리는 방식과 같다.

### 쓴 것

백엔드는 전부 표준 라이브러리다 — `http.server`, `json`, `queue`, `threading`, `urllib.parse`.
FastAPI 도 Flask 도 쓰지 않았다. **서버 PC 에 아무것도 설치하지 않고 돌아야 한다**는 것이 이유다.

프론트엔드도 바닐라다. `index.html` + `style.css` + `app.js` 세 파일이고 빌드 과정이 없다.
React 나 번들러를 쓰지 않는다.

### 상태를 브라우저로 미는 방식 — SSE

WebSocket 이 아니라 Server-Sent Events 를 쓴다. 표준 라이브러리만으로 되고,
상태는 **서버에서 브라우저로 한 방향**이면 충분하기 때문이다. 명령은 따로 POST 로 받는다.

SSE 연결마다 큐를 하나 두고, 상태가 오면 모든 큐에 `put_nowait` 한다. 큐가 차면 그냥 버린다 —
**느린 브라우저 탭 하나가 ROS 콜백을 붙잡지 못하게** 하려는 것이다.

15초 동안 변화가 없으면 keepalive 한 줄을 보낸다. `/tuning/state` 가 변화 시에만 오므로
대기 중에는 아무 것도 안 오고, 그러면 브라우저가 연결이 죽었는지 알 수 없다.

### transient local QoS 는 의미가 바뀌었다

원래 `/tuning/state` 의 transient local 은 **"늦게 켠 UI 가 마지막 상태를 즉시 받는다"** 가
목적이었다. PyQt 일 때는 UI 자체가 ROS 노드였으니 그게 통했다.

**웹에서는 브라우저가 DDS 에 붙지 않는다.** 그 혜택을 못 받는다.
그래서 `web_node` 가 최신 상태를 들고 있다가 브라우저가 접속하는 순간 직접 내려준다.

QoS 자체는 그대로 두는데, **존재 이유가 "브라우저 늦게 접속"에서 "web_node 가 재시작했을 때
복구"로 옮겨갔다.** 이 토픽의 QoS 를 건드릴 일이 생기면 이 점을 기억할 것.

### API

```
GET  /                     index.html
GET  /style.css, /app.js   정적 파일
GET  /api/state            현재 상태 1회 조회
GET  /api/events           SSE 스트림
GET  /api/settings         설정 조회
GET  /api/history          이력 목록
GET  /api/history/<job_id> 이력 상세
POST /api/command/<name>   start | pause | resume | cancel
POST /api/settings         설정 저장
```

상태 코드를 구분한다. 없는 주소 404, 잘못된 입력 400, **ROS 서비스를 못 찾거나 응답이 없으면
503**, 그 밖은 500 이다. 503 이 따로 있어서 `task_manager` 가 안 떠 있는 상황이 화면에 바로 보인다.

---

## 7. 주의할 점 두 가지

### 0Hz 를 측정값으로 쓰지 않는다

`StringResult` 의 `tuned_hz` / `verified_hz` 는 `has_tuned_hz` / `has_verified_hz` 가
false 일 때 **값이 0.0 으로 온다.** DB 에는 NULL 인데 ROS 메시지에 NULL 이 없어서 채워지는 값이다.

화면은 플래그를 먼저 보고 false 면 **'측정 불가'로 표시한다.** 0Hz 나 빈칸이 아니다 (기획 2.4).
이 메시지를 읽는 다른 코드를 짤 때도 같은 규칙을 지켜야 한다.

### 설정 폼 검증은 서버와 같은 조건이어야 한다

웹 폼의 입력 범위가 `task_manager._settings_violations()` 의 거부 조건과 어긋나면
**"폼은 통과했는데 서버가 거부"** 가 난다. 지금은 양쪽을 맞춰 뒀다.
허용 범위의 상·하한이 확정되면 **양쪽을 같이 고쳐야 한다.**

---

## 8. 구현 중 만난 rclpy 함정

다른 노드를 만들 때도 걸릴 수 있어 적어 둔다.

**`rclpy` 의 `Node` 는 `self._clients`, `_publishers`, `_services`, `_subscriptions`,
`_timers`, `_parameters` 를 내부에서 쓴다.** 노드 클래스에서 같은 이름의 속성을 쓰면
기존 목록이 조용히 덮여 사라진다.

에러가 나지 않는다. Service 를 불러도 응답이 영영 오지 않을 뿐이다. 원인을 찾기 어렵다.

---

## 9. 아직 안 된 것

- **이력 조회 실패 사유가 화면에 안 뜬다.** `GetHistory` 응답에 `message` 필드가 없어서
  DB 를 못 열었을 때와 저장된 작업이 없을 때를 구분할 수 없다. 필드를 추가할지는 미정이다.
  지금은 `result_store` 로그를 봐야 한다.
- **오디오 시각화** — 범위 밖. 넣으려면 인터페이스 변경 합의가 필요하다.

---

## 10. 파일 위치

```
src/tunio_server/
├── tunio_server/
│   ├── web_node.py       관제 웹 노드 (456줄)
│   └── mock_state.py     화면 확인용 모의 상태 발행기 (392줄)
├── web/
│   ├── index.html        화면 구조 (141줄)
│   ├── style.css         (353줄)
│   └── app.js            (394줄)
└── launch/server.launch.py   hmi_node 대신 web_node 를 띄운다
```
