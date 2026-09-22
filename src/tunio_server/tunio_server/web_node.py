"""
web_node — 관제 웹. hmi_node(PyQt)를 대체한다. 담당: 정동휘.

브라우저 화면은 기획서 10.1 의 탭 3개(실시간 작업 / 작업 이력 / 설정)를 그대로 옮긴 것이다.
판정은 센트지만 화면 표시는 Hz 다(기획 2.2).

구조
  rclpy 노드 하나가 ROS 쪽(/tuning/state 구독 + Service 클라이언트 7개)을 맡고,
  파이썬 표준 라이브러리 http.server 를 별도 스레드에서 돌려 브라우저에 붙인다.
  외부 웹 프레임워크를 쓰지 않는다 — 서버 PC 에 아무것도 설치하지 않고 돌아야 한다.

  ⚠ 웹서버를 executor 와 같은 스레드에 두면 안 된다. 블로킹 서버가 executor 를 막으면
  /tuning/state 수신이 멈춘다. 그래서 serve_forever 는 daemon 스레드에서 돌리고
  메인 스레드는 MultiThreadedExecutor.spin() 만 한다.

브라우저에 상태를 밀어 주는 방식은 SSE(Server-Sent Events)다. WebSocket 과 달리
표준 라이브러리만으로 되고, 상태는 서버 → 브라우저 한 방향이면 충분하다.
명령은 별도로 POST 로 받는다.

  ⚠ /tuning/state 의 transient local QoS 는 원래 "늦게 켠 UI 가 마지막 상태를 받는다"가
  목적이었다. 웹에서는 브라우저가 DDS 에 붙지 않으므로 그 혜택을 못 받는다.
  이 노드가 최신 상태를 들고 있다가 브라우저 최초 접속 시 직접 내려준다(_latest_state).
  QoS 자체는 이 노드가 재시작했을 때 마지막 상태를 복구하는 용도로 여전히 필요하다.

  ⚠ task_manager 는 /tuning/state 를 변화 시에만 발행한다. 주기 갱신이 아니므로
  화면의 '갱신 시각'도 변화가 있을 때만 움직인다. 정상이다.
"""
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import re
import threading
from urllib.parse import parse_qs, urlparse
import webbrowser

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)

from tunio_interfaces.msg import TuningSettings, TuningState
from tunio_interfaces.srv import CommandTuning, GetHistory, GetSettings, SetSettings

# 명세 5절: task_manager 쪽 발행 QoS 와 값이 같아야 매칭된다.
STATE_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# 화면 문구. 열거값의 뜻은 tunio_interfaces 의 메시지 정의가 기준이다.
# 번호를 바꾸면 조용히 틀리므로, 상수가 추가되면 여기도 같이 추가한다.
PHASE_LABELS = {
    TuningState.PHASE_IDLE: '대기',
    TuningState.PHASE_MEASURE_READY: '측정 준비',
    TuningState.PHASE_PLUCK: '발현',
    TuningState.PHASE_ANALYZE: '분석',
    TuningState.PHASE_ADJUST_PEG: '페그 조절',
    TuningState.PHASE_VERIFY: '최종 검증',
    TuningState.PHASE_STRUM: '화음 마무리',
    TuningState.PHASE_RETURN: '대기 자세 복귀',
    TuningState.PHASE_DONE: '완료',
    TuningState.PHASE_PAUSED: '일시정지',
    TuningState.PHASE_CANCELED: '취소',
    TuningState.PHASE_ERROR: '오류',
    TuningState.PHASE_REGISTER_PEG: '페그 초기 등록',
}
OVERALL_LABELS = {
    TuningState.OVERALL_PENDING: '판정 전',
    TuningState.OVERALL_SUCCESS: '전체 성공',
    TuningState.OVERALL_PARTIAL: '일부 성공',
    TuningState.OVERALL_FAILED: '전체 실패',
}
RESULT_LABELS = {0: '대기', 1: '성공', 2: '실패'}
FAIL_LABELS = {
    0: '',
    1: '소리 부족',
    2: '주파수 판별 불가',
    3: '응답 시간 초과',
    4: '취소로 중단',
    5: '조절 횟수 소진',
    6: '로봇 동작 실패',
    7: '검증 시 틀어짐',
}

# 설정 '기본값 초기화' 버튼이 되돌릴 값(기획 10.1).
# task_manager.__init__ 의 초기값과 같아야 한다. 둘 다 임시값이다(CLAUDE.md 4·13절).
DEFAULT_SETTINGS = {
    'tolerance_cents': 10.0,
    'max_adjust_count': 5,
    'max_measure_retry': 3,
    'measure_timeout_sec': 10.0,
    'onset_timeout_sec': 10.0,
}
SETTING_FIELDS = tuple(DEFAULT_SETTINGS)

# 내보낼 정적 파일. 하위 디렉터리와 '..' 를 허용하지 않는다.
STATIC_RE = re.compile(r'^/[\w.-]+\.(?:css|js|svg|png|webp)$')


def _stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _string_result_to_dict(r):
    """메시지를 dict 로 바꾼다. has_* 가 false 인 주파수는 None 으로 낸다(0Hz 를 쓰지 않는다)."""
    return {
        'string_number': r.string_number,
        'target_hz': r.target_hz,
        'tuned_hz': r.tuned_hz if r.has_tuned_hz else None,
        'verified_hz': r.verified_hz if r.has_verified_hz else None,
        'final_error_cents': r.final_error_cents,
        'adjust_count': r.adjust_count,
        'result': r.result,
        'result_label': RESULT_LABELS.get(r.result, '알 수 없음(%d)' % r.result),
        'failure_reason': r.failure_reason,
        'failure_label': FAIL_LABELS.get(r.failure_reason,
                                         '알 수 없음(%d)' % r.failure_reason),
    }


def _settings_to_dict(s):
    return {f: getattr(s, f) for f in SETTING_FIELDS}


def _state_to_dict(msg):
    return {
        'stamp': _stamp_to_float(msg.stamp),
        'job_id': msg.job_id,
        'phase': msg.phase,
        'phase_label': PHASE_LABELS.get(msg.phase, '알 수 없음(%d)' % msg.phase),
        'current_string': msg.current_string,
        'target_hz': msg.target_hz,
        'measured_hz': msg.measured_hz,
        'measurement_valid': msg.measurement_valid,
        'error_hz': msg.error_hz,
        'adjust_count': msg.adjust_count,
        'results': [_string_result_to_dict(r) for r in msg.results],
        'overall_result': msg.overall_result,
        'overall_label': OVERALL_LABELS.get(msg.overall_result,
                                            '알 수 없음(%d)' % msg.overall_result),
        'active_settings': _settings_to_dict(msg.active_settings),
    }


class ServiceUnavailable(Exception):
    """서비스 서버가 없거나 응답이 없다. 화면에 그대로 알린다."""


class WebNode(Node):

    def __init__(self):
        super().__init__('web_node')
        self.declare_parameter('port', 8080)
        self.declare_parameter('open_browser', True)
        self.declare_parameter('history_limit', 20)
        self.declare_parameter('service_timeout_sec', 5.0)

        cb = ReentrantCallbackGroup()
        self.create_subscription(TuningState, '/tuning/state', self._on_state,
                                 STATE_QOS, callback_group=cb)

        self.cli_command = {
            name: self.create_client(CommandTuning, '/tuning/%s' % name, callback_group=cb)
            for name in ('start', 'pause', 'resume', 'cancel')
        }
        self.cli_get_settings = self.create_client(
            GetSettings, '/tuning/get_settings', callback_group=cb)
        self.cli_set_settings = self.create_client(
            SetSettings, '/tuning/set_settings', callback_group=cb)
        self.cli_history = self.create_client(GetHistory, '/tuning/get_history',
                                              callback_group=cb)

        # ⚠ 속성 이름에 주의. rclpy 의 Node 는 self._clients / _publishers / _services /
        # _subscriptions / _timers / _parameters 를 내부에서 쓴다. 같은 이름을 쓰면
        # Service 클라이언트 목록이 조용히 날아가고 응답이 영원히 오지 않는다.
        self._state_lock = threading.Lock()
        self._latest_state = None    # 브라우저 최초 접속 시 내려줄 최신 상태
        self._sse_queues = []        # SSE 연결마다 큐 하나

    # ------------------------------------------------------------------ ROS 쪽

    def _on_state(self, msg):
        data = _state_to_dict(msg)
        with self._state_lock:
            self._latest_state = data
            targets = list(self._sse_queues)
        for q in targets:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass   # 느린 브라우저 하나가 노드를 붙잡지 않게 한다

    def latest_state(self):
        with self._state_lock:
            return self._latest_state

    def add_client(self):
        q = queue.Queue(maxsize=32)
        with self._state_lock:
            self._sse_queues.append(q)
        return q

    def remove_client(self, q):
        with self._state_lock:
            if q in self._sse_queues:
                self._sse_queues.remove(q)

    def _call(self, client, request, what):
        """
        Service 호출. HTTP 핸들러 스레드에서 불린다.

        여기서 spin 하면 안 된다(메인 스레드가 이미 spin 중이다). call_async 로 던지고
        Future 완료를 Event 로 기다린다 — task_manager 가 Action 을 기다리는 방식과 같다.
        """
        timeout = float(self.get_parameter('service_timeout_sec').value)
        if not client.wait_for_service(timeout_sec=1.0):
            raise ServiceUnavailable('%s 서비스를 찾을 수 없다 (노드가 떠 있는지 확인)' % what)
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            raise ServiceUnavailable('%s 응답이 %.1f초 안에 오지 않았다' % (what, timeout))
        return future.result()

    # ------------------------------------------------------------------ 화면이 쓰는 동작

    def command(self, name):
        res = self._call(self.cli_command[name], CommandTuning.Request(), name)
        return {'accepted': res.accepted, 'message': res.message}

    def get_settings(self):
        res = self._call(self.cli_get_settings, GetSettings.Request(), 'get_settings')
        return {'settings': _settings_to_dict(res.settings), 'defaults': DEFAULT_SETTINGS}

    def set_settings(self, values):
        msg = TuningSettings()
        for f in SETTING_FIELDS:
            if f not in values:
                raise ValueError('%s 값이 없다' % f)
        # 정수 필드에 실수가 들어오면 메시지 할당에서 터지므로 여기서 맞춘다.
        msg.tolerance_cents = float(values['tolerance_cents'])
        msg.max_adjust_count = int(values['max_adjust_count'])
        msg.max_measure_retry = int(values['max_measure_retry'])
        msg.measure_timeout_sec = float(values['measure_timeout_sec'])
        msg.onset_timeout_sec = float(values['onset_timeout_sec'])

        req = SetSettings.Request()
        req.settings = msg
        res = self._call(self.cli_set_settings, req, 'set_settings')
        return {'accepted': res.accepted, 'message': res.message}

    def get_history(self, job_id='', limit=None):
        req = GetHistory.Request()
        req.job_id = job_id
        req.limit = int(limit if limit is not None
                        else self.get_parameter('history_limit').value)
        res = self._call(self.cli_history, req, 'get_history')
        return {
            'found': res.found,
            'jobs': [{'job_id': j.job_id,
                      'finished_at': _stamp_to_float(j.finished_at),
                      'overall_result': j.overall_result,
                      'overall_label': OVERALL_LABELS.get(
                          j.overall_result, '알 수 없음(%d)' % j.overall_result)}
                     for j in res.jobs],
            'strings': [_string_result_to_dict(r) for r in res.strings],
            'settings': _settings_to_dict(res.settings),
        }


class Handler(BaseHTTPRequestHandler):
    """정적 파일 + JSON API + SSE. 요청마다 스레드 하나(ThreadingHTTPServer)."""

    server_version = 'TunioWeb/0.1'
    protocol_version = 'HTTP/1.1'

    def __init__(self, *args, node=None, web_dir=None, **kwargs):
        self.node = node
        self.web_dir = web_dir
        super().__init__(*args, **kwargs)

    # 기본 로그는 요청마다 stderr 에 찍혀 런치 화면을 덮는다. ROS 로거로 낮춰 보낸다.
    def log_message(self, fmt, *args):
        self.node.get_logger().debug('%s %s' % (self.address_string(), fmt % args))

    # -------------------------------------------------------------- 응답 헬퍼

    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, payload, code=200):
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self._send(code, body, 'application/json; charset=utf-8')

    def _error(self, code, message):
        self._json({'error': message}, code=code)

    def _static(self, name):
        types = {'.html': 'text/html; charset=utf-8',
                 '.css': 'text/css; charset=utf-8',
                 '.js': 'application/javascript; charset=utf-8',
                 '.svg': 'image/svg+xml; charset=utf-8',
                 '.png': 'image/png',
                 '.webp': 'image/webp'}
        path = (self.web_dir / name).resolve()
        if self.web_dir not in path.parents or not path.is_file():
            self._error(404, '%s 없음' % name)
            return
        self._send(200, path.read_bytes(), types.get(path.suffix, 'application/octet-stream'))

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length') or 0)
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode('utf-8'))

    # -------------------------------------------------------------- 라우팅

    def do_GET(self):
        url = urlparse(self.path)
        path = url.path
        try:
            if path in ('/', '/index.html'):
                self._static('index.html')
            elif STATIC_RE.match(path):
                # 파일 이름만 받는다. 하위 경로·상위 이동(..)은 정규식에서 걸린다.
                # 실제 경로가 web_dir 안인지는 _static 이 한 번 더 확인한다.
                self._static(path.lstrip('/'))
            elif path == '/api/state':
                self._json({'state': self.node.latest_state()})
            elif path == '/api/events':
                self._events()
            elif path == '/api/settings':
                self._json(self.node.get_settings())
            elif path == '/api/history':
                q = parse_qs(url.query)
                limit = q.get('limit', [None])[0]
                self._json(self.node.get_history(limit=limit))
            elif path.startswith('/api/history/'):
                self._json(self.node.get_history(job_id=path[len('/api/history/'):]))
            else:
                self._error(404, '없는 주소')
        except ServiceUnavailable as e:
            self._error(503, str(e))
        except BrokenPipeError:
            pass          # 브라우저가 먼저 끊었다. 정상 상황이다
        except Exception as e:      # noqa: BLE001 - 어떤 예외든 화면에 사유를 보여준다
            self.node.get_logger().error('GET %s 실패: %r' % (path, e))
            self._error(500, '서버 오류: %s' % e)

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            if path.startswith('/api/command/'):
                name = path[len('/api/command/'):]
                if name not in ('start', 'pause', 'resume', 'cancel'):
                    self._error(404, '없는 명령: %s' % name)
                    return
                self._json(self.node.command(name))
            elif path == '/api/settings':
                self._json(self.node.set_settings(self._read_json_body()))
            else:
                self._error(404, '없는 주소')
        except ServiceUnavailable as e:
            self._error(503, str(e))
        except (ValueError, KeyError, TypeError) as e:
            self._error(400, '입력이 잘못됐다: %s' % e)
        except BrokenPipeError:
            pass
        except Exception as e:      # noqa: BLE001
            self.node.get_logger().error('POST %s 실패: %r' % (path, e))
            self._error(500, '서버 오류: %s' % e)

    # -------------------------------------------------------------- SSE

    def _events(self):
        """
        상태 변화를 브라우저로 밀어 준다.

        최초 1건은 이 노드가 들고 있던 최신 상태를 보낸다(브라우저는 transient local
        혜택을 못 받는다). 이후에는 /tuning/state 가 올 때마다 보낸다.
        15초 동안 변화가 없으면 주석 한 줄을 보내 연결이 살아 있는지 확인한다 —
        task_manager 가 변화 시에만 발행하므로 대기 중에는 아무 것도 오지 않는다.
        """
        q = self.node.add_client()
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close')
            self.end_headers()

            initial = self.node.latest_state()
            if initial is not None:
                self._sse_send(initial)

            while True:
                try:
                    self._sse_send(q.get(timeout=15.0))
                except queue.Empty:
                    self.wfile.write(b': keepalive\n\n')
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass          # 탭을 닫았다
        finally:
            self.node.remove_client(q)

    def _sse_send(self, data):
        payload = json.dumps(data, ensure_ascii=False)
        self.wfile.write(('data: %s\n\n' % payload).encode('utf-8'))
        self.wfile.flush()


def _web_dir():
    """정적 파일 위치. colcon 이 share/tunio_server/web 으로 설치한다."""
    return Path(get_package_share_directory('tunio_server')) / 'web'


def main():
    rclpy.init()
    node = WebNode()
    port = int(node.get_parameter('port').value)
    web_dir = _web_dir().resolve()

    handler = partial(Handler, node=node, web_dir=web_dir)
    httpd = ThreadingHTTPServer(('127.0.0.1', port), handler)
    httpd.daemon_threads = True

    # 웹서버는 별도 스레드. 메인 스레드는 executor 만 돌린다(파일 머리말 참조).
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    url = 'http://localhost:%d' % port
    node.get_logger().info('관제 웹 시작: %s  (정적 파일 %s)' % (url, web_dir))
    if not web_dir.is_dir():
        node.get_logger().error(
            '정적 파일 디렉터리가 없다. colcon build 후 source install/setup.bash 했는지 확인')
    if bool(node.get_parameter('open_browser').value):
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
