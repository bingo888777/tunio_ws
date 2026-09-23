"""Action 취소 가드와 독립 ROS 정지 경로. 실제 이동 명령은 이 모듈에서 보내지 않는다."""
import threading
import time


class MotionCancelled(Exception):
    pass


class GuardedAPI:
    def __init__(self, api, check):
        self.api = api
        self.check = check

    def __getattr__(self, name):
        value = getattr(self.api, name)
        if not callable(value):
            return value

        def call(*args, **kwargs):
            # 취소 후 힘/순응 해제는 허용하되 이동·그리퍼 열기는 허용하지 않는다.
            cleanup = name in ('release_force', 'release_compliance_ctrl')
            if not cleanup:
                self.check()
            result = value(*args, **kwargs)
            if not cleanup:
                self.check()
            return result
        return call


class CancelSession:
    def __init__(self):
        self.requested = threading.Event()
        self.work_done = threading.Event()
        self.stop_done = threading.Event()
        self.error = None

    def check(self):
        if self.requested.is_set():
            raise MotionCancelled('사용자 취소 — 후속 명령 차단')


def request_and_confirm_stop(stop, check_motion, session, timeout, interval):
    """stop(mode=2) 성공 + 작업 코드 이탈 + IDLE(0) 확인 전에는 성공하지 않는다.

    명령 전 검사와 실제 서비스 전송 사이에 취소가 끼는 경합을 위해 작업 코드가
    끝날 때까지 stop을 반복한다. 성공 응답만으로 물리 정지를 추정하지 않는다.
    """
    deadline = time.monotonic() + timeout

    def call(client, request):
        if not client.service_is_ready():
            raise RuntimeError('정지/상태 서비스 연결 안 됨')
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(max(0, deadline - time.monotonic())):
            raise RuntimeError('정지/상태 서비스 응답 시간 초과')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('정지/상태 서비스 실패 응답')
        return response

    try:
        from dsr_msgs2.srv import CheckMotion, MoveStop
        while time.monotonic() < deadline:
            call(stop, MoveStop.Request(stop_mode=2))
            if session.work_done.is_set():
                status = call(check_motion, CheckMotion.Request()).status
                if status == 0:
                    return
                if status not in (1, 2):
                    raise RuntimeError(f'알 수 없는 CheckMotion 상태: {status}')
            time.sleep(min(interval, max(0, deadline - time.monotonic())))
        raise RuntimeError('정지 완료 확인 시간 초과')
    except Exception as exc:
        session.error = str(exc)
    finally:
        session.stop_done.set()
