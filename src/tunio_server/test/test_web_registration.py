"""재등록 HTTP 경로와 ROS 서비스 브리지의 접수/거부 의미를 검증한다."""
from functools import partial
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from tunio_server.web_node import Handler, WebNode


@pytest.mark.parametrize('accepted', [True, False])
def test_reregister_http_routes_to_command_tuning(accepted):
    calls = []
    client = object()

    def call(actual_client, request, name):
        assert actual_client is client
        assert request.get_fields_and_field_types() == {}
        calls.append(name)
        return SimpleNamespace(accepted=accepted, message='접수' if accepted else '이미 진행 중')

    node = SimpleNamespace(cli_command={'reregister_pegs': client}, _call=call,
                           get_logger=lambda: Mock())
    node.command = lambda name: WebNode.command(node, name)
    server = ThreadingHTTPServer(('127.0.0.1', 0), partial(
        Handler, node=node, web_dir=Path(__file__).resolve().parents[1] / 'web'))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        base = 'http://127.0.0.1:%d' % server.server_port
        req = Request(base + '/api/command/reregister_pegs', method='POST')
        with urlopen(req, timeout=3) as response:
            data = json.load(response)
        assert data == {'accepted': accepted, 'message': '접수' if accepted else '이미 진행 중'}
        assert calls == ['reregister_pegs']
        with urlopen(base + '/registration.js', timeout=3) as response:
            assert b'RegistrationTracker' in response.read()
        with pytest.raises(HTTPError) as exc:
            urlopen(Request(base + '/api/command/unknown', method='POST'), timeout=3)
        assert exc.value.code == 404
        assert calls == ['reregister_pegs']
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
