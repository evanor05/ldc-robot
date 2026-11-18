"""相机源分层探测的离线测试。

用项目自带的 scripts/vision/mock_camera.py 当假源（它有 normal/freeze/truncate/fragment
等可控故障模式），再补一个 HTTP 503 小服务和一个空端口。核心断言：
**HTTP 200 ≠ 有画面**，且失败必须落在正确的层（tcp / http / frame）。
"""
from __future__ import annotations

import base64
import contextlib
import json
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest
import urllib.request

REPO = Path(__file__).resolve().parents[2]
MOCK = REPO / 'scripts' / 'vision' / 'mock_camera.py'
sys.path.insert(0, str(REPO / 'scripts' / 'vision'))
import source_probe  # noqa: E402

# 真实可解码的最小 JPEG（2x2 红块，633 字节）。首帧判据是"可解码"，
# 所以成功用例必须用真图；起止标记齐全但内容是垃圾的帧必须被拒绝。
REAL_JPEG = base64.b64decode(
    '/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAoHBwgHBgoICAgLCgoLDhgQDg0NDh0VFhEYIx8lJCIfIiEmKzcvJik0KSEiMEExNDk7Pj4+JS5ESUM8SDc9Pjv/2wBDAQoLCw4NDhwQEBw7KCIoOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozv/wAARCAACAAIDASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAAAgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXGx8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREAAgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPExcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDmaKKK8g/RD//Z')
# 起止标记齐全、中间是垃圾的"假帧"
GARBAGE_JPEG = b'\xff\xd8' + b'\x11' * 64 + b'\xff\xd9'


def free_port() -> int:
    with contextlib.closing(socket.socket()) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


@contextlib.contextmanager
def mock_camera(tmp_path, fps=30):
    image = tmp_path / 'frame.jpg'
    image.write_bytes(FAKE_JPEG)
    port = free_port()
    process = subprocess.Popen(
        [sys.executable, str(MOCK), '--image', str(image), '--port', str(port), '--fps', str(fps)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/status', timeout=1) as response:
                    json.load(response)
                break
            except OSError:
                time.sleep(0.05)
        else:
            raise RuntimeError('mock_camera 未在 10s 内就绪')
        yield port

        def control(mode: str) -> None:
            urllib.request.urlopen(f'http://127.0.0.1:{port}/control/{mode}', timeout=2).read()
        # 通过附加属性暴露控制入口
        yield_control(port)
    finally:
        process.terminate()
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=5)


# ---- 让 with 语句也能拿到 control：用一个薄封装替代 ----
class _Source:
    def __init__(self, port: int):
        self.port = port
        self.url = f'http://127.0.0.1:{port}/stream'

    def control(self, mode: str) -> None:
        urllib.request.urlopen(f'http://127.0.0.1:{self.port}/control/{mode}', timeout=2).read()


@contextlib.contextmanager
def source(tmp_path, fps=30, image_bytes=REAL_JPEG):
    image = tmp_path / 'frame.jpg'
    image.write_bytes(image_bytes)
    port = free_port()
    process = subprocess.Popen(
        [sys.executable, str(MOCK), '--image', str(image), '--port', str(port), '--fps', str(fps)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 10
        ready = False
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{port}/status', timeout=1) as response:
                    json.load(response)
                ready = True
                break
            except OSError:
                time.sleep(0.05)
        assert ready, 'mock_camera 未就绪'
        yield _Source(port)
    finally:
        process.terminate()
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=5)


@contextlib.contextmanager
def http_server(status: int, body: bytes = b''):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            self.send_response(status)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            if body:
                self.wfile.write(body)

    port = free_port()
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f'http://127.0.0.1:{port}/stream'
    finally:
        server.shutdown()
        server.server_close()


def test_normal_source_reports_all_three_stages(tmp_path):
    with source(tmp_path) as src:
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=2)
    assert result['stages'] == {'tcp': 'ok', 'http': 'ok', 'frame': 'ok'}, result['detail']
    assert result['firstFailedStage'] is None
    assert result['httpStatus'] == 200
    assert result['tcpConnectMs'] is not None and result['httpResponseMs'] is not None
    assert result['firstFrameMs'] is not None
    assert result['bytesRead'] >= len(REAL_JPEG)


def test_http_200_without_frames_fails_at_frame_stage(tmp_path):
    """200 不等于有画面：能连、能拿响应头，但拿不到完整帧必须判 frame 失败。"""
    with source(tmp_path) as src:
        src.control('freeze')
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=0.5)
    assert result['stages']['tcp'] == 'ok'
    assert result['stages']['http'] == 'ok', 'HTTP 阶段本身是通的'
    assert result['stages']['frame'] == 'failed'
    assert result['firstFailedStage'] == 'frame'
    assert result['httpStatus'] == 200
    assert '200 不等于' in result['detail']


def test_truncated_frame_is_not_counted_as_first_frame(tmp_path):
    with source(tmp_path) as src:
        src.control('truncate')
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=0.6)
    assert result['stages']['frame'] == 'failed'
    assert result['firstFailedStage'] == 'frame'
    assert result['framesSeen'] == 0


def test_fragmented_stream_still_yields_first_frame(tmp_path):
    """分片写出的流也要能拼出完整帧（现场 TCP 小包很常见）。"""
    with source(tmp_path) as src:
        src.control('fragment')
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=2)
    assert result['stages']['frame'] == 'ok', result['detail']
    assert result['firstFrameMs'] is not None


def test_connection_refused_fails_at_tcp_and_rest_not_attempted():
    port = free_port()          # 没有任何进程监听
    result = source_probe.probe(f'http://127.0.0.1:{port}/stream', timeout=0.5)
    assert result['stages']['tcp'] == 'failed'
    assert result['stages']['http'] == 'not_attempted'
    assert result['stages']['frame'] == 'not_attempted'
    assert result['firstFailedStage'] == 'tcp'
    assert result['httpResponseMs'] is None, '没进入的阶段不得填 0 冒充很快'


def test_http_503_fails_at_http_stage():
    with http_server(503, b'busy') as url:
        result = source_probe.probe(url, timeout=2)
    assert result['stages']['tcp'] == 'ok'
    assert result['stages']['http'] == 'failed'
    assert result['stages']['frame'] == 'not_attempted'
    assert result['httpStatus'] == 503
    assert result['firstFailedStage'] == 'http'


def test_unsupported_scheme_is_reported_not_crashed():
    result = source_probe.probe('rtsp://127.0.0.1/stream')
    assert result['firstFailedStage'] == 'tcp'
    assert '只支持 http' in result['detail']


def test_cli_exit_code_reflects_frame_stage(tmp_path, capsys):
    with source(tmp_path) as src:
        ok = source_probe.main(['--url', src.url, '--timeout', '2', '--json'])
        good_out = capsys.readouterr().out
        src.control('freeze')
        bad = source_probe.main(['--url', src.url, '--timeout', '2',
                                 '--first-frame-timeout', '0.4', '--json'])
        bad_out = capsys.readouterr().out
    assert ok == 0 and bad == 1
    assert json.loads(good_out)['stages']['frame'] == 'ok'
    assert json.loads(bad_out)['firstFailedStage'] == 'frame'


def test_garbage_between_jpeg_markers_is_rejected(tmp_path):
    """起止标记齐全但内容不是图像：浏览器同样显示不出来，必须判 frame 失败。"""
    with source(tmp_path, image_bytes=GARBAGE_JPEG) as src:
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=2)
    assert result['stages']['tcp'] == 'ok'
    assert result['stages']['http'] == 'ok'
    assert result['stages']['frame'] == 'failed', '假帧不得判为首帧成功'
    assert result['firstFailedStage'] == 'frame'
    assert result['decoded'] is False
    assert '无法解码' in result['detail']


def test_missing_decoder_is_not_reported_healthy(tmp_path, monkeypatch):
    """本机没有解码器时无法证明"可解码"，只能标 unverified 且不算通过。"""
    monkeypatch.setattr(source_probe, '_decode_ok', lambda frame: None)
    with source(tmp_path) as src:
        result = source_probe.probe(src.url, timeout=2, first_frame_timeout=2)
        code = source_probe.main(['--url', src.url, '--timeout', '2', '--json'])
    assert result['stages']['frame'] == 'unverified'
    assert result['firstFailedStage'] == 'frame'
    assert '不判健康' in result['detail']
    assert code == 1, '缺解码器不得返回成功退出码'
