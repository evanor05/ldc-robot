#!/usr/bin/env python3
"""相机源网络分层诊断（单次有界探测，不启动识别器）。

现场问题：相机 reconnects=1058，但分不清是"TCP 连不上"、"能连但 HTTP 不通"、
"HTTP 200 却没有可解码帧"。这三种病因的处置完全不同（换网/换端口/换源），
混在一起只能靠猜。本探针把一次连接拆成三段分别计时，并给出 first_failed_stage。

纪律：
- **200 不等于首帧**：HTTP 响应成功但拿不到完整 JPEG，仍算 frame 阶段失败；
- 单次有界：每段都有超时，不重试、不压测、不重启识别器、不改固件；
- 未进入的阶段标 not_attempted，不用 0 或 null 冒充"很快"；
- 只报告本次采样的失败分布，历史 reconnects 计数不做事后归因。

判据是"首帧可解码"，不是"起止标记齐全"：SOI+垃圾+EOI 的假帧在浏览器里同样不会显示，
必须判失败。解码用 Pillow；本机没有 Pillow 时该阶段标 unverified（**不判健康**），
因为无法证明可解码就不该给出"相机正常"的结论。
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / 'web_console' / 'ros_config.json'
SOI = b'\xff\xd8'
EOI = b'\xff\xd9'


def _now_ms(start: float) -> float:
    return round((time.monotonic() - start) * 1000, 1)


def _decode_ok(frame: bytes) -> bool | None:
    """有 Pillow 就顺手验证可解码性；没有就返回 None（表示未验证，不是失败）。"""
    try:
        from PIL import Image  # noqa: PLC0415 - 可选依赖
    except ImportError:
        return None
    try:
        import io  # noqa: PLC0415

        with Image.open(io.BytesIO(frame)) as image:
            image.load()
        return True
    except Exception:  # noqa: BLE001
        return False


def probe(url: str, timeout: float = 3.0, first_frame_timeout: float | None = None) -> dict:
    """对一次连接做 TCP / HTTP / 首帧三段探测；返回结构化结果，不抛异常。"""
    first_frame_timeout = timeout if first_frame_timeout is None else first_frame_timeout
    parsed = urlparse(url)
    result = {
        'url': url,
        'host': parsed.hostname,
        'port': parsed.port or 80,
        'stages': {'tcp': 'not_attempted', 'http': 'not_attempted', 'frame': 'not_attempted'},
        'tcpConnectMs': None,
        'httpResponseMs': None,
        'firstFrameMs': None,
        'httpStatus': None,
        'bytesRead': 0,
        'framesSeen': 0,
        'decoded': None,
        'firstFailedStage': None,
        'detail': '',
    }
    if parsed.scheme != 'http' or not parsed.hostname:
        result.update(stages={'tcp': 'failed', 'http': 'not_attempted', 'frame': 'not_attempted'},
                      firstFailedStage='tcp', detail=f'只支持 http://host:port/path，收到 {url}')
        return result

    # ---- 1) TCP 建连 ----
    started = time.monotonic()
    try:
        connection = socket.create_connection((result['host'], result['port']), timeout=timeout)
    except OSError as error:
        result.update(stages={'tcp': 'failed', 'http': 'not_attempted', 'frame': 'not_attempted'},
                      tcpConnectMs=_now_ms(started), firstFailedStage='tcp',
                      detail=f'TCP 建连失败：{error}')
        return result
    result['tcpConnectMs'] = _now_ms(started)
    result['stages']['tcp'] = 'ok'

    try:
        # ---- 2) HTTP 响应头 ----
        path = parsed.path or '/'
        if parsed.query:
            path += '?' + parsed.query
        request = (f'GET {path} HTTP/1.1\r\nHost: {result["host"]}:{result["port"]}\r\n'
                   'Accept: */*\r\nConnection: close\r\n\r\n').encode()
        http_started = time.monotonic()
        connection.settimeout(timeout)
        try:
            connection.sendall(request)
            header = b''
            while b'\r\n\r\n' not in header:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                header += chunk
                if time.monotonic() - http_started > timeout:
                    raise TimeoutError(f'HTTP 响应头在 {timeout}s 内未完成')
        except (OSError, TimeoutError) as error:
            result.update(stages={**result['stages'], 'http': 'failed'},
                          httpResponseMs=_now_ms(http_started), firstFailedStage='http',
                          detail=f'HTTP 响应失败：{error}')
            return result
        if not header:
            result.update(stages={**result['stages'], 'http': 'failed'},
                          httpResponseMs=_now_ms(http_started), firstFailedStage='http',
                          detail='连接建立后没有任何 HTTP 响应')
            return result

        result['httpResponseMs'] = _now_ms(http_started)
        head, _, body = header.partition(b'\r\n\r\n')
        status_line = head.split(b'\r\n', 1)[0].decode('latin1', 'replace')
        try:
            result['httpStatus'] = int(status_line.split()[1])
        except (IndexError, ValueError):
            result['httpStatus'] = None
        if result['httpStatus'] != 200:
            result.update(stages={**result['stages'], 'http': 'failed'}, firstFailedStage='http',
                          detail=f'HTTP 状态非 200：{status_line[:80]}')
            return result
        result['stages']['http'] = 'ok'

        # ---- 3) 首个完整 JPEG 帧 ----
        frame_started = time.monotonic()
        buffer = bytearray(body)
        result['bytesRead'] = len(buffer)
        deadline = frame_started + first_frame_timeout
        while True:
            start = buffer.find(SOI)
            if start >= 0:
                end = buffer.find(EOI, start + 2)
                if end >= 0:
                    frame = bytes(buffer[start:end + 2])
                    decoded = _decode_ok(frame)
                    result['firstFrameMs'] = _now_ms(frame_started)
                    result['framesSeen'] = 1
                    result['decoded'] = decoded
                    if decoded is True:
                        result['stages']['frame'] = 'ok'
                        result['detail'] = '首帧完整且可解码'
                    elif decoded is False:
                        # 起止标记齐全但解不出来（例如中断流/错位拼接）：
                        # 浏览器和识别器都会失败，不能算有画面。
                        result.update(stages={**result['stages'], 'frame': 'failed'},
                                      firstFailedStage='frame',
                                      detail='JPEG 起止标记齐全但无法解码（不是有效图像）')
                    else:
                        # 缺解码器时无法确认"可解码"：不判健康，也不假装失败。
                        result.update(stages={**result['stages'], 'frame': 'unverified'},
                                      firstFailedStage='frame',
                                      detail='本机没有 JPEG 解码器，无法确认首帧可解码（不判健康；'
                                             '可安装 Pillow 后复测）')
                    return result
            if time.monotonic() > deadline:
                result.update(stages={**result['stages'], 'frame': 'failed'}, firstFailedStage='frame',
                              detail=(f'HTTP 200 但 {first_frame_timeout}s 内没有完整 JPEG 帧'
                                      f'（已收 {result["bytesRead"]} 字节）；'
                                      '200 不等于有画面'))
                return result
            connection.settimeout(max(0.05, deadline - time.monotonic()))
            try:
                chunk = connection.recv(8192)
            except (socket.timeout, TimeoutError):
                # 单次 recv 超时 ≠ 连接关闭：继续等到总截止时间，否则会把"源很慢"
                # 误报成"连接已断"，两种病因的处置完全不同。
                continue
            except OSError:
                chunk = b''
            if not chunk:
                result.update(stages={**result['stages'], 'frame': 'failed'}, firstFailedStage='frame',
                              detail=(f'连接关闭前未收到完整 JPEG 帧（已收 {result["bytesRead"]} 字节）'))
                return result
            buffer += chunk
            result['bytesRead'] = len(buffer)
    finally:
        try:
            connection.close()
        except OSError:
            pass


def configured_url() -> str:
    try:
        config = json.loads(DEFAULT_CONFIG.read_text('utf-8'))
    except (OSError, ValueError):
        return ''
    return str((config.get('camera') or {}).get('stream_url') or '').strip()


def render(result: dict) -> str:
    def stage(name: str) -> str:
        value = result['stages'][name]
        return {'ok': 'OK', 'failed': 'FAIL', 'unverified': '未验证',
                'not_attempted': '未尝试'}[value]

    def ms(value) -> str:
        return '—' if value is None else f'{value:.0f}ms'

    lines = [f"相机源分层探测：{result['url']}",
             f"  1) TCP 建连   [{stage('tcp')}]   {ms(result['tcpConnectMs'])}",
             f"  2) HTTP 响应  [{stage('http')}]   {ms(result['httpResponseMs'])}"
             + (f"  状态 {result['httpStatus']}" if result['httpStatus'] is not None else ''),
             f"  3) 首帧完整  [{stage('frame')}]   {ms(result['firstFrameMs'])}"
             f"   已收 {result['bytesRead']} 字节"]
    if result['firstFailedStage']:
        lines.append(f"  首个失败阶段：{result['firstFailedStage']}")
    if result['detail']:
        lines.append(f"  说明：{result['detail']}")
    return '\n'.join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='相机源网络分层诊断（单次有界探测）')
    parser.add_argument('--url', default=None, help='http://host:port/path；缺省读 ros_config.json 的 camera.stream_url')
    parser.add_argument('--timeout', type=float, default=3.0)
    parser.add_argument('--first-frame-timeout', type=float, default=None)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)

    url = args.url or configured_url()
    if not url:
        print('没有相机地址：请给 --url 或在 web_console/ros_config.json 配置 camera.stream_url',
              file=sys.stderr)
        return 2
    result = probe(url, timeout=args.timeout, first_frame_timeout=args.first_frame_timeout)
    print(json.dumps(result, ensure_ascii=False, indent=2) if args.json else render(result))
    return 0 if result['stages']['frame'] == 'ok' else 1


if __name__ == '__main__':
    raise SystemExit(main())
