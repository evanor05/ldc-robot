"""Bounded incremental multipart parser; never re-encodes camera JPEG bytes."""
import re
from urllib.parse import urlsplit

MAX_FRAME = 4 * 1024 * 1024
MAX_HEADERS = 8192


def validate_url(url):
    parsed = urlsplit(url)
    if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError('摄像头地址必须是不含凭据的 HTTP(S) URL')
    return url


class MjpegParser:
    def __init__(self, content_type, limit=MAX_FRAME):
        match = re.search(r'boundary\s*=\s*"?([^";\s]+)', content_type, re.I)
        if not match or not 1 <= len(match[1]) <= 70:
            raise ValueError('MJPEG 响应缺少有效 boundary')
        self.boundary = b'--' + match[1].encode('ascii')
        self.buffer = bytearray()
        self.limit = limit
        self.length = None

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            if self.length is None:
                start = self.buffer.find(self.boundary + b'\r\n')
                if start < 0:
                    if len(self.buffer) > MAX_HEADERS:
                        raise ValueError('MJPEG boundary 超限或响应已截断')
                    break
                end = self.buffer.find(b'\r\n\r\n', start)
                if end < 0:
                    if len(self.buffer) > MAX_HEADERS:
                        raise ValueError('MJPEG part header 超限')
                    break
                if end-start > MAX_HEADERS:
                    raise ValueError('MJPEG part header 超限')
                headers = self.buffer[start+len(self.boundary)+2:end].decode('ascii')
                parts = dict(line.lower().split(':', 1) for line in headers.split('\r\n') if ':' in line)
                if parts.get('content-type', '').strip() != 'image/jpeg':
                    raise ValueError('MJPEG part 不是 JPEG')
                try:
                    length = int(parts.get('content-length', ''))
                except ValueError as error:
                    raise ValueError('MJPEG 缺少 Content-Length') from error
                if not 4 <= length <= self.limit:
                    raise ValueError('MJPEG frame 超限')
                self.length = length
                del self.buffer[:end+4]
            if len(self.buffer) < self.length:
                break
            frame = bytes(self.buffer[:self.length])
            del self.buffer[:self.length]
            self.length = None
            if not frame.startswith(b'\xff\xd8') or not frame.endswith(b'\xff\xd9'):
                raise ValueError('JPEG frame 截断或标记无效')
            frames.append(frame)
        if len(self.buffer) > self.limit + MAX_HEADERS:
            raise ValueError('MJPEG buffer 超限')
        return frames

    def finish(self):
        if self.length is not None or bytes(self.buffer).strip() not in (b'', self.boundary+b'--'):
            raise ValueError('MJPEG frame 在结束前被截断')
