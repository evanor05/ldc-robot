"""Bounded binary framing shared by Python 3.14 ROS and Python 3.12 YOLO."""
import json
import os
import select
import struct
import time

MAX_META = 256 * 1024
MAX_IMAGE = 4 * 1024 * 1024


def read_exact(fd, length, deadline):
    data = bytearray()
    while len(data) < length:
        timeout = deadline-time.monotonic()
        if timeout <= 0 or not select.select([fd], [], [], timeout)[0]:
            raise TimeoutError('YOLO worker 响应超时')
        part = os.read(fd, min(65536, length-len(data)))
        if not part:
            raise EOFError('YOLO worker 已退出或 IPC 截断')
        data.extend(part)
    return bytes(data)


def read_packet(fd, timeout):
    deadline = time.monotonic()+timeout
    length = struct.unpack('<I', read_exact(fd, 4, deadline))[0]
    if not 1 <= length <= MAX_META:
        raise ValueError('worker metadata 超限')
    metadata = json.loads(read_exact(fd, length, deadline))
    length = struct.unpack('<I', read_exact(fd, 4, deadline))[0]
    if length > MAX_IMAGE:
        raise ValueError('worker image 超限')
    return metadata, read_exact(fd, length, deadline)


def write_packet(fd, metadata, image=b'', timeout=5):
    data = json.dumps(metadata, allow_nan=False, separators=(',', ':')).encode()
    if not 1 <= len(data) <= MAX_META or len(image) > MAX_IMAGE:
        raise ValueError('IPC 消息超限')
    data = memoryview(struct.pack('<I', len(data))+data+struct.pack('<I', len(image))+image)
    deadline = time.monotonic()+timeout
    while data:
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not select.select([], [fd], [], remaining)[1]:
            raise TimeoutError('YOLO worker 写入超时')
        try:
            count = os.write(fd, data[:65536])
        except BlockingIOError:
            continue
        if count == 0:
            raise EOFError('worker IPC 已关闭')
        data = data[count:]
