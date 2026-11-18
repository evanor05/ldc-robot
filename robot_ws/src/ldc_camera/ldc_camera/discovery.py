"""MAVLink-style UDP discovery client for the ESP32-S3 camera module.

Protocol source: ``firmware/camera_stream/main/udp_discovery.c`` and
``firmware/camera_stream/scripts/test_discovery.py``.

The camera listens on UDP 33333.  It does not answer until it receives a GCS
heartbeat, then it locks onto that source and periodically emits MAVLink v2
STATUSTEXT frames containing::

    DEVICE_NAME:<name>
    DEVICE_IP:<ip>
    DEVICE_TYPE:maturo_esp32s3_cam
    HTTP_PORT:80
    STREAM_PORT:81

Only the standard library is used, so this module can also be imported by
small tools without ROS/OpenCV.
"""

from __future__ import annotations

import socket
import time
from typing import Optional

DISCOVERY_PORT = 33333

MAVLINK_V1_MAGIC = 0xFE
MAVLINK_V2_MAGIC = 0xFD
MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_STATUSTEXT = 253
MAVLINK_MSG_HEARTBEAT_LEN = 9
MAVLINK_MSG_STATUSTEXT_LEN = 51
MAVLINK_MSG_HEARTBEAT_CRC_EXTRA = 50
MAVLINK_MSG_STATUSTEXT_CRC_EXTRA = 83
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4


def _mavlink_crc_accumulate(data: int, crc: int) -> int:
    data ^= crc & 0xFF
    data ^= (data << 4) & 0xFF
    return (((data << 8) & 0xFFFF) | (crc >> 8)) ^ (data >> 4) ^ ((data << 3) & 0xFFFF)


def _mavlink_crc(buffer: bytes, crc_extra: int) -> int:
    crc = 0xFFFF
    for byte in buffer:
        crc = _mavlink_crc_accumulate(byte, crc)
    return _mavlink_crc_accumulate(crc_extra, crc)


def pack_mavlink_v2(
    sequence: int,
    sys_id: int,
    comp_id: int,
    msg_id: int,
    payload: bytes,
    crc_extra: int,
) -> bytes:
    header = bytes(
        (
            len(payload),
            0,
            0,
            sequence & 0xFF,
            sys_id & 0xFF,
            comp_id & 0xFF,
            msg_id & 0xFF,
            (msg_id >> 8) & 0xFF,
            (msg_id >> 16) & 0xFF,
        )
    )
    crc = _mavlink_crc(header + payload, crc_extra)
    return bytes((MAVLINK_V2_MAGIC,)) + header + payload + bytes((crc & 0xFF, crc >> 8))


def build_gcs_heartbeat(sequence: int = 0) -> bytes:
    payload = bytearray(MAVLINK_MSG_HEARTBEAT_LEN)
    payload[4] = MAV_TYPE_GCS
    payload[5] = MAV_AUTOPILOT_INVALID
    payload[6] = 0
    payload[7] = MAV_STATE_ACTIVE
    payload[8] = 3  # MAVLink version
    return pack_mavlink_v2(
        sequence=sequence,
        sys_id=255,
        comp_id=0,
        msg_id=MAVLINK_MSG_ID_HEARTBEAT,
        payload=bytes(payload),
        crc_extra=MAVLINK_MSG_HEARTBEAT_CRC_EXTRA,
    )


def parse_statustext_frames(data: bytes) -> list[str]:
    """Return STATUSTEXT strings found in a UDP datagram (v1 or v2)."""
    texts: list[str] = []
    index = 0
    size = len(data)

    while index < size:
        magic = data[index]
        if magic == MAVLINK_V2_MAGIC:
            if index + 12 > size:
                break
            payload_len = data[index + 1]
            frame_len = 10 + payload_len + 2
            if index + frame_len > size:
                break
            msg_id = (
                data[index + 7]
                | (data[index + 8] << 8)
                | (data[index + 9] << 16)
            )
            payload = data[index + 10:index + 10 + payload_len]
            if msg_id == MAVLINK_MSG_ID_STATUSTEXT and payload:
                text = payload[1:1 + 50].split(b"\x00", 1)[0]
                texts.append(text.decode("utf-8", errors="replace").strip())
            index += frame_len
        elif magic == MAVLINK_V1_MAGIC:
            if index + 8 > size:
                break
            payload_len = data[index + 1]
            frame_len = 6 + payload_len + 2
            if index + frame_len > size:
                break
            msg_id = data[index + 5]
            payload = data[index + 6:index + 6 + payload_len]
            if msg_id == MAVLINK_MSG_ID_STATUSTEXT and payload:
                text = payload[1:1 + 50].split(b"\x00", 1)[0]
                texts.append(text.decode("utf-8", errors="replace").strip())
            index += frame_len
        else:
            # Resynchronise: skip one byte until the next magic.
            index += 1

    return texts


class DiscoveryPortError(RuntimeError):
    """The fixed UDP discovery listener port is occupied or unusable."""


def discover_camera(
    timeout: float = 2.0,
    port: int = DISCOVERY_PORT,
    broadcast: str = "255.255.255.255",
    bind: str = "0.0.0.0",
    bind_port: Optional[int] = None,
) -> Optional[dict]:
    """Broadcast a GCS heartbeat and return the first complete camera record.

    The firmware replies to a fixed UDP port (33333), not to the caller's
    ephemeral source port.  Therefore the client must listen on that fixed
    port.  ``bind_port`` defaults to ``port`` so existing callers keep the
    firmware-compatible behaviour; tests may pass a separate free port.

    Returns a dict with ``ip``, ``name``, ``http_port``, ``stream_port`` and
    ``stream_url``; ``None`` if no camera answers within ``timeout`` seconds.
    Raises ``DiscoveryPortError`` if the listener port cannot be bound.
    """
    if timeout <= 0:
        return None

    listen_port = port if bind_port is None else bind_port
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            sock.bind((bind, listen_port))
        except OSError as error:
            raise DiscoveryPortError(
                f'UDP 发现端口 {listen_port} 被占用或不可用；请停止占用该端口的进程后重试'
            ) from error
        sock.settimeout(0.2)

        devices: dict[str, dict] = {}
        deadline = time.monotonic() + timeout
        next_heartbeat = 0.0
        sequence = 0

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                try:
                    sock.sendto(build_gcs_heartbeat(sequence), (broadcast, port))
                except OSError:
                    pass
                sequence = (sequence + 1) & 0xFF
                next_heartbeat = now + 0.7

            try:
                data, source = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            ip = source[0]
            device = devices.setdefault(
                ip,
                {
                    "ip": ip,
                    "name": "",
                    "http_port": 80,
                    "stream_port": 81,
                    "stream_url": "",
                    "seen_ip": False,
                    "seen_stream_port": False,
                },
            )
            for text in parse_statustext_frames(data):
                if text.startswith("DEVICE_NAME:"):
                    device["name"] = text.split(":", 1)[1]
                elif text.startswith("DEVICE_IP:"):
                    value = text.split(":", 1)[1].strip()
                    if value:
                        device["ip"] = value
                        device["seen_ip"] = True
                elif text.startswith("HTTP_PORT:"):
                    value = text.split(":", 1)[1].strip()
                    if value.isdigit():
                        device["http_port"] = int(value)
                elif text.startswith("STREAM_PORT:"):
                    value = text.split(":", 1)[1].strip()
                    if value.isdigit():
                        device["stream_port"] = int(value)
                        device["seen_stream_port"] = True

            for candidate in devices.values():
                if candidate["seen_ip"] and candidate["seen_stream_port"]:
                    candidate["stream_url"] = (
                        f"http://{candidate['ip']}:{candidate['stream_port']}/"
                    )
                    result = dict(candidate)
                    result.pop("seen_ip", None)
                    result.pop("seen_stream_port", None)
                    return result

        return None
    finally:
        sock.close()
