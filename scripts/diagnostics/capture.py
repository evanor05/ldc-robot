"""Passive ROS subscriptions + bounded backend snapshots; never commands a robot."""
import json
import os
from pathlib import Path
import queue
import signal
import sqlite3
import threading
import time
import urllib.request
from .storage import Archive, TYPES, atomic_json


def backend_snapshots(port, stop, output):
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    sequence = 0
    while not stop.is_set():
        snapshot = {}
        next_sequence = sequence
        for name, route in [('navigation', '/navigation_status'), ('events', f'/navigation/events?since={sequence}')]:
            try:
                with opener.open(f'http://127.0.0.1:{port}/api{route}', timeout=.5) as response:
                    raw = response.read(512*1024+1)
                    if len(raw)>512*1024:
                        raise ValueError('后端快照过大')
                    snapshot[name] = json.loads(raw)
                    if not isinstance(snapshot[name], dict):
                        raise ValueError('后端快照必须为对象')
                if name == 'events':
                    latest = snapshot[name].get('latest', sequence)
                    events = snapshot[name].get('events', [])
                    if not isinstance(events, list) or any(not isinstance(event, dict) for event in events):
                        raise ValueError('后端事件格式无效')
                    ids = [event['seq'] for event in events if isinstance(event.get('seq'), int)]
                    if isinstance(latest, int) and latest < sequence:
                        next_sequence = 0
                        snapshot['eventsGap'] = '后端事件序号已重置，下次从头读取'
                    else:
                        next_sequence = max([sequence, *ids])
                        if ids and min(ids) > sequence+1:
                            snapshot['eventsGap'] = '事件已超出后端缓冲范围，早期事件可能缺失'
            except (OSError, ValueError) as error:
                snapshot[name] = {'error': str(error)[:200]}
        try:
            output.put_nowait(snapshot)
            # Advance only after handing this snapshot to the recorder.
            sequence = next_sequence
        except queue.Full:
            pass
        stop.wait(.5)


def capture(folder, seconds, max_bytes, camera_every, api_port):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.utilities import get_message
    archive = Archive(folder, max_bytes)
    stop = threading.Event()
    snapshots = queue.Queue(maxsize=1)
    worker = threading.Thread(target=backend_snapshots, args=(api_port, stop, snapshots), daemon=True)
    rclpy.init()
    node = rclpy.create_node('ldc_passive_recorder')
    start = time.monotonic()
    reason = 'duration'
    last_image = 0.
    last_image_stamp = None
    invalid = 0
    failure = ''
    def stopped(*_):
        nonlocal reason
        reason = 'operator_stop'
        stop.set()
    old_signals = {sig: signal.signal(sig, stopped) for sig in (signal.SIGINT, signal.SIGTERM)}
    def receive(topic):
        def callback(msg):
            nonlocal last_image, last_image_stamp, reason, invalid, failure
            if stop.is_set():
                return
            now = time.monotonic()
            image = None
            if topic == '/camera/image_raw/compressed':
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                if not camera_every or now-last_image < camera_every or stamp == last_image_stamp:
                    return
                image = bytes(msg.data)
                if msg.format.lower() not in ('jpeg', 'jpg') or not image.startswith(b'\xff\xd8') or not image.endswith(b'\xff\xd9'):
                    invalid += 1
                    return
                last_image, last_image_stamp = now, stamp
            try:
                archive.append(topic, node.get_clock().now().nanoseconds, now-start, serialize_message(msg))
                if image is not None:
                    archive.image(f'{archive.rows:08d}_{stamp[0]}_{stamp[1]:09d}.jpg', image)
            except OverflowError as error:
                reason = str(error)
                stop.set()
            except (ValueError, OSError, sqlite3.Error) as error:
                reason = 'archive_error'
                failure = str(error)
                stop.set()
        return callback
    subscriptions = []
    for topic, typename in TYPES.items():
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if topic in ('/tf_static', '/map') else DurabilityPolicy.VOLATILE)
        subscriptions.append(node.create_subscription(get_message(typename), topic, receive(topic), qos))
    def status(state):
        return dict(schema='ldc-capture/1', state=state, seconds=time.monotonic()-start,
                    counts=archive.counts, bytes=archive.path.stat().st_size+archive.images_bytes,
                    maxBytes=max_bytes, passive=True, reason=reason, invalidImages=invalid,
                    hasSensorData=any(archive.counts.get(topic, 0) for topic in ('/scan', '/odom', '/imu')),
                    error=failure,
                    simulation=os.environ.get('ROBOT_SIMULATION')=='1', domain=os.environ.get('ROS_DOMAIN_ID', '0'),
                    pid=os.getpid(), createdAt=time.time()-(time.monotonic()-start))
    worker.start()
    tick = 0.
    atomic_json(Path(folder)/'capture.json', status('recording'))
    try:
        while not stop.is_set() and time.monotonic()-start < seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            try:
                archive.event(time.monotonic()-start, snapshots.get_nowait())
            except queue.Empty:
                pass
            except OverflowError as error:
                reason = str(error)
                break
            if time.monotonic()-tick > 1:
                atomic_json(Path(folder)/'capture.json', status('recording'))
                tick = time.monotonic()
    except (OSError, sqlite3.Error, ValueError) as error:
        failure = str(error)
        reason = 'archive_error'
    finally:
        stop.set()
        worker.join(timeout=2)
        for sig, previous in old_signals.items():
            signal.signal(sig, previous)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        result = status('failed' if failure else 'complete' if archive.rows else 'empty')
        result['sha256'] = archive.close()
        atomic_json(Path(folder)/'capture.json', result)
    return result
