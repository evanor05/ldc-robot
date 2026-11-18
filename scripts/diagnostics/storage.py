"""Bounded, append-only CDR archive. No user-supplied message types are imported."""
import hashlib
import json
from pathlib import Path
import sqlite3
import time

TYPES = {
    '/odom': 'nav_msgs/msg/Odometry', '/imu': 'sensor_msgs/msg/Imu',
    '/scan': 'sensor_msgs/msg/LaserScan', '/amcl_pose': 'geometry_msgs/msg/PoseWithCovarianceStamped',
    '/initialpose': 'geometry_msgs/msg/PoseWithCovarianceStamped', '/plan': 'nav_msgs/msg/Path',
    '/map': 'nav_msgs/msg/OccupancyGrid', '/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/cmd_vel': 'geometry_msgs/msg/Twist', '/cmd_vel_auto': 'geometry_msgs/msg/Twist',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo', '/camera/image_raw/compressed': 'sensor_msgs/msg/CompressedImage',
    '/diagnostics': 'diagnostic_msgs/msg/DiagnosticArray', '/battery_state': 'sensor_msgs/msg/BatteryState',
    '/navigate_to_pose/_action/status': 'action_msgs/msg/GoalStatusArray',
}
COMMAND_TOPICS = {'/cmd_vel', '/cmd_vel_auto', '/initialpose', '/navigate_to_pose/_action/status'}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


class Archive:
    def __init__(self, folder, max_bytes):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.path = self.folder/'recording.sqlite3'
        self.limit = max_bytes
        self.images_bytes = 0
        self.counts = {}
        self.rows = 0
        self.db = sqlite3.connect(self.path)
        self.db.executescript('CREATE TABLE messages (id INTEGER PRIMARY KEY, topic TEXT, ros_ns INTEGER, elapsed REAL, cdr BLOB);'
                              'CREATE TABLE events (id INTEGER PRIMARY KEY, elapsed REAL, payload TEXT);')
        self.db.execute(f'PRAGMA max_page_count={max(16, (max_bytes-65536)//4096)}')
        self.db.commit()

    def remaining(self):
        return self.limit-self.path.stat().st_size-self.images_bytes-65536

    def append(self, topic, ros_ns, elapsed, cdr):
        if topic not in TYPES or len(cdr) > 8*1024*1024:
            raise ValueError('未知话题或消息超过 8 MiB')
        if self.remaining() < len(cdr)+8192:
            raise OverflowError('达到采集容量上限')
        self.db.execute('INSERT INTO messages(topic,ros_ns,elapsed,cdr) VALUES(?,?,?,?)', (topic, ros_ns, elapsed, cdr))
        self.db.commit()
        self.rows += 1
        self.counts[topic] = self.counts.get(topic, 0)+1

    def event(self, elapsed, data):
        encoded = json.dumps(data, ensure_ascii=False, allow_nan=False)
        if len(encoded.encode()) > 1024*1024 or self.remaining() < len(encoded.encode())+8192:
            raise OverflowError('诊断事件达到采集容量上限')
        self.db.execute('INSERT INTO events(elapsed,payload) VALUES(?,?)', (elapsed, encoded))
        self.db.commit()

    def image(self, filename, data):
        if self.remaining() < len(data)+8192:
            raise OverflowError('图像达到采集容量上限')
        images = self.folder/'images'
        images.mkdir(exist_ok=True)
        with (images/filename).open('xb') as handle:
            handle.write(data)
        self.images_bytes += len(data)

    def close(self):
        self.db.close()
        with self.path.open('rb') as handle:
            return hashlib.file_digest(handle, 'sha256').hexdigest()


def open_archive(folder):
    path = (Path(folder)/'recording.sqlite3').resolve(strict=True)
    return sqlite3.connect(path.as_uri()+'?mode=ro', uri=True)


def verify_archive(folder):
    folder = Path(folder)
    metadata = json.loads((folder/'capture.json').read_text())
    if metadata.get('state') not in ('complete', 'empty') or not metadata.get('sha256'):
        raise ValueError('仅回放已完整结束且有校验值的采集；中断记录可先查看报告')
    with (folder/'recording.sqlite3').open('rb') as handle:
        digest = hashlib.file_digest(handle, 'sha256').hexdigest()
    if digest != metadata['sha256']:
        raise ValueError('采集校验失败：数据库与结束记录不一致，禁止回放')
    return metadata


def report(folder):
    folder = Path(folder)
    capture = json.loads((folder/'capture.json').read_text())
    with open_archive(folder) as db:
        counts = dict(db.execute('SELECT topic,COUNT(*) FROM messages GROUP BY topic'))
        count = db.execute('SELECT COUNT(*) FROM events').fetchone()[0]
        events = [dict(elapsed=t, **json.loads(payload)) for t, payload in db.execute('SELECT elapsed,payload FROM (SELECT id,elapsed,payload FROM events ORDER BY id DESC LIMIT 120) ORDER BY id')]
    # Return bounded summaries. The full event archive stays on disk.
    return dict(capture=capture, counts=counts, snapshots=events[-120:],
                snapshotCount=count, passive=True, replayDomain=196)
