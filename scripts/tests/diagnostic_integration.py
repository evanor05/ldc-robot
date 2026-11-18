#!/usr/bin/env python3
"""Real DDS + CDR recording/replay acceptance; fixed Domains 195/196, synthetic input."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def environment(domain):
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_LOCALHOST_ONLY='1', FASTDDS_BUILTIN_TRANSPORTS='UDPv4', ROBOT_SIMULATION='1')
    for key in ('ROS_STATIC_PEERS', 'ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)


def observe(output):
    environment(196)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    rclpy.init(); node = rclpy.create_node('replay_acceptance_observer')
    counts = {name: 0 for name in ['odom', 'image', 'observed_velocity', 'robot_velocity', 'robot_initialpose']}
    def receive(name):
        def callback(_): counts[name] += 1
        return callback
    for cls, topic, name in [(Odometry, '/odom', 'odom'), (CompressedImage, '/camera/image_raw/compressed', 'image'),
                             (Twist, '/replay/recorded/cmd_vel', 'observed_velocity'), (Twist, '/cmd_vel', 'robot_velocity'),
                             (PoseWithCovarianceStamped, '/initialpose', 'robot_initialpose')]:
        node.create_subscription(cls, topic, receive(name), qos_profile_sensor_data)
    (output/'observer-ready').touch()
    deadline = time.monotonic()+10
    while time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    (output/'observed.json').write_text(json.dumps(counts, indent=2))
    node.destroy_node(); rclpy.shutdown()


def main(output):
    environment(195)
    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, Imu, CompressedImage
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from scripts.diagnostics.storage import report, open_archive
    from scripts.diagnostics.replay import summarize_capture
    output.mkdir(parents=True, exist_ok=False)
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            data = {'events': [{'seq': 1, 'type': 'synthetic_fixture', 'reason': '测试事件，无实车动作'}]} if '/events' in self.path else {'phase': 'stopped', 'mode': 'locked', 'synthetic': True}
            self.send_response(200); self.end_headers(); self.wfile.write(json.dumps(data).encode())
        def log_message(self, *_): pass
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    rclpy.init(); node = rclpy.create_node('diagnostic_fixture_source')
    publishers = [(cls, topic, node.create_publisher(cls, topic, qos_profile_sensor_data)) for cls, topic in
                  [(Odometry, '/odom'), (LaserScan, '/scan'), (Imu, '/imu'), (CompressedImage, '/camera/image_raw/compressed'),
                   (Twist, '/cmd_vel'), (PoseWithCovarianceStamped, '/initialpose')]]
    jpeg = cv2.imencode('.jpg', np.full((240, 320, 3), 100, np.uint8))[1].tobytes()
    start = time.monotonic()
    def publish():
        stamp = node.get_clock().now().to_msg()
        for cls, topic, publisher in publishers:
            msg = cls()
            if hasattr(msg, 'header'): msg.header.stamp = stamp
            if topic == '/odom':
                msg.header.frame_id='odom'; msg.child_frame_id='base_footprint'
                msg.pose.pose.orientation.w=1.; msg.pose.pose.position.x=(time.monotonic()-start)*.02
            elif topic == '/scan':
                msg.header.frame_id='laser_frame'; msg.angle_min=-math.pi; msg.angle_increment=math.pi/180
                msg.angle_max=msg.angle_min+359*msg.angle_increment; msg.range_min=.08; msg.range_max=8.; msg.ranges=[2.]*360
                msg.ranges[0]=float('inf'); msg.ranges[1]=float('nan')
            elif topic == '/imu':
                msg.header.frame_id='imu_link'; msg.orientation.w=1.
            elif topic == '/camera/image_raw/compressed':
                msg.header.frame_id='camera_optical_frame'; msg.format='jpeg'; msg.data=jpeg
            elif topic == '/cmd_vel': msg.linear.x=.2  # Only synthetic Domain 195.
            else: msg.header.frame_id='map'; msg.pose.pose.orientation.w=1.
            publisher.publish(msg)
    node.create_timer(.05, publish)
    log = (output/'process.log').open('w')
    processes = []
    try:
        command = [sys.executable, str(ROOT/'scripts/diagnostics/cli.py'), 'record', '--output', str(output/'capture'),
                   '--seconds', '5', '--max-mb', '4', '--camera-every', '1', '--api-port', str(server.server_address[1])]
        recorder = subprocess.Popen(command, stdout=log, stderr=log); processes.append(recorder)
        while recorder.poll() is None:
            rclpy.spin_once(node, timeout_sec=.05)
        assert recorder.returncode == 0
        captured = report(output/'capture')
        assert captured['capture']['state']=='complete'
        assert captured['counts']['/odom']>20 and captured['counts']['/cmd_vel']>20
        assert captured['counts']['/camera/image_raw/compressed']>=3
        assert captured['capture']['bytes']<=4*1024*1024
        summary = summarize_capture(output/'capture')
        assert summary['odom']['forward_delta_m']>.05
        # No source-side commands were introduced by the passive recorder.
        assert node.count_publishers('/cmd_vel')==1
        observer = subprocess.Popen([sys.executable, __file__, '--observe', '--output', str(output)], stdout=log, stderr=log)
        processes.append(observer)
        deadline=time.monotonic()+10
        while not (output/'observer-ready').exists():
            assert time.monotonic()<deadline
            time.sleep(.05)
        replay = subprocess.Popen([sys.executable, str(ROOT/'scripts/diagnostics/cli.py'), 'replay', str(output/'capture')], stdout=log, stderr=log)
        processes.append(replay); replay.wait(timeout=15); observer.wait(timeout=15)
        assert replay.returncode==observer.returncode==0
        observed=json.loads((output/'observed.json').read_text())
        assert observed['odom']>20 and observed['observed_velocity']>20 and observed['image']>=2, observed
        assert observed['robot_velocity']==observed['robot_initialpose']==0, observed
        (output/'acceptance.json').write_text(json.dumps(dict(result='PASS', synthetic=True, sourceDomain=195, replayDomain=196,
                      capture=captured['capture'], replay=observed, summary=summary), indent=2, ensure_ascii=False))
        print('PASS passive subscriptions, bounded archive, JPEG sampling, summary and isolated command-safe replay')
    finally:
        for proc in processes:
            if proc.poll() is None:
                proc.terminate(); proc.wait(timeout=6)
        log.close(); node.destroy_node(); rclpy.shutdown(); server.shutdown(); server.server_close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--output', type=Path, required=True); parser.add_argument('--observe', action='store_true')
    args=parser.parse_args()
    observe(args.output) if args.observe else main(args.output)
