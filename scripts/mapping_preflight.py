#!/usr/bin/env python3
"""Require live sensor samples and odometry TF before starting real SLAM."""
import argparse
import json
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--auto', action='store_true', help='自动建图：要求四周有足够旋转空间')
args = parser.parse_args()

rclpy.init()
node = rclpy.create_node('mapping_input_preflight')
buffer = Buffer()
listener = TransformListener(buffer, node)
received = {'scan': False, 'odom': False}
last_scan = {'msg': None}

def scan(message):
    received['scan'] = bool(message.ranges)
    last_scan['msg'] = message

def odom(message):
    received['odom'] = True

subscriptions = [
    node.create_subscription(LaserScan, '/scan', scan, rclpy.qos.qos_profile_sensor_data),
    node.create_subscription(Odometry, '/odom', odom, rclpy.qos.qos_profile_sensor_data),
]
deadline = time.monotonic() + 3.5
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if all(received.values()) and buffer.can_transform('odom', 'laser_frame', rclpy.time.Time()):
        break
missing = []
if not received['scan']:
    missing.append('实时雷达 /scan')
if not received['odom']:
    missing.append('实时里程计 /odom')
if not buffer.can_transform('odom', 'laser_frame', rclpy.time.Time()):
    missing.append('odom 到雷达的 TF')
if args.auto and last_scan['msg'] is not None:
    ranges = [r for r in last_scan['msg'].ranges if math.isfinite(r) and 0.02 < r < 12.0]
    if ranges:
        nearest = min(ranges)
        if nearest < 0.20:
            missing.append(f'自动建图旋转空间不足（最近障碍 {nearest:.2f} m，需 ≥0.20 m）')
print(json.dumps({'ready': not missing, 'missing': missing}, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(1 if missing else 0)
