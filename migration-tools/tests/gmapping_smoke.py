"""Offline GMapping smoke tests for LDC mapping fixes.

Runs the real project executable in ROS domain 197 (localhost only). Covers:
- configured YAML parameters and transient-local map retention
- canonical [-pi, pi) scans
- firmware-style [0, 2pi) scans (angle normalization)
- nonzero initial odometry (map origin must stay anchored)
- NaN range robustness
- 10 Hz timestamp handling (temporalUpdate=1 s must process ~1 Hz, not 0.5 Hz)
"""
import math
import os
os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')

import json
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage

LOG_DIR = Path(tempfile.mkdtemp(prefix='ros-car-gmapping-'))
LOG_FILE = LOG_DIR / 'gmapping.log'

rclpy.init()
node = rclpy.create_node('gmapping_migration_test')
broadcaster = TransformBroadcaster(node)
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
maps = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(OccupancyGrid, '/map', maps.append, qos)
map_to_odom = []

def on_tf(message):
    for transform in message.transforms:
        if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
            map_to_odom.append(transform)

node.create_subscription(TFMessage, '/tf', on_tf, 10)

ANGLE_INCREMENT = 2.0 * math.pi / 360.0

# Asymmetric room: 5 m x 4 m box plus a pillar in the front-right quadrant.
ROOM = [(-2.5, -2.0, 2.5, -2.0), (2.5, -2.0, 2.5, 2.0),
        (2.5, 2.0, -2.5, 2.0), (-2.5, 2.0, -2.5, -2.0)]
PILLAR = [(1.2, -0.6, 1.8, -0.6), (1.8, -0.6, 1.8, 0.6),
          (1.8, 0.6, 1.2, 0.6), (1.2, 0.6, 1.2, -0.6)]


def ray_distance(angle, origin=(0.0, 0.0), yaw=0.0):
    global_angle = yaw + angle
    dx, dy = math.cos(global_angle), math.sin(global_angle)
    ox, oy = origin
    best = 8.0
    for x1, y1, x2, y2 in ROOM + PILLAR:
        ex, ey = x2 - x1, y2 - y1
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-9:
            continue
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / denominator
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / denominator
        if t > 1e-6 and -1e-9 <= u <= 1.0 + 1e-9:
            best = min(best, t)
    return best


def make_scan(now, style, robot_pose=(0.0, 0.0, 0.0)):
    scan = LaserScan()
    scan.header.stamp = now.to_msg()
    scan.header.frame_id = 'laser_frame'
    if style == 'canonical':
        scan.angle_min = -math.pi
        scan.angle_increment = ANGLE_INCREMENT
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.ranges = [ray_distance(scan.angle_min + i * scan.angle_increment,
                                    (robot_pose[0], robot_pose[1]), robot_pose[2]) for i in range(360)]
    else:  # firmware style: index 0 is 0 rad
        scan.angle_min = 0.0
        scan.angle_increment = ANGLE_INCREMENT
        scan.angle_max = 359 * scan.angle_increment
        scan.ranges = [ray_distance(i * scan.angle_increment,
                                    (robot_pose[0], robot_pose[1]), robot_pose[2]) for i in range(360)]
    scan.scan_time = 0.1
    scan.time_increment = 0.1 / 360
    scan.range_min = 0.02
    scan.range_max = 12.0
    return scan


def send_tf(stamp, odom_pose=(0.0, 0.0, 0.0)):
    x, y, yaw = odom_pose
    stamp_message = stamp.to_msg() if hasattr(stamp, 'to_msg') else stamp
    transform = TransformStamped()
    transform.header.stamp = stamp_message
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_footprint'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    broadcaster.sendTransform(transform)
    transform = TransformStamped()
    transform.header.stamp = stamp_message
    transform.header.frame_id = 'base_footprint'
    transform.child_frame_id = 'laser_frame'
    transform.transform.translation.z = 0.18
    transform.transform.rotation.w = 1.0
    broadcaster.sendTransform(transform)


def start_gmapping():
    log = open(LOG_FILE, 'a')
    process = subprocess.Popen(
        ['ros2', 'launch', 'slam_gmapping', 'slam_gmapping.launch.py'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return process, log


def stop_process(process, log):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    log.close()


def spin_for(seconds, odom_pose, style='canonical', nan_index=None,
             alive_check=None, moving=False):
    deadline = time.monotonic() + seconds
    started = time.monotonic()
    next_publish = 0.0
    while time.monotonic() < deadline:
        if alive_check is not None:
            assert alive_check(), 'GMapping exited during scan publishing'
        rclpy.spin_once(node, timeout_sec=0.01)
        now = node.get_clock().now()
        stamp = now - rclpy.duration.Duration(seconds=0.02)
        elapsed = time.monotonic() - started
        robot_pose = odom_pose
        if moving:
            robot_pose = (odom_pose[0] + 0.05 * elapsed,
                          odom_pose[1] + 0.02 * elapsed,
                          odom_pose[2] + 0.02 * elapsed)
        send_tf(stamp, robot_pose)
        if time.monotonic() >= next_publish:
            relative_pose = (robot_pose[0] - odom_pose[0],
                             robot_pose[1] - odom_pose[1],
                             math.atan2(math.sin(robot_pose[2] - odom_pose[2]),
                                        math.cos(robot_pose[2] - odom_pose[2])))
            scan = make_scan(stamp, style, relative_pose)
            if nan_index is not None:
                scan.ranges[nan_index] = float('nan')
            scan_pub.publish(scan)
            next_publish = time.monotonic() + 0.1
        time.sleep(0.01)


def wait_for_map(timeout=8.0, alive_check=None):
    deadline = time.monotonic() + timeout
    while not maps and time.monotonic() < deadline:
        if alive_check is not None:
            assert alive_check(), 'GMapping exited before publishing a map'
        rclpy.spin_once(node, timeout_sec=0.1)
    assert maps, f'No map published; logs={LOG_DIR}'
    return maps[-1]


def get_params():
    client = node.create_client(GetParameters, '/slam_gmapping/get_parameters')
    assert client.wait_for_service(timeout_sec=5), 'parameter service unavailable'
    request = GetParameters.Request(names=['base_frame', 'maxRange', 'maxUrange',
                                           'map_update_interval', 'zero_at_start'])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    values = future.result().values
    assert values[0].string_value == 'base_footprint'
    assert [v.double_value for v in values[1:4]] == [6.0, 4.0, 5.0]
    assert values[4].bool_value is True


def occupied_mask(message):
    return [value == 100 for value in message.data]


def run_case(name, style='canonical', odom_pose=(0.0, 0.0, 0.0), seconds=2.0,
             nan_index=None, expect_alive=True, check_params=False, moving=False,
             late_check=False):
    maps.clear()
    map_to_odom.clear()
    process, log = start_gmapping()
    try:
        spin_for(seconds, odom_pose, style=style, nan_index=nan_index,
                 alive_check=lambda: process.poll() is None, moving=moving)
        deadline = time.monotonic() + 10.0
        while not maps and time.monotonic() < deadline:
            spin_for(0.2, odom_pose, style=style, nan_index=nan_index,
                     alive_check=lambda: process.poll() is None, moving=moving)
        assert maps, f'No map published; logs={LOG_DIR}'
        message = maps[-1]
        if check_params:
            get_params()
        if late_check:
            late = []
            node.create_subscription(OccupancyGrid, '/map', late.append, qos)
            deadline = time.monotonic() + 2.0
            while not late and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert late, 'Map not retained for late subscribers'
        alive = process.poll() is None
        assert alive == expect_alive, f'{name}: GMapping alive={alive}, expected {expect_alive}'
        return message, alive
    finally:
        stop_process(process, log)


def main():
    checks = []
    try:
        # 1) canonical scan + params + transient-local retention.
        message, _ = run_case('canonical', style='canonical', seconds=2.0,
                              check_params=True, late_check=True)
        assert any(value == 100 for value in message.data)
        assert any(value == 0 for value in message.data)
        assert (message.info.width, message.info.height) == (384, 384)
        canonical_mask = occupied_mask(message)
        checks.append('canonical scan creates occupied/free map, params applied, map retained')

        # 2) firmware-style [0, 2pi) scan must produce the same physical map.
        firmware_map, _ = run_case('firmware_angles', style='firmware', seconds=2.0)
        assert (firmware_map.info.width, firmware_map.info.height) == (384, 384)
        firmware_mask = occupied_mask(firmware_map)
        mismatches = sum(1 for a, b in zip(canonical_mask, firmware_mask) if a != b)
        assert mismatches <= 5, f'angle normalization changed map: {mismatches} cells differ'
        assert 'Normalized scan angles' in LOG_FILE.read_text(errors='replace')
        checks.append('[0,2pi) scan normalized to same map as [-pi,pi)')

        # 3) nonzero initial odometry must not move/expand the map frame.
        offset_map, _ = run_case('nonzero_odom', odom_pose=(-7.4, -1.4, 0.629),
                                 seconds=3.0, moving=True)
        assert (offset_map.info.width, offset_map.info.height) == (384, 384), \
            f'map expanded: {offset_map.info.width}x{offset_map.info.height}'
        assert abs(offset_map.info.origin.position.x + 10.0) < 0.01
        assert abs(offset_map.info.origin.position.y + 10.0) < 0.01
        assert map_to_odom, 'map->odom TF not published'
        published = next((t for t in map_to_odom
                          if math.hypot(t.transform.translation.x,
                                        t.transform.translation.y) > 0.1), map_to_odom[-1])
        yaw = math.atan2(2.0 * (published.transform.rotation.w * published.transform.rotation.z),
                         1.0 - 2.0 * published.transform.rotation.z ** 2)
        c, s = math.cos(0.629), math.sin(0.629)
        expected_x = -c * -7.4 - s * -1.4
        expected_y = s * -7.4 - c * -1.4
        assert abs(published.transform.translation.x - expected_x) < 0.4, \
            f'map->odom x {published.transform.translation.x} != {expected_x}'
        assert abs(published.transform.translation.y - expected_y) < 0.4, \
            f'map->odom y {published.transform.translation.y} != {expected_y}'
        assert abs(yaw + 0.629) < 0.1, f'map->odom yaw {yaw} != -0.629'
        checks.append('nonzero initial odom keeps map origin at (-10,-10), map->odom TF correct')

        # 4) NaN range must not crash the node.
        nan_map, _ = run_case('nan', seconds=2.0, nan_index=10)
        assert any(value == 100 for value in nan_map.data)
        checks.append('NaN range filtered without crashing')

        # 5) 10 Hz scans with temporalUpdate=1 s must process ~1 Hz (>=4 in 5 s).
        run_case('timing', seconds=10.0)
        counts = [int(value) for value in re.findall(r'm_count (\d+)', LOG_FILE.read_text(errors='replace'))]
        assert counts and max(counts) >= 6, f'only {max(counts) if counts else 0} updates in 10 s'
        checks.append(f'10 Hz timestamps processed {max(counts) + 1} scans in ~10 s')

        print(json.dumps({'result': 'PASS', 'checks': checks, 'map_size': [384, 384],
                          'logs': str(LOG_DIR)}, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
