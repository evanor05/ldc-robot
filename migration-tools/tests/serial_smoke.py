"""Exercise the adapted ROS bridge using a pseudo-terminal, never a real device."""
import json
import math
import os
import pty
import select
import struct
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import tempfile

audit = Path(tempfile.mkdtemp(prefix='ros-car-serial-'))
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

def crc_x25(data):
    crc = 0xffff
    for byte in data:
        tmp = byte ^ (crc & 0xff)
        tmp ^= (tmp << 4) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc

def attitude_packet():
    payload = struct.pack('<Iffffff', 123, 0.0, 0.0, 0.2, 1.0, 2.0, 3.0)
    header = bytes([len(payload), 0, 0, 0, 1, 1, 30, 0, 0])
    checksum = crc_x25(header + payload + bytes([39]))
    return bytes([0xfd]) + header + payload + struct.pack('<H', checksum)

master, slave = pty.openpty()
slave_path = os.ttyname(slave)
assert slave_path.startswith('/dev/pts/'), slave_path
log = open(audit / 'serial-bridge.log', 'w')
process = subprocess.Popen([
    str(Path(get_package_prefix('ldc_mavlink_bridge')) / 'lib/ldc_mavlink_bridge/mavlink_bridge'),
    '--ros-args', '-p', f'port:={slave_path}', '-p', 'baudrate:=230400',
    '-p', 'cmd_vel_topic:=/preflight/cmd_vel', '-p', 'imu_topic:=/preflight/imu',
    '-p', 'odom_topic:=/preflight/odom', '-p', 'scan_topic:=/preflight/scan',
], stdout=log, stderr=subprocess.STDOUT)
rclpy.init()
node = rclpy.create_node('bridge_preflight')
publisher = node.create_publisher(Twist, '/preflight/cmd_vel', 10)
received = []
subscription = node.create_subscription(Imu, '/preflight/imu', received.append, qos_profile_sensor_data)
buffer = bytearray()

def read_velocity(timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if select.select([master], [], [], 0.1)[0]:
            buffer.extend(os.read(master, 4096))
        while buffer:
            if buffer[0] != 0xfd:
                del buffer[0]
                continue
            if len(buffer) < 10:
                break
            total = 12 + buffer[1] + (13 if buffer[2] & 1 else 0)
            if len(buffer) < total:
                break
            packet = bytes(buffer[:total])
            del buffer[:total]
            msgid = int.from_bytes(packet[7:10], 'little')
            if msgid == 84:
                payload = packet[10:10 + packet[1]].ljust(53, b'\0')
                assert crc_x25(packet[1:10 + packet[1]] + bytes([143])) == int.from_bytes(packet[10 + packet[1]:12 + packet[1]], 'little')
                return struct.unpack_from('<f', payload, 16)[0], struct.unpack_from('<f', payload, 44)[0]
    raise TimeoutError('No MAVLink velocity packet received')

try:
    deadline = time.monotonic() + 8
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert publisher.get_subscription_count() > 0, 'ROS subscriber discovery failed'
    command = Twist()
    command.linear.x = 0.123
    command.angular.z = 0.321
    publisher.publish(command)
    velocity = read_velocity()
    assert abs(velocity[0] - 0.123) < 1e-6 and abs(velocity[1] - 0.321) < 1e-6, velocity
    publisher.publish(Twist())
    zero = read_velocity()
    assert zero == (0.0, 0.0), zero
    deadline = time.monotonic() + 5
    while not received and time.monotonic() < deadline:
        os.write(master, attitude_packet())
        rclpy.spin_once(node, timeout_sec=0.1)
    assert received, 'No ROS IMU message received'
    angular = received[-1].angular_velocity
    assert (angular.x, angular.y, angular.z) == (1.0, -2.0, -3.0)
    result = {'result': 'PASS', 'device': 'pseudo-terminal only', 'checks': ['ROS Twist to MAVLink velocity with valid CRC', 'zero velocity encoding', 'MAVLink ATTITUDE to ROS Imu'], 'limitations': ['No physical vehicle', 'No micro-ROS', 'No physical stopping-distance test']}
    result['logs'] = str(audit)
    print(json.dumps(result, indent=2))
    open(audit / 'result.json', 'w').write(json.dumps(result, indent=2) + '\n')
finally:
    node.destroy_node()
    rclpy.shutdown()
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
    os.close(master)
    os.close(slave)
    log.close()
