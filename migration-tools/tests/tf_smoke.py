"""Real ROS process test: invalid input, frame conversion, source stamp, no stale TF."""
import os
os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

with tempfile.TemporaryDirectory(prefix='ros-car-tf-') as directory:
    with open(Path(directory) / 'node.log', 'w+') as log:
        process = subprocess.Popen([
            str(Path(get_package_prefix('ldc_bringup')) / 'lib/ldc_bringup/ldc_bringup'),
            '--ros-args', '-r', 'odom:=/migration_check/odom', '-r', '/tf:=/migration_check/tf'
        ], stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('tf_migration_test')
        pub = node.create_publisher(Odometry, '/migration_check/odom', qos_profile_sensor_data)
        received = []
        sub = node.create_subscription(TFMessage, '/migration_check/tf', lambda m: received.extend(m.transforms), 10)
        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
        try:
            end = time.monotonic() + 8
            while (not pub.get_subscription_count() or not node.count_publishers('/migration_check/tf')) and time.monotonic() < end:
                spin(0.1)
            assert pub.get_subscription_count(), 'Odometry discovery failed'
            spin(0.3)
            assert not received, 'Published TF before odometry'
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.orientation.w = 0.0
            pub.publish(msg)
            spin(0.3)
            assert not received, 'Accepted zero quaternion'
            msg.pose.pose.orientation.w = 2.0  # must normalize
            msg.pose.pose.position.x = 1.25
            msg.pose.pose.position.z = 0.06205
            pub.publish(msg)
            spin(0.5)
            assert len(received) == 1, len(received)
            tf = received[0]
            assert tf.header.stamp == msg.header.stamp, 'Acquisition timestamp lost'
            assert tf.header.frame_id == 'odom' and tf.child_frame_id == 'base_footprint'
            assert math.isclose(tf.transform.translation.x, 1.25)
            assert abs(tf.transform.translation.z) < 1e-9
            assert math.isclose(tf.transform.rotation.w, 1.0)
            spin(0.4)
            assert len(received) == 1, 'Repeats stale TF'
            msg.child_frame_id = 'base_footprint'
            msg.pose.pose.position.z = 0.0
            pub.publish(msg)
            spin(0.3)
            assert len(received) == 2 and received[-1].transform.translation.z == 0.0
            msg.child_frame_id = 'unknown_child'
            pub.publish(msg)
            spin(0.3)
            assert len(received) == 2, 'Accepted unknown frame'
            print(json.dumps({'result':'PASS','checks':['no initial TF','reject zero quaternion','normalize quaternion','base_link to footprint composition','source timestamp','no stale repeats','base_footprint input','reject unknown child']}))
        finally:
            node.destroy_node()
            rclpy.shutdown()
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            if process.returncode not in (0, -15):
                log.seek(0); print(log.read())
