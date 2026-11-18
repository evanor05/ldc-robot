#!/usr/bin/env python3
"""DDS run-channel isolation probe; private topics only, no actuator publisher."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import Twist

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.velocity_channel import VelocityChannel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not (os.environ.get('ROS_DOMAIN_ID') == '198' and
            os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'):
        parser.error('Requires Domain198 and LOCALHOST discovery')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'result': 'FAIL', 'domain': 198, 'noActuatorPublisher': True, 'checks': []}
    rclpy.init()
    node = rclpy.create_node('velocity_epoch_acceptance')
    received, restarted = [], []
    channel = VelocityChannel(node, lambda msg: received.append(msg.linear.x))
    successor = None

    def publisher(topic, value):
        pub = node.create_publisher(Twist, topic, 10)
        msg = Twist()
        msg.linear.x = value
        return pub, msg

    def spin(seconds, publications):
        deadline, next_publish = time.monotonic() + seconds, 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_publish:
                for pub, msg in publications:
                    pub.publish(msg)
                next_publish = time.monotonic() + .05
            rclpy.spin_once(node, timeout_sec=.01)

    try:
        first = channel.rotate()
        old = publisher(first, .1)
        spin(1.5, [old])
        assert len(received) >= 10 and all(v == .1 for v in received)
        report['checks'].append('current first-run channel receives positive DDS input')
        second = channel.rotate()
        current = publisher(second, .2)
        received.clear()
        spin(1.5, [old, current])
        assert first != second and len(received) >= 10 and all(v == .2 for v in received)
        report['checks'].append('old topic continues publishing but cannot enter new run')
        channel.close()
        count = len(received)
        spin(.7, [old, current])
        assert len(received) == count
        report['checks'].append('closed channel rejects sustained old/current publishers')
        successor = VelocityChannel(node, lambda msg: restarted.append(msg.linear.x))
        third = successor.rotate()
        replacement = publisher(third, .3)
        spin(1.5, [old, current, replacement])
        assert third not in (first, second) and successor.boot_id != channel.boot_id
        assert len(restarted) >= 10 and all(v == .3 for v in restarted)
        report['checks'].append('restarted owner receives only its new boot/run channel')
        report.update(result='PASS', firstTopic=first, secondTopic=second, restartedTopic=third,
                      currentSamples=count, restartedSamples=len(restarted))
    finally:
        channel.close()
        if successor:
            successor.close()
        node.destroy_node()
        rclpy.shutdown()
        (args.output / 'acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
