#!/usr/bin/env python3
"""Real ROS launch check for the passive migration stage, never a vehicle test."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.navigation_bridge import Bridge
from ldc_exploration.cmd_vel_gate import CmdVelGate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not (os.environ.get('ROBOT_SIMULATION') == '1' and
            os.environ.get('ROS_DOMAIN_ID') == '197' and
            os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'):
        parser.error('Requires isolated simulation Domain197 with LOCALHOST discovery')
    args.output.mkdir(parents=True, exist_ok=False)
    processes, logs, nodes = [], [], []
    report = {'result': 'FAIL', 'synthetic': True, 'domain': 197, 'checks': [],
              'positiveAuthorizedMotionTested': False, 'automaticMappingComplete': False}
    executor = None

    def start(name, command):
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        processes.append(subprocess.Popen(command, cwd=ROOT, env=os.environ.copy(),
                         start_new_session=True, stdout=log, stderr=subprocess.STDOUT))

    try:
        start('robot', [sys.executable, str(ROOT / 'scripts/offline_robot.py')])
        start('launch', ['ros2', 'launch', 'ldc_exploration', 'auto_mapping.launch.py'])
        rclpy.init()
        executor = SingleThreadedExecutor()
        observer = rclpy.create_node('passive_exploration_acceptance')
        nodes.append(observer)
        executor.add_node(observer)
        maps, states, odometry, output, input_commands, gids = [], [], [], [], [], set()
        observer.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m.info.width), 10)
        observer.create_subscription(String, '/exploration/status', lambda m: states.append(json.loads(m.data)), 10)
        observer.create_subscription(Odometry, '/odom', lambda m: odometry.append(m.pose.pose.position.x), qos_profile_sensor_data)

        def observe(message, info):
            output.append((message.linear.x, message.angular.z))
            gid = info['publisher_gid']
            assert gid is not None, 'DDS publisher identity was not provided'
            gids.add(bytes(gid['data']).hex())

        observer.create_subscription(Twist, '/cmd_vel', observe, 10)
        observer.create_subscription(Twist, '/cmd_vel_auto', lambda m: input_commands.append(m.linear.x), 10)
        action = ActionClient(observer, NavigateToPose, '/navigate_to_pose')
        enable = observer.create_publisher(Bool, '/exploration/enable', 10)
        resume = observer.create_publisher(Bool, '/exploration/stop', 10)
        emergency = observer.create_publisher(Bool, '/emergency_stop', 10)
        commands = observer.create_publisher(Twist, '/cmd_vel_auto', 10)

        def spin_until(condition, seconds=30):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert all(p.poll() is None for p in processes), 'Fixture process exited'
                executor.spin_once(timeout_sec=.05)
                if condition():
                    return
            raise AssertionError('Expected ROS state did not arrive')

        spin_until(lambda: maps and states and action.server_is_ready())
        assert not observer.get_publishers_info_by_topic('/cmd_vel')
        assert not states[-1]['enabled']
        report['checks'].append('bare launch has no final velocity publisher and a disabled frontier')

        bridge, gate = Bridge(journal_path=':memory:'), CmdVelGate()
        for node in (bridge, gate):
            nodes.append(node)
            executor.add_node(node)
        spin_until(lambda: len(output) >= 10 and commands.get_subscription_count() >= 2)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            msg = Twist()
            msg.linear.x, msg.angular.z = .1, .4
            commands.publish(msg)
            enable.publish(Bool(data=True))
            resume.publish(Bool(data=False))
            emergency.publish(Bool(data=False))  # Synthetic chassis can move if a command leaks.
            executor.spin_once(timeout_sec=.03)
        assert sum(value == .1 for value in input_commands) >= 20, 'Nonzero input was not delivered'
        assert len(output) >= 20 and all(v == (0., 0.) for v in output)
        assert len(gids) == 1
        publishers = observer.get_publishers_info_by_topic('/cmd_vel')
        assert len(publishers) == 1 and publishers[0].node_name == 'console_navigation_bridge'
        assert states and not any(state['enabled'] for state in states)
        assert not gate.enabled
        assert odometry and max(abs(value) for value in odometry) < .001
        report['checks'] += ['legacy gate never owns final cmd_vel',
            'only Bridge GID produces final zero despite sustained nonzero upstream input',
            'legacy enable/resume Bool cannot grant motion', 'synthetic chassis remains stationary']
        report.update(result='PASS', finalPublisherGids=sorted(gids), zeroCommands=len(output),
                      nonzeroInputs=sum(value == .1 for value in input_commands))
    finally:
        if executor is not None:
            executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        for log in logs:
            log.close()
        (args.output / 'acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == '__main__':
    main()
