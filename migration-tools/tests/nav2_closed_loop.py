"""Closed-loop Nav2 verification in a synthetic 2D world; no vehicle, no transport.

Drives the real navigation stack with a kinematic differential-drive robot, a
ray-cast lidar and odometry that can be given scale error and noise, so goal
reaching, obstacle avoidance and localisation are measured rather than assumed.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import argparse
import json
import math
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

REPO = Path(__file__).resolve().parents[2]
RESOLUTION = 0.05
WORLD_W, WORLD_H = 320, 240          # 16 m x 12 m
ORIGIN = (-8.0, -6.0)
FOOTPRINT_HALF = (0.1445 / 2, 0.095 / 2)   # URDF chassis box, centred on base_link
WHEEL_TRACK = 0.162
LIDAR_BEAMS = 361
LIDAR_RANGE_MIN, LIDAR_RANGE_MAX = 0.05, 12.0
LIDAR_HZ = 10.0
ODOM_HZ = 50.0
VEL_LAG = 0.15                        # first-order command lag, seconds
NAV2_NODES = ['map_server', 'amcl', 'controller_server', 'planner_server', 'smoother_server',
              'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother']


class Grid:
    """Occupancy grid with world<->pixel conversion; pixel row 0 is the highest y."""

    def __init__(self, occupied):
        self.occ = occupied
        self.h, self.w = occupied.shape
        self.ox, self.oy = ORIGIN

    def to_pixel(self, xs, ys):
        px = np.floor((np.asarray(xs, float) - self.ox) / RESOLUTION).astype(np.int64)
        py = np.floor((self.oy + self.h * RESOLUTION - np.asarray(ys, float)) / RESOLUTION).astype(np.int64)
        return px, py

    def set_box(self, x0, y0, x1, y1):
        px0, py0 = self.to_pixel([min(x0, x1)], [max(y0, y1)])
        px1, py1 = self.to_pixel([max(x0, x1)], [min(y0, y1)])
        self.occ[max(int(py0[0]), 0):int(py1[0]) + 1, max(int(px0[0]), 0):int(px1[0]) + 1] = True

    def occupied_at(self, xs, ys):
        px, py = self.to_pixel(xs, ys)
        inside = (px >= 0) & (px < self.w) & (py >= 0) & (py < self.h)
        px = np.clip(px, 0, self.w - 1)
        py = np.clip(py, 0, self.h - 1)
        return self.occ[py, px] & inside

    def raycast(self, x, y, yaw, angles, step=RESOLUTION / 2):
        samples = int(math.ceil(LIDAR_RANGE_MAX / step)) + 1
        dist = np.arange(samples) * step
        bearing = yaw + angles
        xs = x + np.cos(bearing)[:, None] * dist[None, :]
        ys = y + np.sin(bearing)[:, None] * dist[None, :]
        px, py = self.to_pixel(xs, ys)
        outside = (px < 0) | (px >= self.w) | (py < 0) | (py >= self.h)
        hit = self.occ[np.clip(py, 0, self.h - 1), np.clip(px, 0, self.w - 1)] | outside
        blocked = hit.any(axis=1)
        first = np.argmax(hit, axis=1)
        return np.where(blocked, np.minimum(first * step, LIDAR_RANGE_MAX), LIDAR_RANGE_MAX)

    def write(self, directory, name):
        image = np.where(self.occ, 0, 254).astype(np.uint8)
        (directory / f'{name}.pgm').write_bytes(
            f'P5\n{self.w} {self.h}\n255\n'.encode() + image.tobytes())
        (directory / f'{name}.yaml').write_text(
            f'image: {name}.pgm\nmode: trinary\nresolution: {RESOLUTION}\n'
            f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return directory / f'{name}.yaml'


def base_grid():
    grid = Grid(np.zeros((WORLD_H, WORLD_W), dtype=bool))
    wall = 0.1
    x0, y0 = ORIGIN
    x1, y1 = x0 + WORLD_W * RESOLUTION, y0 + WORLD_H * RESOLUTION
    grid.set_box(x0, y0, x1, y0 + wall)
    grid.set_box(x0, y1 - wall, x1, y1)
    grid.set_box(x0, y0, x0 + wall, y1)
    grid.set_box(x1 - wall, y0, x1, y1)
    return grid


def scenarios():
    """Each scenario owns a world, so obstacles cannot leak between runs."""
    straight = base_grid()
    turn = base_grid()

    doorway = base_grid()
    doorway.set_box(-2.0, ORIGIN[1], -1.9, -1.25)
    doorway.set_box(-2.0, -0.75, -1.9, -ORIGIN[1])

    detour = base_grid()
    detour.set_box(-1.6, 0.4, -0.4, 1.6)

    tight = base_grid()
    tight.set_box(-2.0, ORIGIN[1], -1.9, 2.82)
    tight.set_box(-2.0, 3.18, -1.9, -ORIGIN[1])

    return [
        {'name': 'straight_4m', 'grid': straight, 'start': (-6.0, -4.0, 0.0), 'goal': (-2.0, -4.0, 0.0)},
        {'name': 'turn_90_6m', 'grid': turn, 'start': (-6.0, -4.0, 0.0), 'goal': (-6.0, 2.0, math.pi / 2)},
        {'name': 'doorway_050', 'grid': doorway, 'start': (-5.0, -1.0, 0.0), 'goal': (1.0, -1.0, 0.0)},
        {'name': 'detour_box', 'grid': detour, 'start': (-5.0, 1.0, 0.0), 'goal': (3.0, 1.0, 0.0)},
        {'name': 'tight_036', 'grid': tight, 'start': (-5.0, 3.0, 0.0), 'goal': (1.0, 3.0, 0.0)},
        {'name': 'drift_turn_90', 'grid': base_grid(), 'start': (-6.0, -4.0, 0.0), 'goal': (-6.0, 2.0, math.pi / 2),
         'odom': {'scale_error': 0.02, 'yaw_bias': math.radians(0.2), 'noise': 0.005}},
    ]


class Robot:
    """True state plus the odometry the stack is allowed to see."""

    def __init__(self, start, odom_error=None, rng=None):
        self.x, self.y, self.yaw = start
        self.v = self.w = 0.0
        self.cmd_v = self.cmd_w = 0.0
        self.error = odom_error or {}
        self.rng = rng or np.random.default_rng(1234)
        self.odom_x, self.odom_y, self.odom_yaw = start

    def command(self, twist):
        self.cmd_v, self.cmd_w = twist.linear.x, twist.angular.z

    def step(self, dt):
        self.v += (self.cmd_v - self.v) * dt / VEL_LAG
        self.w += (self.cmd_w - self.w) * dt / VEL_LAG
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        scale = 1.0 + self.error.get('scale_error', 0.0)
        noise = self.error.get('noise', 0.0)
        self.odom_x += self.v * scale * math.cos(self.odom_yaw) * dt + self.rng.normal(0, noise)
        self.odom_y += self.v * scale * math.sin(self.odom_yaw) * dt + self.rng.normal(0, noise)
        self.odom_yaw += (self.w + self.error.get('yaw_bias', 0.0)) * dt

    def footprint_cells(self, grid):
        corners = [(sx * FOOTPRINT_HALF[0], sy * FOOTPRINT_HALF[1])
                   for sx in (-1, 1) for sy in (-1, 1)]
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        xs = [self.x + cx * cos - cy * sin for cx, cy in corners] + [self.x]
        ys = [self.y + cx * sin + cy * cos for cx, cy in corners] + [self.y]
        return grid.occupied_at(xs, ys).any()


class Nav2Session:
    def __init__(self, node, map_yaml, params_file, log_dir):
        self.node = node
        self.log_dir = log_dir
        self.processes = []
        self.files = []
        self.clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in NAV2_NODES}
        self.pending = {}
        self.pending_since = {}
        self.states = {}
        commands = [
            ['ros2', 'launch', 'ldc_bringup', 'ldc_bringup.launch.py'],
            ['ros2', 'launch', 'ldc_navigation', 'navigation2.launch.py',
             f'map:={map_yaml}', f'params_file:={params_file}',
             # 该验收直接订阅 /cmd_vel，显式使用直连测试输出；生产默认已改为桥接输入。
             'cmd_vel_output:=cmd_vel'],
        ]
        for index, command in enumerate(commands):
            log = open(log_dir / f'nav2-{index}.log', 'w')
            self.files.append(log)
            self.processes.append(subprocess.Popen(
                command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def poll_states(self):
        """Best-effort lifecycle snapshot for reporting.

        A lost get_state response used to wedge readiness detection, so this never
        gates the run: stale futures are dropped and re-issued.
        """
        for name, client in self.clients.items():
            future = self.pending.get(name)
            if future is None:
                if client.service_is_ready():
                    self.pending[name] = client.call_async(GetState.Request())
                    self.pending_since[name] = time.monotonic()
            elif future.done():
                self.states[name] = future.result().current_state.id
                self.pending.pop(name)
            elif time.monotonic() - self.pending_since[name] > 2.0:
                self.pending.pop(name)
        return dict(self.states)

    def alive(self):
        for process in self.processes:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited with {process.returncode}; logs in {self.log_dir}')

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in self.processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in self.files:
            log.close()
        for client in self.clients.values():
            self.node.destroy_client(client)


class ScenarioRun:
    def __init__(self, node, scenario, params_file, log_root, timeout, tf_buffer, action):
        self.node = node
        self.scenario = scenario
        self.params_file = params_file
        self.log_root = log_root
        self.timeout = timeout
        self.rng = np.random.default_rng(20261027)
        self.robot = Robot(scenario['start'], scenario.get('odom'), self.rng)
        self.angles = np.linspace(-math.pi, math.pi, LIDAR_BEAMS)
        self.grid = scenario['grid']
        self.trajectory = []
        self.commands = []
        self.collision_samples = 0
        self.samples = 0
        self.plans = []
        self.pose_errors = []
        self.initial_error = None
        self.last_scan = 0.0
        self.last_pose_hint = 0.0
        self.last_tick_wall = time.monotonic()
        self.session = None

        self.odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_sub = node.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.plan_sub = node.create_subscription(PathMsg, '/plan', self.plans.append, 10)
        self.tf_buffer = tf_buffer
        self.action = action

    def on_cmd(self, twist):
        self.robot.command(twist)
        self.commands.append((time.monotonic(), twist.linear.x, twist.angular.z))

    def tick(self):
        period = 1.0 / ODOM_HZ
        now_wall = time.monotonic()
        if now_wall - self.last_tick_wall < period:
            time.sleep(period - (now_wall - self.last_tick_wall))
        now_wall = time.monotonic()
        dt = min(now_wall - self.last_tick_wall, 0.1)
        self.last_tick_wall = now_wall
        now = self.node.get_clock().now().to_msg()
        self.robot.step(dt)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.robot.odom_x
        odom.pose.pose.position.y = self.robot.odom_y
        odom.pose.pose.position.z = 0.06205
        odom.pose.pose.orientation.z = math.sin(self.robot.odom_yaw / 2)
        odom.pose.pose.orientation.w = math.cos(self.robot.odom_yaw / 2)
        odom.twist.twist.linear.x = self.robot.v
        odom.twist.twist.angular.z = self.robot.w
        self.odom_pub.publish(odom)

        monotonic = time.monotonic()
        if monotonic - self.last_scan >= 1.0 / LIDAR_HZ:
            self.last_scan = monotonic
            ranges = self.grid.raycast(self.robot.x, self.robot.y, self.robot.yaw, self.angles)
            ranges = np.clip(ranges + self.rng.normal(0, 0.01, LIDAR_BEAMS), LIDAR_RANGE_MIN, LIDAR_RANGE_MAX)
            dropout = self.rng.random(LIDAR_BEAMS) < 0.02
            ranges[dropout] = LIDAR_RANGE_MAX
            scan = LaserScan()
            scan.header.stamp = now
            scan.header.frame_id = 'laser_frame'
            scan.angle_min, scan.angle_max = float(self.angles[0]), float(self.angles[-1])
            scan.angle_increment = float(self.angles[1] - self.angles[0])
            scan.scan_time = 1.0 / LIDAR_HZ
            scan.range_min, scan.range_max = LIDAR_RANGE_MIN, LIDAR_RANGE_MAX
            scan.ranges = ranges.astype(np.float32).tolist()
            self.scan_pub.publish(scan)

        self.samples += 1
        self.trajectory.append((monotonic, self.robot.x, self.robot.y, self.robot.yaw))
        if self.robot.footprint_cells(self.grid):
            self.collision_samples += 1
        error = self.localisation_error()
        if error is not None:
            self.pose_errors.append(error)

    def localisation_error(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return (math.hypot(t.x - self.robot.x, t.y - self.robot.y),
                abs((yaw - self.robot.yaw + math.pi) % (2 * math.pi) - math.pi))

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        # Stamped slightly in the past: AMCL rejects hints newer than the odom TF.
        msg.header.stamp = (self.node.get_clock().now() - rclpy.time.Duration(nanoseconds=50_000_000)).to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.robot.x
        msg.pose.pose.position.y = self.robot.y
        msg.pose.pose.orientation.z = math.sin(self.robot.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.robot.yaw / 2)
        msg.pose.covariance = [0.05, 0, 0, 0, 0, 0,
                               0, 0.05, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.1]
        self.pose_pub.publish(msg)

    def startup_tick(self):
        """Sensors must run before Nav2 activates: costmaps need map->odom from AMCL."""
        self.tick()
        monotonic = time.monotonic()
        if monotonic - self.last_pose_hint >= 0.5:
            self.last_pose_hint = monotonic
            self.publish_initial_pose()

    def wait_localised(self, timeout=25.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.startup_tick()
            rclpy.spin_once(self.node, timeout_sec=0.005)
            error = self.localisation_error()
            if error and error[0] < 0.15 and error[1] < 0.2:
                self.initial_error = error
                return error
        return self.localisation_error()

    def wait_stack_ready(self, session, timeout=120.0):
        """All nine lifecycle nodes must be active.

        Publishers, services and the action server already exist while a node is
        merely configured, so graph checks alone accept the stack too early and
        bt_navigator rejects the goal as inactive.
        """
        deadline = time.monotonic() + timeout
        states, missing = {}, set()
        while time.monotonic() < deadline:
            self.startup_tick()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            session.alive()
            states = session.poll_states()
            missing = {name for name in NAV2_NODES if states.get(name) != 3}
            if not missing:
                if not self.node.get_publishers_info_by_topic('/cmd_vel'):
                    missing.add('cmd_vel_publisher')
                elif not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
                    missing.add('map_to_odom_tf')
                elif not self.action.wait_for_server(timeout_sec=0.0):
                    missing.add('navigate_to_pose_server')
                else:
                    return states
        raise RuntimeError(f'Nav2 not ready after {timeout}s, missing {sorted(missing)}; '
                           f'states={states}; logs in {self.log_dir}')

    def run(self):
        name = self.scenario['name']
        self.log_dir = self.log_root / name
        self.log_dir.mkdir(parents=True, exist_ok=True)
        map_yaml = self.grid.write(self.log_dir, name)
        result = {'scenario': name, 'start': list(self.scenario['start']), 'goal': list(self.scenario['goal'])}
        # FastDDS occasionally loses a lifecycle change_state response on this host; retry once.
        states = None
        for attempt in (1, 2):
            self.session = Nav2Session(self.node, map_yaml, self.params_file, self.log_dir)
            try:
                states = self.wait_stack_ready(self.session)
                break
            except RuntimeError as exc:
                result[f'startup_attempt_{attempt}_error'] = str(exc)
                self.session.close()
                self.session = None
                if attempt == 2:
                    result['outcome'] = 'STARTUP_FAILED'
                    result['logs'] = str(self.log_dir)
                    return result
        result['nav2_active'] = all(state == 3 for state in states.values())
        result['nav2_states'] = states
        try:
            error = self.wait_localised()
            result['initial_localisation_error_m'] = round(error[0], 3) if error else None
            if not error or error[0] >= 0.3:
                result['outcome'] = 'LOCALISATION_FAILED'
                result['logs'] = str(self.log_dir)
                return result

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = self.scenario['goal'][:2]
            goal.pose.pose.orientation.z = math.sin(self.scenario['goal'][2] / 2)
            goal.pose.pose.orientation.w = math.cos(self.scenario['goal'][2] / 2)

            assert self.action.wait_for_server(timeout_sec=10), 'navigate_to_pose server missing'
            handle = self.action.send_goal_async(goal)
            start = time.monotonic()
            while not handle.done() and time.monotonic() - start < 20:
                self.tick()
                rclpy.spin_once(self.node, timeout_sec=0.005)
            if not handle.done() or not handle.result().accepted:
                result['outcome'] = 'GOAL_REJECTED'
                result['logs'] = str(self.log_dir)
                return result
            goal_handle = handle.result()
            outcome_future = goal_handle.get_result_async()

            self.trajectory.clear()
            self.commands.clear()
            self.plans.clear()
            self.pose_errors.clear()
            self.collision_samples = 0
            self.samples = 0
            deadline = start + self.timeout
            still_since = None
            max_still = 0.0
            while time.monotonic() < deadline:
                self.tick()
                rclpy.spin_once(self.node, timeout_sec=0.005)
                if outcome_future.done():
                    break
                speed = math.hypot(self.robot.v, self.robot.w * WHEEL_TRACK / 2)
                if speed < 0.02:
                    still_since = still_since or time.monotonic()
                    max_still = max(max_still, time.monotonic() - still_since)
                else:
                    still_since = None

            duration = time.monotonic() - start
            succeeded = outcome_future.done() and outcome_future.result().status == 4
            if not succeeded and not outcome_future.done():
                goal_handle.cancel_goal_async()
                for _ in range(40):
                    rclpy.spin_once(self.node, timeout_sec=0.05)
            result.update(self.metrics(succeeded, duration, max_still))
            result['logs'] = str(self.log_dir)
            return result
        finally:
            if self.session is not None:
                self.session.close()

    def metrics(self, succeeded, duration, max_still):
        gx, gy, gyaw = self.scenario['goal']
        final = self.trajectory[-1] if self.trajectory else None
        path_length = 0.0
        for (_, x0, y0, _), (_, x1, y1, _) in zip(self.trajectory, self.trajectory[1:]):
            path_length += math.hypot(x1 - x0, y1 - y0)
        planned = self.plans[0] if self.plans else None
        planned_length = 0.0
        cross_track = 0.0
        if planned and len(planned.poses) > 1:
            points = np.array([[p.pose.position.x, p.pose.position.y] for p in planned.poses])
            planned_length = float(np.hypot(*np.diff(points, axis=0).T).sum())
            poses = np.array([[t[1], t[2]] for t in self.trajectory])
            if len(poses):
                cross_track = float(np.sqrt(((poses[:, None, :] - points[None, :, :]) ** 2).sum(-1)).min(axis=1).max())
        speeds = [math.hypot(v, w * WHEEL_TRACK / 2) for _, v, w in self.commands]
        angular = [w for _, _, w in self.commands]
        reversals = sum(1 for a, b in zip(angular, angular[1:])
                        if abs(a) > 0.1 and abs(b) > 0.1 and a * b < 0)
        pose_error = self.pose_errors[-1] if self.pose_errors else None
        final_distance = math.hypot(final[1] - gx, final[2] - gy) if final else None
        yaw_error = None
        if final:
            yaw_error = abs((final[3] - gyaw + math.pi) % (2 * math.pi) - math.pi)
        return {
            'outcome': 'SUCCEEDED' if succeeded else 'FAILED',
            'duration_s': round(duration, 2),
            'final_distance_to_goal_m': round(final_distance, 3) if final_distance is not None else None,
            'final_yaw_error_rad': round(yaw_error, 3) if yaw_error is not None else None,
            'path_length_m': round(path_length, 2),
            'planned_length_m': round(planned_length, 2),
            'length_ratio': round(path_length / planned_length, 2) if planned_length else None,
            'max_cross_track_m': round(cross_track, 3),
            'replans': len(self.plans),
            'collision_samples': self.collision_samples,
            'longest_stop_s': round(max_still, 2),
            'max_speed_mps': round(max(speeds), 3) if speeds else 0.0,
            'angular_reversals': reversals,
            'final_localisation_error_m': round(pose_error[0], 3) if pose_error else None,
            'mean_localisation_error_m': round(float(np.mean([e[0] for e in self.pose_errors])), 3) if self.pose_errors else None,
            'p95_localisation_error_m': round(float(np.percentile([e[0] for e in self.pose_errors], 95)), 3) if self.pose_errors else None,
            'max_localisation_error_m': round(max(e[0] for e in self.pose_errors), 3) if self.pose_errors else None,
            'max_localisation_yaw_error_rad': round(max(e[1] for e in self.pose_errors), 3) if self.pose_errors else None,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='run')
    parser.add_argument('--params', default=str(REPO / 'robot_ws/src/ldc_navigation/param/ldc_nav2.yaml'))
    parser.add_argument('--scenarios', default='all', help='comma separated scenario names')
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--out', default=None)
    parser.add_argument('--keep-logs', action='store_true')
    args = parser.parse_args()

    selected = scenarios()
    if args.scenarios != 'all':
        wanted = {name.strip() for name in args.scenarios.split(',')}
        selected = [s for s in selected if s['name'] in wanted]
        assert selected, f'No scenario matched {args.scenarios}'

    log_root = Path(args.out).parent / 'logs' if args.out else Path(tempfile.mkdtemp(prefix='nav2-closed-loop-'))
    log_root.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('nav2_closed_loop')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    results = []
    try:
        for index, scenario in enumerate(selected, 1):
            print(f'[{index}/{len(selected)}] {scenario["name"]}', flush=True)
            run = ScenarioRun(node, scenario, args.params, log_root, args.timeout, tf_buffer, action)
            try:
                results.append(run.run())
            except Exception as exc:
                results.append({'scenario': scenario['name'], 'outcome': 'HARNESS_ERROR', 'error': str(exc),
                                'logs': str(run.log_dir) if hasattr(run, 'log_dir') else None})
            finally:
                node.destroy_subscription(run.cmd_sub)
                node.destroy_subscription(run.plan_sub)
                node.destroy_publisher(run.odom_pub)
                node.destroy_publisher(run.scan_pub)
                node.destroy_publisher(run.pose_pub)
            print('    ' + json.dumps({k: v for k, v in results[-1].items() if k != 'logs'}), flush=True)
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if not args.keep_logs and not args.out:
            subprocess.run(['rm', '-rf', str(log_root)], check=False)

    succeeded = sum(1 for r in results if r.get('outcome') == 'SUCCEEDED')
    report = {
        'label': args.label,
        'params_file': args.params,
        'vehicle_connected': False,
        'synthetic_world': True,
        'scenarios_total': len(results),
        'scenarios_succeeded': succeeded,
        'logs': str(log_root),
        'results': results,
    }
    text = json.dumps(report, indent=2)
    if args.out:
        Path(args.out).write_text(text + '\n')
    print(text)
    return 0 if succeeded == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
