"""Legacy frontier diagnostics, passive while ownership moves to the Bridge.

Construction and old Bool endpoints cannot authorize motion. The frontier core
remains reusable by the new intent path; this compatibility node stays locked.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .frontier_core import (
    FrontierBlacklist,
    MapInfo,
    find_frontiers,
    goal_yaw,
    map_is_explorable,
    map_stats,
    quaternion_from_yaw,
    select_frontier,
)


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_cells', 12)
        self.declare_parameter('goal_timeout', 30.0)
        self.declare_parameter('idle_stop_seconds', 8.0)
        self.declare_parameter('max_goals', 30)
        self.declare_parameter('blacklist_radius', 0.6)
        self.declare_parameter('blacklist_seconds', 60.0)
        self.declare_parameter('robot_radius', 0.15)
        self.declare_parameter('min_goal_distance', 0.3)
        self.declare_parameter('max_goal_distance', 3.0)
        self.declare_parameter('goal_clearance', 0.35)
        self.declare_parameter('speed_limit', 0.25)
        self.declare_parameter('bootstrap_enabled', True)
        self.declare_parameter('bootstrap_angular', 0.4)
        self.declare_parameter('bootstrap_duration', 3.0)
        self.declare_parameter('bootstrap_timeout', 8.0)
        self.declare_parameter('bootstrap_max_attempts', 2)
        self.declare_parameter('bootstrap_interval', 2.0)

        def value(name):
            return self.get_parameter(name).value

        self.map_topic = str(value('map_topic'))
        self.base_frame = str(value('base_frame'))
        self.min_frontier_cells = int(value('min_frontier_cells'))
        self.goal_timeout = float(value('goal_timeout'))
        self.idle_stop_seconds = float(value('idle_stop_seconds'))
        self.max_goals = int(value('max_goals'))
        self.robot_radius = float(value('robot_radius'))
        self.min_goal_distance = float(value('min_goal_distance'))
        self.max_goal_distance = float(value('max_goal_distance'))
        self.goal_clearance = float(value('goal_clearance'))
        self.speed_limit = float(value('speed_limit'))
        self.bootstrap_enabled = bool(value('bootstrap_enabled'))
        self.bootstrap_angular = float(value('bootstrap_angular'))
        self.bootstrap_duration = float(value('bootstrap_duration'))
        self.bootstrap_timeout = float(value('bootstrap_timeout'))
        self.bootstrap_max_attempts = int(value('bootstrap_max_attempts'))
        self.bootstrap_interval = float(value('bootstrap_interval'))
        self.blacklist = FrontierBlacklist(float(value('blacklist_radius')))
        self.blacklist_seconds = float(value('blacklist_seconds'))

        self.map_data = None
        self.info = None
        self.map_at = 0.0
        self.coverage_unknown_ratio = 1.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/exploration/status', 10)
        # 仅空白图 bootstrap 使用；仍经 cmd_vel_gate 的安全门控。
        self.bootstrap_twist_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/exploration/enable', self.on_enable, 10)
        self.create_subscription(Bool, '/exploration/stop', self.on_stop, 10)
        self.create_subscription(Bool, '/exploration/cancel', self.on_stop, 10)

        self.enabled = False
        self.stopped = False
        self.state = 'disabled'
        self.message = '等待控制桥统一授权；旧 Bool 启用接口不可授予运动'
        self.goal_handle = None
        self.goal_future = None
        self.goal_token = 0
        self.goal_started = 0.0
        self.current_target = None
        self.current_cluster = None
        self.goals_sent = 0
        self.goals_failed = 0
        self.idle_since = time.monotonic()
        self.frontiers = 0
        self.last_feedback = None
        self.bootstrap_active = False
        self.bootstrap_started = 0.0
        self.bootstrap_attempts = 0
        self.bootstrap_done = False
        self.bootstrap_next_at = 0.0
        self.create_timer(1.0, self.tick)
        self.create_timer(0.05, self.bootstrap_tick)
        self.create_timer(0.5, self.publish_status)

    # ---- callbacks ----

    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        if (info.width <= 0 or info.height <= 0 or
                not math.isfinite(info.resolution) or info.resolution <= 0 or
                len(msg.data) != info.width * info.height):
            self.map_data = None
            self.info = None
            self.message = '收到无效地图，等待 GMapping 重新发布'
            return
        self.map_data = msg.data
        self.info = MapInfo(int(info.width), int(info.height), float(info.resolution),
                            float(info.origin.position.x), float(info.origin.position.y))
        self.map_at = time.monotonic()
        stats = map_stats(self.map_data, self.info)
        self.coverage_unknown_ratio = stats.unknown_ratio

    def on_enable(self, msg: Bool) -> None:
        if msg.data:
            self.message = '旧 Bool 启用接口不可授予运动；请使用控制桥的显式授权'
        else:
            self.enabled = False
            self.state = 'disabled'
            self.message = '自动建图已暂停，手动模式'
            self.stop_bootstrap()
            self.cancel_current()

    def on_stop(self, msg: Bool) -> None:
        if msg.data:
            self.enabled = False
            self.stopped = True
            self.state = 'stopped'
            self.message = '收到停止请求；已取消当前目标'
            self.stop_bootstrap()
            self.cancel_current()
        else:
            self.message = '恢复数据不恢复运动；等待控制桥的显式授权'

    # ---- blank-map bootstrap (low-speed in-place rotation) ----

    def start_bootstrap(self, now: float) -> None:
        self.bootstrap_active = True
        self.bootstrap_started = now
        self.bootstrap_attempts += 1
        self.state = 'bootstrapping'
        self.message = (f'空白地图 bootstrap：原地旋转 {self.bootstrap_angular:.2f} rad/s '
                        f'{self.bootstrap_duration:.1f}s（第 {self.bootstrap_attempts} 次）')

    def stop_bootstrap(self) -> None:
        if not self.bootstrap_active:
            return
        self.bootstrap_active = False
        twist = Twist()
        self.bootstrap_twist_pub.publish(twist)

    def finish_bootstrap(self, success: bool, now: float, reason: str = '') -> None:
        if not self.bootstrap_active:
            return
        self.bootstrap_active = False
        # Always send one zero after the bootstrap command window.
        self.bootstrap_twist_pub.publish(Twist())
        if success:
            self.bootstrap_done = True
            self.state = 'exploring'
            self.idle_since = now
            self.message = 'bootstrap 已生成初始地图，开始 frontier 探索'
            return
        if self.bootstrap_attempts >= self.bootstrap_max_attempts:
            self.state = 'waiting_for_map'
            self.message = (reason or 'bootstrap 超时') + '；已达最大尝试次数，请手动移动一小段'
        else:
            self.state = 'bootstrapping_wait'
            self.bootstrap_next_at = now + self.bootstrap_interval
            self.message = (reason or 'bootstrap 超时') + '；稍后重试'

    def bootstrap_tick(self) -> None:
        if not self.bootstrap_active or not self.enabled or self.stopped:
            return
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.bootstrap_angular
        self.bootstrap_twist_pub.publish(msg)

    def robot_xy(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', self.base_frame, rclpy.time.Time())
        except Exception:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    # ---- goal lifecycle ----

    def cancel_current(self) -> None:
        self.goal_token += 1
        handle, self.goal_handle = self.goal_handle, None
        self.goal_future = None
        self.current_target = None
        self.current_cluster = None
        self.goal_started = 0.0
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def fail_current(self, reason: str) -> None:
        target = self.current_target
        if target is not None:
            self.blacklist.add(target[0], target[1], time.monotonic(), self.blacklist_seconds)
        self.goals_failed += 1
        self.idle_since = time.monotonic()
        self.message = reason
        self.state = 'exploring'
        self.cancel_current()

    def send_goal(self, cluster, robot_xy) -> None:
        target = cluster.target
        yaw = goal_yaw(robot_xy, target)
        z, w = quaternion_from_yaw(yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(target[0])
        goal.pose.pose.position.y = float(target[1])
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        self.goal_token += 1
        token = self.goal_token
        self.current_target = (float(target[0]), float(target[1]))
        self.current_cluster = cluster
        self.goal_started = time.monotonic()
        self.last_feedback = None
        self.goals_sent += 1
        self.state = 'goal_active'
        self.message = f'发送 frontier 目标 ({target[0]:.2f}, {target[1]:.2f})，簇大小 {cluster.size}'
        future = self.action.send_goal_async(goal, feedback_callback=lambda msg: self.on_feedback(msg, token))
        self.goal_future = future
        future.add_done_callback(lambda result: self.on_goal_accepted(result, token))

    def on_feedback(self, msg, token: int) -> None:
        if token != self.goal_token:
            return
        feedback = getattr(msg, 'feedback', None)
        if feedback is not None:
            self.last_feedback = {
                'distance_remaining': getattr(feedback, 'distance_remaining', None),
                'number_of_recoveries': getattr(feedback, 'number_of_recoveries', None),
            }

    def on_goal_accepted(self, future, token: int) -> None:
        if token != self.goal_token:
            # Even a revoked request can be accepted remotely after cancellation.
            # Discarding this response would orphan an executing Nav2 goal.
            try:
                handle = future.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(f'晚到目标取消未确认：{error}')
            return
        self.goal_future = None
        try:
            handle = future.result()
        except Exception as error:
            self.fail_current('目标受理失败：' + str(error))
            return
        if not handle.accepted:
            self.fail_current('Nav2 拒绝 frontier 目标')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda result: self.on_goal_result(result, token))

    def on_goal_result(self, future, token: int) -> None:
        if token != self.goal_token:
            return
        try:
            result = future.result()
            status = result.status
        except Exception as error:
            self.fail_current('目标结果异常：' + str(error))
            return
        self.goal_handle = None
        self.goal_future = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.current_target = None
            self.current_cluster = None
            self.goal_started = 0.0
            self.state = 'exploring'
            self.message = 'frontier 目标到达，继续探索'
            self.idle_since = time.monotonic()
        else:
            self.fail_current(f'frontier 目标失败（状态 {status}）')

    # ---- main loop ----

    def tick(self) -> None:
        now = time.monotonic()
        if not self.enabled or self.stopped:
            self.publish_status()
            return
        if self.map_data is None or self.info is None:
            self.state = 'waiting_for_map'
            self.message = '等待 GMapping 发布 /map'
            self.idle_since = now
            self.publish_status()
            return
        if self.state == 'waiting_for_map':
            self.idle_since = now
        stats = map_stats(self.map_data, self.info)
        self.coverage_unknown_ratio = stats.unknown_ratio
        usable = map_is_explorable(stats)
        if self.bootstrap_active:
            elapsed = now - self.bootstrap_started
            if usable:
                self.finish_bootstrap(True, now)
            elif elapsed >= min(self.bootstrap_duration, self.bootstrap_timeout):
                self.finish_bootstrap(False, now, 'bootstrap 结束但地图仍为空')
                self.publish_status()
                return
            else:
                self.state = 'bootstrapping'
                self.publish_status()
                return
        if not usable:
            # Never start a bootstrap while a Nav2 goal is still active; the
            # bootstrap owns /cmd_vel_auto and must not fight the controller.
            if self.goal_handle is not None or self.goal_future is not None:
                self.cancel_current()
                self.state = 'waiting_for_map'
                self.message = '地图不可用，已取消当前目标'
                self.publish_status()
                return
            if (self.bootstrap_enabled and not self.bootstrap_done and
                    self.bootstrap_attempts < self.bootstrap_max_attempts and
                    now >= self.bootstrap_next_at):
                self.start_bootstrap(now)
                self.publish_status()
                return
            self.state = 'waiting_for_map'
            if self.bootstrap_enabled:
                self.message = '等待初始地图/请先手动移动；bootstrap 不可用或已达次数上限'
            else:
                self.message = '等待初始地图/请先手动移动；bootstrap 已关闭'
            if self.goal_handle is not None or self.goal_future is not None:
                self.cancel_current()
            self.publish_status()
            return
        if self.goal_handle is not None or self.goal_future is not None:
            if now - self.goal_started > self.goal_timeout:
                self.fail_current(f'frontier 目标 {self.goal_timeout:.0f}s 未完成，加入黑名单')
            self.publish_status()
            return
        if self.goals_sent >= self.max_goals:
            self.state = 'finished'
            self.message = f'已达到 max_goals={self.max_goals}'
            self.publish_status()
            return
        robot = self.robot_xy()
        if robot is None:
            self.state = 'waiting_for_tf'
            self.message = '等待 map→base_footprint TF'
            self.publish_status()
            return
        clusters = find_frontiers(
            self.map_data, self.info, robot,
            min_frontier_cells=self.min_frontier_cells,
            min_robot_distance=0.25,
            min_goal_distance=self.min_goal_distance,
            max_goal_distance=self.max_goal_distance,
            robot_radius=self.robot_radius,
            goal_clearance=self.goal_clearance)
        self.frontiers = len(clusters)
        if not clusters:
            if self.idle_since <= 0:
                self.idle_since = now
            if now - self.idle_since >= self.idle_stop_seconds:
                self.state = 'finished'
                self.message = '没有可探索的 frontier，自动建图结束'
            else:
                self.state = 'idle'
                self.message = '暂无可达 frontier，继续等待地图更新'
            self.publish_status()
            return
        self.idle_since = 0.0
        cluster = select_frontier(clusters, self.blacklist, now)
        if cluster is None:
            self.state = 'idle'
            self.message = '可用 frontier 均在黑名单中'
            self.publish_status()
            return
        if not self.action.server_is_ready():
            self.state = 'waiting_for_nav2'
            self.message = '等待 Nav2 /navigate_to_pose action server'
            self.publish_status()
            return
        self.send_goal(cluster, robot)
        self.publish_status()

    def publish_status(self) -> None:
        payload = {
            'state': self.state,
            'frontiers': int(self.frontiers),
            'goals_sent': int(self.goals_sent),
            'goals_failed': int(self.goals_failed),
            'current_goal': ({'x': self.current_target[0], 'y': self.current_target[1]}
                             if self.current_target else None),
            'coverage_unknown_ratio': round(float(self.coverage_unknown_ratio), 4),
            'blacklist_count': int(self.blacklist.count(time.monotonic())),
            'enabled': bool(self.enabled),
            'message': self.message,
            'speed_limit': float(self.speed_limit),
            'bootstrap_active': bool(self.bootstrap_active),
            'bootstrap_attempts': int(self.bootstrap_attempts),
            'bootstrap_done': bool(self.bootstrap_done),
            'last_feedback': self.last_feedback,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
