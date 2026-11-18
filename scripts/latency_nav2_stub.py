#!/usr/bin/env python3
"""D 工单合成 Nav2 替身（仅私有测试域使用，不属于产品代码）。

提供 Bridge 控制链路所需的最小 Nav2 表面：
- 9 个节点的 get_state/change_state 生命周期服务（configure→inactive→active）；
- /navigate_to_pose Action 服务器：受理目标并保持执行，收到取消后按
  LATENCY_CANCEL_DELAY（默认 0.05s）返回 CANCELED 终态；
- GetParameters 服务（LATENCY_PARAMS=online 时创建；offline 时完全不创建，
  模拟参数服务离线）。服务名按 RuntimeProfile.PARAMETERS 的无前导斜杠键拼接；
  注意 Bridge.request_effective_params 用带前导斜杠的键拼出 '//...' 非法服务名，
  客户端创建即失败（当前工作区实测），本替身如实记录实际被调用的服务；
- /map（transient_local 全 free 栅格）、map→odom TF、/amcl_pose（复制 /odom
  最新位姿与时间戳，协方差对角 0.01，满足 LocalizationHealth 校验）；
- 按 stdin JSON 行控制（{"velocityTopic":..., "velocityEnabled":..., "linearX":...}）
  指定的 velocityTopic 以 20Hz 发布非零 Twist，模拟 Nav2 控制器输出
  （Bridge on_auto 转发到 /cmd_vel）。

多线程执行器 + ReentrantCallbackGroup：目标执行循环不阻塞生命周期/参数服务。
"""
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist  # noqa: E402
from lifecycle_msgs.srv import GetState, ChangeState  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Odometry  # noqa: E402
from rcl_interfaces.msg import ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import GetParameters  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

import navigation_bridge as nb  # noqa: E402
from navigation_profile import RuntimeProfile  # noqa: E402

STATE_LABELS = {1: 'unconfigured', 2: 'inactive', 3: 'active', 4: 'finalized'}

# (类型, 值)；未列出的请求名返回 PARAMETER_NOT_SET。
PARAM_VALUES = {
    'controller_server': {
        'controller_plugins': ('string_array', ['FollowPath']),
        'controller_frequency': ('double', 20.0),
        'odom_topic': ('string', '/odom'),
        'FollowPath.plugin': ('string', 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController'),
        'FollowPath.min_vel_x': ('double', 0.0),
        'FollowPath.max_vel_x': ('double', 0.26),
        'FollowPath.min_speed_xy': ('double', 0.0),
        'FollowPath.max_speed_xy': ('double', 0.26),
        'FollowPath.trans_stopped_velocity': ('double', 0.25),
        'progress_checker.plugin': ('string', 'nav2_controller::SimpleProgressChecker'),
        'progress_checker.required_movement_radius': ('double', 0.5),
        'progress_checker.movement_time_allowance': ('double', 10.0),
    },
    'planner_server': {
        'planner_plugins': ('string_array', ['GridBased']),
        'GridBased.plugin': ('string', 'nav2_navfn_planner/NavfnPlanner'),
    },
    'amcl': {
        'base_frame_id': ('string', 'base_footprint'),
        'odom_frame_id': ('string', 'odom'),
        'scan_topic': ('string', '/scan'),
        'update_min_d': ('double', 0.25),
        'update_min_a': ('double', 0.2),
    },
    'map_server': {'yaml_filename': ('string', 'latency-stub-map.yaml')},
    'local_costmap/local_costmap': {
        'robot_base_frame': ('string', 'base_link'), 'robot_radius': ('double', 0.22),
        'footprint': ('string', '[]'), 'resolution': ('double', 0.05),
        'inflation_layer.inflation_radius': ('double', 0.35),
    },
    'global_costmap/global_costmap': {
        'robot_base_frame': ('string', 'base_link'), 'robot_radius': ('double', 0.22),
        'footprint': ('string', '[]'), 'resolution': ('double', 0.05),
        'inflation_layer.inflation_radius': ('double', 0.35),
    },
}


def make_value(spec):
    value = ParameterValue()
    if spec is None:
        value.type = ParameterType.PARAMETER_NOT_SET
        return value
    kind, raw = spec
    if kind == 'double':
        value.type = ParameterType.PARAMETER_DOUBLE; value.double_value = float(raw)
    elif kind == 'string':
        value.type = ParameterType.PARAMETER_STRING; value.string_value = str(raw)
    elif kind == 'bool':
        value.type = ParameterType.PARAMETER_BOOL; value.bool_value = bool(raw)
    elif kind == 'integer':
        value.type = ParameterType.PARAMETER_INTEGER; value.integer_value = int(raw)
    elif kind == 'string_array':
        value.type = ParameterType.PARAMETER_STRING_ARRAY; value.string_array_value = list(raw)
    elif kind == 'double_array':
        value.type = ParameterType.PARAMETER_DOUBLE_ARRAY; value.double_array_value = list(raw)
    return value


class Stub:
    def __init__(self, node, log_path):
        self.node = node
        self.log_handle = open(log_path, 'a', buffering=1, encoding='utf-8') if log_path else None
        self.log_lock = threading.Lock()
        self.params_online = os.environ.get('LATENCY_PARAMS', 'online') == 'online'
        self.cancel_delay = float(os.environ.get('LATENCY_CANCEL_DELAY', '0.05'))
        self.states = {name: 1 for name in nb.NODES}
        self.latest_odom = None
        self.vel_topic = None
        self.vel_publisher = None
        self.vel_enabled = False
        self.vel_x = 0.2
        self.tick_count = 0
        group = ReentrantCallbackGroup()
        for name in nb.NODES:
            node.create_service(GetState, f'/{name}/get_state',
                                self.make_get_state(name), callback_group=group)
            node.create_service(ChangeState, f'/{name}/change_state',
                                self.make_change_state(name), callback_group=group)
        self.param_nodes = sorted(set(RuntimeProfile.PARAMETERS) |
                                  {key.lstrip('/') for key in nb.EFFECTIVE_PARAM_TARGETS})
        if self.params_online:
            for name in self.param_nodes:
                node.create_service(GetParameters, f'/{name}/get_parameters',
                                    self.make_get_params(name), callback_group=group)
        self.action_server = ActionServer(
            node, NavigateToPose, '/navigate_to_pose', self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel, callback_group=group)
        node.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.control_thread = threading.Thread(target=self.read_control, daemon=True)
        self.control_thread.start()
        self.amcl_pub = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        self.tf = TransformBroadcaster(node)
        self.map_pub = node.create_publisher(
            OccupancyGrid, '/map',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publish_map()
        node.create_timer(0.05, self.fast_tick)
        self.log('stub_init', paramsOnline=self.params_online, cancelDelay=self.cancel_delay,
                 paramServices=[f'/{n}/get_parameters' for n in self.param_nodes] if self.params_online else [])

    def log(self, event, **fields):
        if self.log_handle is None:
            return
        record = {'event': event, 'mono': time.monotonic(), 'wall': time.time(), **fields}
        with self.log_lock:
            try:
                self.log_handle.write(json.dumps(record, ensure_ascii=False) + '\n')
            except (OSError, ValueError):
                pass

    # ---- 生命周期 ----

    def make_get_state(self, name):
        def handler(request, response):
            state = self.states[name]
            response.current_state.id = state
            response.current_state.label = STATE_LABELS.get(state, '')
            return response
        return handler

    def make_change_state(self, name):
        def handler(request, response):
            transition = request.transition.id
            current = self.states[name]
            if transition == 1 and current == 1:
                self.states[name] = 2
            elif transition == 3 and current == 2:
                self.states[name] = 3
            elif transition == 2 and current == 3:
                self.states[name] = 2
            elif transition == 4 and current == 2:
                self.states[name] = 1
            else:
                response.success = False
                return response
            self.log('lifecycle_transition', node=name, transition=transition,
                     state=self.states[name])
            response.success = True
            return response
        return handler

    # ---- 参数服务 ----

    def make_get_params(self, name):
        def handler(request, response):
            self.log('get_parameters_called', service=f'/{name}/get_parameters',
                     names=list(request.names))
            table = PARAM_VALUES.get(name, {})
            response.values = [make_value(table.get(key)) for key in request.names]
            return response
        return handler

    # ---- Action ----

    def on_goal(self, goal_request):
        self.log('goal_received', x=goal_request.pose.pose.position.x,
                 y=goal_request.pose.pose.position.y)
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.log('cancel_received', goalId=bytes(goal_handle.goal_id.uuid).hex())
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        self.log('goal_executing', goalId=goal_id)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                time.sleep(self.cancel_delay)
                goal_handle.canceled()
                self.log('goal_canceled', goalId=goal_id)
                result = NavigateToPose.Result()
                if hasattr(result, 'error_code'):
                    result.error_code = 0
                if hasattr(result, 'error_msg'):
                    result.error_msg = ''
                return result
            time.sleep(0.02)
        goal_handle.abort()
        return NavigateToPose.Result()

    # ---- 环境输入 ----

    def on_odom(self, msg):
        self.latest_odom = msg

    def read_control(self):
        try:
            for line in sys.stdin:
                try:
                    self.apply_control(json.loads(line))
                except (ValueError, TypeError):
                    pass
        except (OSError, ValueError):
            pass

    def apply_control(self, control):
        if not isinstance(control, dict):
            return
        topic = control.get('velocityTopic')
        if topic != self.vel_topic:
            if self.vel_publisher is not None:
                self.node.destroy_publisher(self.vel_publisher)
                self.vel_publisher = None
            self.vel_topic = topic
            if topic:
                self.vel_publisher = self.node.create_publisher(Twist, topic, 10)
        self.vel_enabled = bool(control.get('velocityEnabled'))
        self.vel_x = float(control.get('linearX', 0.2))
        self.log('control', velocityTopic=self.vel_topic, velocityEnabled=self.vel_enabled,
                 linearX=self.vel_x)

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.node.get_clock().now().to_msg()
        grid.info.resolution = 0.05
        grid.info.width = 200
        grid.info.height = 200
        grid.info.origin.position.x = -5.0
        grid.info.origin.position.y = -5.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (200 * 200)
        self.map_pub.publish(grid)

    def fast_tick(self):
        now = self.node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.tick_count += 1
        # Bridge.begin 会清空地图缓存并要求"本轮 map_server 重新发布"；
        # 真实环境每轮重启 map_server，这里以 1Hz 重发布等效覆盖该语义。
        if self.tick_count % 20 == 0:
            self.publish_map()
        if self.tick_count % 2 == 0 and self.latest_odom is not None:
            odom = self.latest_odom
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = odom.header.stamp
            pose.pose.pose = odom.pose.pose
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = covariance[35] = 0.01
            covariance[14] = covariance[21] = covariance[28] = 1e-6
            pose.pose.covariance = covariance
            self.amcl_pub.publish(pose)
        if self.vel_enabled and self.vel_publisher is not None:
            twist = Twist()
            twist.linear.x = self.vel_x
            self.vel_publisher.publish(twist)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('latency_nav2_stub')
    stub = Stub(node, args.log)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    print('stub-ready', flush=True)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        stub.log('stub_exit')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()