#!/usr/bin/env python3
"""Console-owned ROS action client and velocity authority; JSON lines over stdio.

第二阶段控制权模型：
- 运动租约绑定唯一的控制页面会话（sessionId）。只有会话所有者的心跳可以续期；
  其他页面默认只读，任意页面仍可急停/取消（安全类操作不校验归属）。
- 接管是显式操作：先取消当前运动并等待终态确认，再生成新租约；旧租约随即失效。
- 断线、后台重启、传感器恢复后保持锁定，重新运动需要明确启用
  （启用遥控或提交新目标都算明确启用，且只在无所有者时自动授予）。
- 事件环记录启动、定位、目标受理、取消、失败和完成，关联 runId/goalId/原因。
"""
import collections
import json
import math
import os
import queue
import sys
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState, ChangeState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

if __package__:
    from .sensor_health import MeasurementHealth, scan_error, odom_error, automatic_velocity_valid
    from .localization_health import LocalizationHealth, stamp_ns, planar_pose
    from .navigation_profile import RuntimeProfile
    from .nav_stuck_policy import NoProgressPolicy
else:
    from sensor_health import MeasurementHealth, scan_error, odom_error, automatic_velocity_valid
    from localization_health import LocalizationHealth, stamp_ns, planar_pose
    from navigation_profile import RuntimeProfile
    from nav_stuck_policy import NoProgressPolicy

# 运行证据清单（F）：任何异常都不得影响控制流程，因此整体 try/except 后置空。
try:
    if __package__:
        from . import run_manifest
    else:
        import run_manifest
except Exception:  # noqa: BLE001 - 证据落盘是附加能力
    run_manifest = None

# 运动租约有效期：浏览器后台标签页会节流定时器，2 秒极易误判为失联。
# 默认 5 秒，可用 ROBOT_LEASE_TTL_SECONDS 覆盖。
LEASE_TTL_SECONDS = float(os.environ.get('ROBOT_LEASE_TTL_SECONDS', '5'))

# 每次运行的日志目录（由 scripts/run_real_logged.sh 或 offline_console.py 注入）。
# 留空表示不落盘，仅保留内存事件环。
LOG_DIR = os.environ.get('ROBOT_CONSOLE_LOG_DIR', '').strip()
SAMPLE_INTERVAL_SECONDS = 1.0
# /scan 有效样本年龄上限。固件整圈发布偶发丢帧时可短暂放宽，
# 但默认必须与 docs/LDC数据有效性修复记录.md 的 1.5s 安全边界一致。
SCAN_MAX_AGE_SECONDS = float(os.environ.get('ROBOT_SCAN_MAX_AGE', '1.5'))
# 目标所在的原始地图栅格必须是 free；occupied/unknown/越界都在 Action 之前拒绝。
GOAL_FREE_MAX = 64

# D：启动后实读这些运行中的参数值，防止"源码/配置改了但运行没生效"。
EFFECTIVE_PARAM_TARGETS = {
    '/controller_server': ('FollowPath.min_vel_x', 'FollowPath.max_vel_x',
                           'FollowPath.min_speed_xy', 'FollowPath.max_speed_xy',
                           'progress_checker.plugin', 'progress_checker.required_movement_radius',
                           'progress_checker.movement_time_allowance'),
    '/local_costmap/local_costmap': ('robot_radius', 'inflation_layer.inflation_radius'),
    '/global_costmap/global_costmap': ('robot_radius', 'inflation_layer.inflation_radius'),
}


def param_value(value):
    """rcl_interfaces/ParameterValue → Python 值；PARAMETER_NOT_SET 返回 None。"""
    if value.type == ParameterType.PARAMETER_STRING:
        return value.string_value
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if value.type == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if value.type == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if value.type == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    return None


NODES = ('map_server', 'amcl', 'controller_server', 'planner_server', 'smoother_server',
         'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother')


class Bridge(rclpy.node.Node):
    def __init__(self):
        super().__init__('console_navigation_bridge')
        self.commands = queue.Queue()
        self.shutdown_requested = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.transition = None
        self.startup_done = False
        self.startup_failed = False
        self.run_id = 0
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # 本轮导航的 manifest runId（F：每次启动记录生效参数与源码指纹）。
        self.manifest_run_id = None
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop = self.create_publisher(Bool, '/emergency_stop', 10)
        self.pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.scan_at = self.odom_at = self.pose_at = self.lease_at = self.manual_at = 0.
        self.scan_health = MeasurementHealth(scan_error, max_age=SCAN_MAX_AGE_SECONDS)
        self.odom_health = MeasurementHealth(odom_error)
        self.localization_health = LocalizationHealth()
        self.runtime_profile = RuntimeProfile(self)
        self.localization_ok_prev = False
        self.last_stop = None
        self.lock_kind = 'idle'

        # ---- 离线诊断留档（JSONL，一行一条，便于事后分析）----
        self._log_handles = {}
        self._scan_arrivals = collections.deque(maxlen=400)
        self._odom_arrivals = collections.deque(maxlen=800)
        self._next_sample_at = 0.
        self._open_logs()
        # /map 只用于目标可通行性预检，不参与定位；由 map_server 的 transient local 缓存提供。
        self.map_grid = None
        self.map_info = None
        self.map_received_at = 0.
        self.map_invalid_prev = False
        self.pose_requested = float('inf')
        # 初始位姿用"消息 stamp"判定 AMCL 是否接受，而不是收包时刻：
        # 用收包时刻时，请求之前产生的旧消息也会被当成"已确认"（2026-11-11 18:04 现场）。
        self.pose_requested_stamp_ns = 0
        self.last_pose_stamp_ns = 0
        self.initial_pose_msg = None
        self.initial_pose_sent_at = 0.
        self.initial_pose_failed = None
        self.initial_pose_retries = max(1, int(float(os.environ.get('ROBOT_INITIAL_POSE_RETRIES', '5'))))
        self.initial_pose_interval = float(os.environ.get('ROBOT_INITIAL_POSE_INTERVAL', '1.0'))
        self.initial_pose_retries_left = 0
        self.initial_pose_next_at = 0.
        # 两个超时必须分开：3~5s 只等"初始位姿被接受"；60s 只等"启动有进展"。
        # 此前只有一个从 nav_started 起算的 60s，页面只能看到"导航启动超时"。
        self.initial_pose_timeout = float(os.environ.get('ROBOT_INITIAL_POSE_TIMEOUT', '4'))
        self.startup_timeout = float(os.environ.get('ROBOT_STARTUP_TIMEOUT', '60'))
        self.last_progress_at = 0.
        self.startup_failure = None
        # 无位移失败保护（D）：位移测量 + 清 costmap 去重 + 实读生效参数。
        self.stuck = NoProgressPolicy()
        self.odom_pose = None
        self.clear_costmap_clients = {}
        self.effective_params = None
        self.nav_started = 0.
        self.running = False
        self.map_path = ''
        self.mode = 'locked'
        self.control_id = None
        self.velocity_sequence = -1
        self.error = ''
        self.goal_handle = None
        self.goal_id = None
        self.goal_state = 'idle'
        self.feedback = None
        self.points = []
        self.current = None
        self.cancel_waiters = []
        self.cancel_deadline = 0.
        self.pending_goal_request = None
        self.cancel_reason = ''
        self.lifecycle = {}
        self.lifecycle_polled_at = 0.
        self.prev_node_states = {}
        self.pending_states = {}
        # 控制权归属：唯一控制页面会话；心跳续期仅对所有者有效。
        self.owner = None
        self.localization_confirmed = False
        self.sensors_ok_prev = True
        self.last_denied_at = {}
        self.events = collections.deque(maxlen=1000)
        self.event_seq = 0
        self.lifecycle_clients = {name: self.create_client(GetState, f'/{name}/get_state') for name in NODES}
        self.transition_clients = {name: self.create_client(ChangeState, f'/{name}/change_state') for name in NODES}
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self.on_auto, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self.tick)
        self.create_timer(.5, self.check_lifecycle)
        threading.Thread(target=self.read_commands, daemon=True).start()

    # ---- 事件记录 ----

    # ---- 离线日志 ----

    def _open_logs(self):
        if not LOG_DIR:
            return
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            for name in ('bridge-events', 'bridge-samples'):
                self._log_handles[name] = open(
                    os.path.join(LOG_DIR, name + '.jsonl'), 'a', buffering=1, encoding='utf-8')
            self._append_log('bridge-samples', {
                'kind': 'session-start', 'at': time.time(),
                'pid': os.getpid(), 'leaseTtlSeconds': LEASE_TTL_SECONDS,
                'scanMaxAgeSeconds': self.scan_health.max_age,
                'odomMaxAgeSeconds': self.odom_health.max_age})
        except OSError as error:
            print('navigation_bridge: 无法写入日志目录 %s: %s' % (LOG_DIR, error), flush=True)
            self._log_handles = {}

    def _append_log(self, name, payload):
        handle = self._log_handles.get(name)
        if handle is None:
            return
        try:
            handle.write(json.dumps(payload, ensure_ascii=False) + '\n')
        except (OSError, ValueError):
            pass

    def _write_sample(self):
        now = time.monotonic()
        def arrivals(queue, window=5.0):
            recent = [t for t in queue if now - t <= window]
            gaps = [b - a for a, b in zip(recent, recent[1:])]
            return {'count5s': len(recent), 'maxGap5s': round(max(gaps), 3) if gaps else None}
        checks = self.structured_checks()
        self._append_log('bridge-samples', {
            'kind': 'tick', 'at': time.time(),
            'mode': self.mode, 'lockKind': self.lock_kind,
            'owner': bool(self.owner), 'leaseAge': round(now - self.lease_at, 2) if self.owner else None,
            'running': self.running, 'goalState': self.goal_state, 'runId': self.run_id,
            'error': self.error or None,
            'lidar': {**{k: checks['lidar'][k] for k in ('ok', 'ageSeconds', 'sampleAgeSeconds', 'detail')},
                      **arrivals(self._scan_arrivals)},
            'odometry': {**{k: checks['odometry'][k] for k in ('ok', 'ageSeconds', 'sampleAgeSeconds', 'detail')},
                         **arrivals(self._odom_arrivals)},
            'tf': {'ok': checks['tf']['ok'], 'detail': checks['tf'].get('detail')}})

    def close_logs(self):
        for handle in self._log_handles.values():
            try:
                handle.close()
            except OSError:
                pass
        self._log_handles = {}

    def record(self, event_type, reason='', **extra):
        self.event_seq += 1
        event = {'seq': self.event_seq, 'at': time.time(), 'runId': self.run_id,
                 'goalId': self.goal_id, 'type': event_type, 'reason': reason}
        for key, value in extra.items():
            if value is not None:
                event[key] = value
        self.events.append(event)
        self._append_log('bridge-events', event)

    def record_throttled(self, event_type, reason='', interval=5., **extra):
        key = (event_type, extra.get('sessionId', ''))
        now = time.monotonic()
        if now - self.last_denied_at.get(key, 0.) < interval:
            return
        self.last_denied_at[key] = now
        self.record(event_type, reason, **extra)

    def read_commands(self):
        for line in sys.stdin:
            try:
                self.commands.put(json.loads(line))
            except (ValueError, TypeError):
                pass
        self.commands.put({'op': 'shutdown'})

    def respond(self, request, **data):
        try:
            print(json.dumps({'id': request.get('id'), **data}, ensure_ascii=False), flush=True)
        except BrokenPipeError:
            self.shutdown_requested = True
            self.lock()
            sink = os.open(os.devnull, os.O_WRONLY)
            os.dup2(sink, sys.stdout.fileno())
            os.close(sink)

    def on_scan(self, msg):
        now = time.monotonic()
        self._scan_arrivals.append(now)
        if self.scan_health.observe(msg, self.get_clock().now().nanoseconds, now):
            self.scan_at = now

    def on_odom(self, msg):
        now = time.monotonic()
        self._odom_arrivals.append(now)
        if self.odom_health.observe(msg, self.get_clock().now().nanoseconds, now):
            self.odom_at = now
            self.localization_health.observe_odometry(msg, now)
            pose = planar_pose(msg.pose.pose)
            self.odom_pose = pose
            self.stuck.observe_odom(pose)

    def sensor_checks(self):
        now, ros_ns = time.monotonic(), self.get_clock().now().nanoseconds
        return {'lidar': self.scan_health.check(ros_ns, now),
                'odometry': self.odom_health.check(ros_ns, now)}

    def on_pose(self, msg):
        now = time.monotonic()
        stamp = stamp_ns(msg)
        if stamp > 0:
            self.last_pose_stamp_ns = stamp
        if not self.running or not self.localization_health.observe(msg, self.get_clock().now().nanoseconds, now):
            return
        self.pose_at = now
        self.last_progress_at = now
        # 只有"请求 stamp 之后"产生的 AMCL 位姿才算确认；延迟到达的旧消息不算。
        if (not self.localization_confirmed and math.isfinite(self.pose_requested) and
                self.pose_requested_stamp_ns > 0 and stamp >= self.pose_requested_stamp_ns):
            self.localization_confirmed = True
            self.initial_pose_retries_left = 0
            self.initial_pose_failed = None
            self.record('localization_confirmed', 'AMCL 已接受初始位置',
                        requestedStamp=self.pose_requested_stamp_ns, poseStamp=stamp)

    def on_map(self, msg):
        """缓存 map_server 的静态地图，只用于 goals 的地图内/unknown/occupied 预检。"""
        try:
            info = msg.info
            q = info.origin.orientation
            norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if (info.width <= 0 or info.height <= 0 or
                    not math.isfinite(info.resolution) or info.resolution <= 0 or
                    not all(math.isfinite(v) for v in
                            (info.origin.position.x, info.origin.position.y, q.x, q.y, q.z, q.w)) or
                    norm < 1e-12 or len(msg.data) != info.width * info.height):
                raise ValueError('地图尺寸、分辨率、原点或数据长度无效')
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.map_grid = msg.data
            self.map_info = dict(width=info.width, height=info.height,
                                 resolution=float(info.resolution),
                                 originX=float(info.origin.position.x),
                                 originY=float(info.origin.position.y), yaw=float(yaw))
            self.map_received_at = time.monotonic()
            self.map_invalid_prev = False
        except Exception as error:
            self.map_grid = None
            self.map_info = None
            if not self.map_invalid_prev:
                self.record('map_invalid', '拒绝无效地图：' + str(error))
                self.map_invalid_prev = True

    def validate_goal(self, point):
        if self.map_grid is None or self.map_info is None:
            raise ValueError('尚未收到有效地图 /map，不能发送目标')
        info = self.map_info
        x, y = float(point['x']), float(point['y'])
        dx, dy = x - info['originX'], y - info['originY']
        cos, sin = math.cos(info['yaw']), math.sin(info['yaw'])
        mx = int(math.floor((dx*cos + dy*sin) / info['resolution']))
        my = int(math.floor((-dx*sin + dy*cos) / info['resolution']))
        if not (0 <= mx < info['width'] and 0 <= my < info['height']):
            raise ValueError(f'目标 ({x:.2f}, {y:.2f}) 超出地图范围')
        value = int(self.map_grid[my * info['width'] + mx])
        if value < 0:
            raise ValueError(f'目标 ({x:.2f}, {y:.2f}) 位于未知区域')
        if value >= GOAL_FREE_MAX:
            raise ValueError(f'目标 ({x:.2f}, {y:.2f}) 位于障碍区域')
        return value

    def validate_goals(self, points, request):
        for index, point in enumerate(points):
            try:
                self.validate_goal(point)
            except ValueError as error:
                self.record('goal_rejected', str(error),
                            sessionId=request.get('sessionId'),
                            requestId=request.get('requestId'),
                            point=point, index=index)
                raise

    def localization_check(self):
        check = self.localization_health.check(self.get_clock().now().nanoseconds, time.monotonic())
        check = {**check, 'requestedStamp': self.pose_requested_stamp_ns or None,
                 'lastPoseStamp': self.last_pose_stamp_ns or None,
                 'initialPoseTimeout': self.initial_pose_failed}
        if self.initial_pose_failed:
            check = {**check, 'ok': False,
                     'code': self.initial_pose_failed.get('code', 'initial_pose_timeout'),
                     'detail': self.initial_pose_failed.get('detail', '初始位置未被接受')}
        if not self.running:
            return {**check, 'ok': False, 'code': 'stopped', 'detail': '导航未启动', 'waitingUser': False}
        if check['ok'] and not self.fresh_transform('map', 'base_footprint'):
            return {**check, 'ok': False, 'code': 'tf', 'detail': 'map → base_footprint TF 缺失或过期'}
        return check

    def fresh_transform(self, parent, child):
        try:
            transform = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            return -2 < age < 1.5
        except Exception:
            return False

    def sensors(self):
        missing = []
        checks = self.sensor_checks()
        if not checks['lidar']['ok']:
            missing.append('实时雷达 /scan：' + checks['lidar']['detail'])
        if not checks['odometry']['ok']:
            missing.append('实时里程计 /odom：' + checks['odometry']['detail'])
        if not self.fresh_transform('odom', 'laser_frame'):
            missing.append('odom → laser_frame TF')
        return missing

    def requirements(self):
        missing = self.sensors()
        now = time.monotonic()
        missing += [name + ' 未激活' for name in NODES
                    if self.lifecycle.get(name, (0, 0))[0] != 3 or
                    now - self.lifecycle.get(name, (0, 0))[1] > 6 or not self.lifecycle_clients[name].service_is_ready()]
        if not self.startup_done:
            missing.append('导航生命周期启动尚未完成')
        if not self.action.server_is_ready():
            missing.append('NavigateToPose Action')
        localization = self.localization_check()
        if not localization['ok']:
            missing.append(localization['detail'])
        if not self.fresh_transform('map', 'base_footprint'):
            missing.append('map → base_footprint TF')
        return missing

    # ---- 结构化就绪检查（P2-1）----

    def structured_checks(self):
        now = time.monotonic()

        def age(stamp):
            return round(now - stamp, 2) if stamp else None

        sensor_checks = self.sensor_checks()
        tf_odom_laser = self.fresh_transform('odom', 'laser_frame')
        tf_map_base = self.fresh_transform('map', 'base_footprint')
        tf_ok = tf_odom_laser and (tf_map_base if self.running else True)
        tf_detail = []
        if not tf_odom_laser:
            tf_detail.append('odom → laser_frame 缺失或过期')
        if self.running and not tf_map_base:
            tf_detail.append('map → base_footprint 缺失或过期')
        active_nodes = [name for name in NODES
                        if self.lifecycle.get(name, (0, 0))[0] == 3 and now - self.lifecycle.get(name, (0, 0))[1] <= 6]
        action_ready = self.action.server_is_ready()
        nav2_ok = bool(self.running and self.startup_done and not self.startup_failed and
                       len(active_nodes) == len(NODES) and action_ready)
        if not self.running:
            nav2_detail = '导航未启动'
        elif self.startup_failed:
            nav2_detail = '启动失败：' + (self.error or '请停止导航后重试')
        else:
            nav2_detail = f'{len(active_nodes)}/{len(NODES)} 节点激活' + ('' if action_ready else '；Action 未就绪')
        localization = self.localization_check()
        owner_ok = self.owner is None or now - self.lease_at <= LEASE_TTL_SECONDS
        if self.owner is None:
            control_detail = '无控制页面（任意页面可取得控制权）'
        elif owner_ok:
            control_detail = '控制页面在线'
        else:
            control_detail = '控制页面心跳失联，租约即将释放'
        return {
            **sensor_checks,
            'tf': {'ok': tf_ok, 'ageSeconds': None,
                   'detail': 'TF 树完整' if tf_ok else '；'.join(tf_detail)},
            'localization': localization,
            'nav2': {'ok': nav2_ok, 'ageSeconds': age(self.lifecycle_polled_at),
                     'detail': nav2_detail, 'activeNodes': len(active_nodes), 'totalNodes': len(NODES)},
            'control': {'ok': owner_ok, 'leaseAgeSeconds': age(self.lease_at) if self.owner else None,
                        'detail': control_detail, 'owner': self.owner, 'mode': self.mode},
        }

    def next_action(self, checks):
        if not self.running:
            return '选择地图开始导航，或先开始建图'
        if self.startup_failed:
            return '导航启动失败：请停止导航后重新启动'
        if not checks['lidar']['ok'] or not checks['odometry']['ok']:
            return '检查小车电源与 Wi-Fi，等待雷达和里程计恢复；运动保持锁定'
        if not checks['tf']['ok']:
            return '检查 TF 广播（雷达外参与里程计）'
        if not checks['nav2']['ok']:
            return '等待 Nav2 节点激活（最长 60 秒）'
        if checks['localization']['waitingUser']:
            if math.isinf(self.pose_requested):
                return '请在地图上设置初始位置和朝向'
            return '等待 AMCL 接受初始位置；长时间无响应请重新设置'
        if self.mode == 'locked' and self.error:
            return self.error
        if self.mode == 'locked':
            return '就绪：可双击地图发送目标，或启用遥控'
        return '运动进行中：由控制页面管理'

    def check_lifecycle(self):
        if not self.running:
            return
        for name, client in self.lifecycle_clients.items():
            if name in self.pending_states:
                pending, sent_at = self.pending_states[name]
                if not pending.done():
                    if time.monotonic() - sent_at < 1:
                        continue
                    client.remove_pending_request(pending)
                    pending.cancel()
            if client.service_is_ready():
                future = client.call_async(GetState.Request())
                self.pending_states[name] = (future, time.monotonic())
                generation = self.run_id
                def done(f, key=name, epoch=generation):
                    if epoch != self.run_id:
                        return
                    try:
                        state = f.result().current_state.id
                        self.lifecycle[key] = (state, time.monotonic())
                        self.lifecycle_polled_at = time.monotonic()
                        previous = self.prev_node_states.get(key)
                        if previous is not None and previous != state:
                            self.last_progress_at = time.monotonic()
                            if previous == 3 and state != 3:
                                self.record('node_inactive', f'{key} 退出激活状态（{previous} → {state}）', node=key)
                            elif previous != 3 and state == 3:
                                self.record('node_active', f'{key} 已激活', node=key)
                        self.prev_node_states[key] = state
                    except Exception:
                        pass
                future.add_done_callback(done)

    def status(self):
        missing = self.requirements() if self.running else self.sensors()
        if not self.running:
            phase = 'stopped'
        elif self.startup_failed:
            phase = 'failed'
        elif self.initial_pose_failed and not self.localization_confirmed:
            # 初始位姿超时优先于"依赖项齐全"：此时 AMCL 仍没给出新位姿，不能显示就绪。
            phase = 'failed'
        elif self.goal_state in ('accepted', 'executing', 'canceling'):
            phase = self.goal_state
        elif self.goal_state in ('failed', 'canceled') and self.error:
            phase = self.goal_state
        elif not missing:
            phase = self.goal_state if self.goal_state in ('succeeded', 'failed', 'canceled') else 'ready'
        elif self.pose_at < self.pose_requested and self.lifecycle.get('amcl', (0, 0))[0] == 3:
            phase = 'waiting_localization'
        else:
            progress_at = self.last_progress_at or self.nav_started
            phase = 'failed' if time.monotonic() - progress_at > self.startup_timeout else 'starting'
        checks = self.structured_checks()
        now = time.monotonic()
        return dict(isRunning=self.running, phase=phase, ready=self.running and not missing,
                    missing=missing, error=self.error, mapPath=self.map_path,
                    mode=self.mode, goalId=self.goal_id, goalState=self.goal_state,
                    feedback=self.feedback, current=self.current, queue=self.points,
                    localized=checks['localization']['ok'],
                    initialPoseFailure=self.initial_pose_failed,
                    startupFailure=self.startup_failure,
                    stuckPolicy=self.stuck.snapshot(),
                    effectiveParams=self.effective_params,
                    runtimeProfile=self.runtime_profile.snapshot(), lastStop=self.last_stop,
                    lockKind=self.lock_kind,
                    controlOwner=self.owner,
                    leaseAgeSeconds=round(now - self.lease_at, 2) if self.owner else None,
                    eventsSeq=self.event_seq, checks=checks, nextAction=self.next_action(checks))

    def zero(self):
        if rclpy.ok():
            self.velocity.publish(Twist())

    def lock(self, reason='', kind='protective'):
        """停车并锁定。

        kind='estop'   : 人工急停，向车端发 /emergency_stop=True（固件锁存）。
        kind='protective': 数据断流/节点失效/租约过期等保护性停车，只发布零速度，
                          依赖固件 300 ms 速度看门狗停车，不锁存硬件急停。
        """
        self.mode = 'locked'
        self.control_id = None
        # 人工急停是锁存态：后续保护停车不得把它降级为普通保护停车。
        if kind == 'estop' or self.lock_kind != 'estop':
            self.lock_kind = kind
        self.zero()
        if kind == 'estop' and rclpy.ok():
            self.estop.publish(Bool(data=True))
        if reason:
            self.error = reason
            self.last_stop = dict(at=time.time(), reason=reason, kind=self.lock_kind,
                                  runId=self.run_id, goalId=self.goal_id)

    # ---- 控制权归属 ----

    def session(self, request):
        session_id = request.get('sessionId')
        if not isinstance(session_id, str) or not session_id.strip() or len(session_id) > 128:
            raise ValueError('缺少控制会话标识，请刷新页面')
        return session_id.strip()

    def require_owner(self, request):
        """运动/目标类操作：仅控制页面可用；无所有者时把明确操作视为取得控制权。"""
        session_id = self.session(request)
        if self.owner is None:
            self.owner = session_id
            self.lease_at = time.monotonic()
            self.record('control_claimed', '无控制页面时由明确操作取得控制权',
                        sessionId=session_id, requestId=request.get('requestId'))
            return session_id
        if self.owner != session_id:
            self.record_throttled('request_denied', f'非控制页面尝试 {request.get("op")}',
                                  sessionId=session_id, op=request.get('op'))
            raise ValueError('控制权由其他页面持有，需先显式接管')
        self.lease_at = time.monotonic()
        return session_id

    def cancel(self, request=None, reason='', on_confirm=None, kind='protective'):
        self.points = []
        self.cancel_reason = reason
        self.lock(reason, kind)
        if request:
            self.cancel_waiters.append((request, time.monotonic() + 5, on_confirm))
        if self.goal_state in ('accepted', 'executing', 'canceling'):
            self.goal_state = 'canceling'
            self.cancel_deadline = time.monotonic() + 5
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
        else:
            self.finish_cancel()

    def finish_cancel(self):
        waiters, self.cancel_waiters = self.cancel_waiters, []
        for request, _, on_confirm in waiters:
            if on_confirm:
                on_confirm(request)
            else:
                self.respond(request, success=True)
        self.record('cancel_confirmed', self.cancel_reason)

    def on_auto(self, message):
        if self.mode == 'auto' and not automatic_velocity_valid(message):
            self.record('invalid_velocity', 'Nav2 速度数值、轴向或限幅无效')
            self.cancel(reason='自动速度无效，运动已锁定')
            return
        if (self.mode == 'auto' and self.goal_state == 'executing' and
                not self.requirements() and self.owner is not None and time.monotonic() - self.lease_at < LEASE_TTL_SECONDS):
            self.velocity.publish(message)

    def next_goal(self):
        if not self.points:
            self.goal_state = 'succeeded'
            self.mode = 'locked'
            self.zero()
            self.record('queue_completed', '全部目标完成')
            return
        self.current = self.points.pop(0)
        # 位移测量从派发目标这一刻开始，用于区分"没动"和"动了但失败"。
        self.stuck.begin_goal(time.monotonic(), self.odom_pose, self.current)
        self.goal_id = uuid.uuid4().hex
        token = self.goal_id
        self.goal_state = 'accepted'
        self.feedback = None
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.current['x'])
        goal.pose.pose.position.y = float(self.current['y'])
        goal.pose.pose.orientation.z = math.sin(self.current.get('yaw', 0.) / 2)
        goal.pose.pose.orientation.w = math.cos(self.current.get('yaw', 0.) / 2)
        self.record('goal_dispatched', f"目标 ({self.current['x']:.2f}, {self.current['y']:.2f})",
                    point=self.current, remaining=len(self.points))
        def feedback(msg):
            if token == self.goal_id:
                f = msg.feedback
                self.feedback = {'distance_remaining': f.distance_remaining,
                    'estimated_time_remaining': {'sec': f.estimated_time_remaining.sec,
                                                 'nanosec': f.estimated_time_remaining.nanosec}}
        future = self.action.send_goal_async(goal, feedback_callback=feedback)
        future.add_done_callback(lambda f: self.accepted(f, token))

    def accepted(self, future, token):
        if token != self.goal_id:
            return
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                raise RuntimeError('Nav2 拒绝目标')
            canceled = self.goal_state == 'canceling'
            self.goal_state = 'canceling' if canceled else 'executing'
            self.record('goal_accepted', '目标已受理，等待执行' if not canceled else '受理时已处于取消流程')
            if self.pending_goal_request:
                self.respond(self.pending_goal_request, success=not canceled, goalId=token,
                             error='目标已取消' if canceled else '')
                self.pending_goal_request = None
            self.goal_handle.get_result_async().add_done_callback(lambda f: self.completed(f, token))
            if canceled:
                self.goal_handle.cancel_goal_async()
        except Exception as error:
            self.failed(str(error))

    def handle_goal_failure(self, error, error_code=0):
        """D：区分"无位移卡死"与普通失败，避免把 20 个目标一个个喂进膨胀区。"""
        at = time.monotonic()
        classification = self.stuck.classify(at, self.odom_pose, error_code, error)
        decision = self.stuck.decide(at, classification)
        cleared = self.clear_costmaps_once() if decision['clearCostmap'] else False
        if decision['action'] == 'fail':
            # 有位移的失败：保持原有语义（记录 + 锁定 + 清队列）。
            self.failed(error)
            return
        reason = decision['reason']
        self.goal_state = 'failed'
        self.cancel_deadline = 0.
        self.points = []
        self.goal_handle = None
        # 单次无位移失败：保护性停车（等人工挪车/重设位姿）；
        # 连续到达上限：stuck 锁存，仅人工接管可以解除。
        self.lock(reason, kind='stuck' if decision['latched'] else 'protective')
        self.record('goal_failed_stuck' if decision['latched'] else 'goal_failed_no_progress',
                    reason, errorCode=error_code, consecutive=decision['consecutive'],
                    costmapCleared=cleared, classification=classification)
        if self.pending_goal_request:
            self.respond(self.pending_goal_request, success=False, error=reason,
                         noProgress=True, consecutive=decision['consecutive'])
            self.pending_goal_request = None
        self.finish_cancel()

    def clear_costmaps_once(self):
        """失败瞬间清一次 costmap；服务不可用如实记录，不重试、不假装成功。"""
        if not rclpy.ok():
            return False
        cleared, missing = [], []
        for name, service in (('local', '/local_costmap/clear_entirely_local_costmap'),
                              ('global', '/global_costmap/clear_entirely_global_costmap')):
            client = self.clear_costmap_clients.get(name)
            if client is None:
                try:
                    client = self.create_client(ClearEntireCostmap, service)
                except Exception:  # noqa: BLE001
                    missing.append(name)
                    continue
                self.clear_costmap_clients[name] = client
            if not client.service_is_ready():
                missing.append(name)
                continue
            try:
                client.call_async(ClearEntireCostmap.Request())
                cleared.append(name)
            except Exception:  # noqa: BLE001
                missing.append(name)
        self.record('costmap_cleared' if cleared else 'costmap_clear_skipped',
                    f"清 costmap：{','.join(cleared) or '无可用服务'}"
                    + (f"；不可用：{','.join(missing)}" if missing else ''),
                    cleared=cleared, missing=missing)
        return bool(cleared)

    def request_effective_params(self):
        """启动后实读运行中的参数值（D）。

        读不到就记 unverified —— 绝不用 yaml 里的期望值冒充"运行生效值"，
        否则"源码改了运行没生效"这个坑会被证据本身掩盖。
        """
        if self.effective_params is not None:
            return
        self.effective_params = {'readAt': time.time(), 'values': {}, 'missing': [],
                                 'unverified': [], 'servers': []}
        for node_name, names in EFFECTIVE_PARAM_TARGETS.items():
            try:
                client = self.create_client(GetParameters, f'{node_name}/get_parameters')
                ready = client.service_is_ready() or client.wait_for_service(timeout_sec=0.2)
            except Exception as error:  # noqa: BLE001
                self.effective_params['unverified'].append(
                    {'server': node_name, 'reason': f'参数客户端创建失败：{error}'})
                continue
            if not ready:
                self.effective_params['unverified'].append(
                    {'server': node_name, 'reason': '参数服务不可用'})
                continue
            try:
                request = GetParameters.Request()
                request.names = list(names)
                future = client.call_async(request)
                future.add_done_callback(
                    lambda done, server=node_name, keys=tuple(names):
                    self._store_effective_params(server, keys, done))
            except Exception as error:  # noqa: BLE001
                self.effective_params['unverified'].append(
                    {'server': node_name, 'reason': f'参数请求失败：{error}'})

    def _store_effective_params(self, server, names, future):
        if self.effective_params is None:
            return
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.effective_params['unverified'].append(
                {'server': server, 'reason': f'读取失败：{error}'})
            return
        if server not in self.effective_params['servers']:
            self.effective_params['servers'].append(server)
        for name, value in zip(names, response.values):
            key = f'{server}:{name}'
            resolved = param_value(value)
            if resolved is None:
                self.effective_params['missing'].append(key)
            else:
                self.effective_params['values'][key] = resolved
        self.record('effective_params', f'{server} 实读 {len(names)} 个参数',
                    server=server, values=self.effective_params['values'],
                    missing=self.effective_params['missing'],
                    unverified=self.effective_params['unverified'])
        if self.manifest_run_id and run_manifest is not None:
            try:
                run_manifest.update_manifest(self.manifest_run_id,
                                             {'effectiveParams': self.effective_params})
            except Exception as error:  # noqa: BLE001
                self.record('run_manifest_failed', f'生效参数写入清单失败: {error}',
                            runId=self.manifest_run_id)

    def failed(self, error):
        self.goal_state = 'failed'
        self.cancel_deadline = 0.
        self.points = []
        self.goal_handle = None
        self.lock(error)
        self.record('goal_failed', error)
        if self.pending_goal_request:
            self.respond(self.pending_goal_request, success=False, error=error)
            self.pending_goal_request = None
        self.finish_cancel()

    def completed(self, future, token):
        if token != self.goal_id:
            return
        try:
            result = future.result()
            canceling = self.goal_state == 'canceling'
            self.goal_handle = None
            if canceling or result.status == GoalStatus.STATUS_CANCELED:
                self.goal_state = 'canceled'
                self.cancel_deadline = 0.
                self.points = []
                self.lock(self.cancel_reason)
                self.record('goal_canceled', self.cancel_reason or '目标已取消')
                self.finish_cancel()
            elif result.status == GoalStatus.STATUS_SUCCEEDED and result.result.error_code == 0:
                self.stuck.on_success()
                self.record('goal_succeeded', '目标到达')
                self.next_goal()
            else:
                self.handle_goal_failure(
                    result.result.error_msg or f'导航失败，状态 {result.status}，错误码 {result.result.error_code}',
                    result.result.error_code)
        except Exception as error:
            self.failed(str(error))

    def process(self, request):
        op = request.get('op')
        if op == 'status':
            return self.respond(request, success=True, **self.status())
        if op == 'events':
            since = request.get('since')
            since = since if isinstance(since, int) and since >= 0 else 0
            # 返回 since 之后最早的 200 条，客户端按游标推进可拉全事件环。
            events = [event for event in self.events if event['seq'] > since][:200]
            return self.respond(request, success=True, events=events, latest=self.event_seq)
        if op == 'heartbeat':
            # 只有控制页面会话可以续期；任何其他页面的心跳一律拒绝（P1-3 核心）。
            session_id = self.session(request)
            if self.owner != session_id:
                self.record_throttled('heartbeat_denied', '非控制页面心跳，未续期', sessionId=session_id)
                raise ValueError('控制权不属于本页面，运动租约未续期')
            self.lease_at = time.monotonic()
        elif op == 'claim':
            session_id = self.session(request)
            if self.owner is not None and self.owner != session_id:
                self.record_throttled('request_denied', 'claim 被拒绝：控制权由其他页面持有', sessionId=session_id)
                raise ValueError('控制权由其他页面持有，请使用显式接管')
            self.owner = session_id
            self.lease_at = time.monotonic()
            self.record('control_claimed', '页面取得控制权', sessionId=session_id, requestId=request.get('requestId'))
        elif op == 'takeover':
            # 显式接管：先取消并确认停止，再生成新租约；旧租约立即失效。
            session_id = self.session(request)
            def grant(req, new_owner=session_id):
                self.owner = new_owner
                self.lease_at = time.monotonic()
                self.record('control_takeover', '控制权已接管，运动保持锁定直至明确启用',
                            sessionId=new_owner, requestId=request.get('requestId'))
                self.respond(req, success=True, owner=new_owner, mode=self.mode, controlId=self.control_id)
            self.cancel_reason = '控制权被接管'
            self.points = []
            self.lock(self.cancel_reason)
            if self.goal_state in ('accepted', 'executing', 'canceling'):
                self.goal_state = 'canceling'
                self.cancel_deadline = time.monotonic() + 5
                self.cancel_waiters.append((request, time.monotonic() + 5, grant))
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                self.record('cancel_requested', '接管前先取消当前运动', sessionId=session_id)
            else:
                grant(request)
            return
        elif op == 'release':
            session_id = self.session(request)
            if self.owner != session_id:
                raise ValueError('只有持有控制权的页面可以释放')
            self.owner = None
            self.record('control_released', '控制页面释放控制权', sessionId=session_id)
            if self.mode != 'locked':
                self.cancel(reason='控制页面已释放控制权，运动停止')
        elif op == 'begin':
            session_id = self.require_owner(request)
            self.run_id += 1
            for name, (future, _) in self.pending_states.items():
                if not future.done():
                    self.lifecycle_clients[name].remove_pending_request(future)
                    future.cancel()
            self.pending_states = {}
            if self.transition:
                name, future, _, _ = self.transition
                if not future.done():
                    self.transition_clients[name].remove_pending_request(future)
                    future.cancel()
            self.transition = None
            # 清理上一轮残留，避免旧反馈/旧目标串到新运行（P1-2）。
            stale, self.cancel_waiters = self.cancel_waiters, []
            for waiter, _, on_confirm in stale:
                if not on_confirm:
                    self.respond(waiter, success=False, error='导航已重启')
            self.startup_done = self.startup_failed = False
            self.running = True
            self.map_path = request['mapPath']
            # 新地图必须由本轮 map_server 重新发布；旧图不能用于目标预检。
            self.map_grid = None
            self.map_info = None
            self.map_received_at = 0.
            self.map_invalid_prev = False
            self.nav_started = time.monotonic()
            self.last_progress_at = self.nav_started
            self.startup_failure = None
            self.initial_pose_msg = None
            self.initial_pose_failed = None
            self.initial_pose_retries_left = 0
            self.pose_requested = float('inf')
            self.pose_requested_stamp_ns = 0
            self.localization_health.reset()
            self.runtime_profile.reset()
            self.stuck.reset()
            self.effective_params = None
            self.odom_pose = None
            self.localization_ok_prev = False
            self.localization_confirmed = False
            self.lifecycle = {}
            self.prev_node_states = {}
            self.lifecycle_polled_at = 0.
            self.error = ''
            self.goal_state = 'idle'
            self.goal_id = None
            self.goal_handle = None
            self.feedback = None
            self.current = None
            self.points = []
            self.pending_goal_request = None
            self.cancel_reason = ''
            self.cancel_deadline = 0.
            self.mode = 'locked'
            self.control_id = None
            self.velocity_sequence = -1
            self.record('startup_begin', f'导航启动：{self.map_path}', sessionId=session_id,
                        mapPath=self.map_path, requestId=request.get('requestId'))
            self.manifest_start(session_id)
        elif op in ('cancel', 'estop', 'stop'):
            # 安全类操作：任意页面可用，不校验归属。
            if op == 'stop':
                self.running = False
            reason = '已急停' if op == 'estop' else ''
            self.cancel(request, reason, kind='estop' if op == 'estop' else 'protective')
            self.record({'estop': 'estop', 'stop': 'navigation_stop', 'cancel': 'cancel_requested'}[op],
                        reason or '外部请求取消/停止', sessionId=request.get('sessionId'),
                        requestId=request.get('requestId'))
            if op in ('estop', 'stop'):
                self.manifest_finish('aborted', '已急停' if op == 'estop' else '用户停止导航')
            return
        elif op == 'manual_enable':
            session_id = self.require_owner(request)
            if self.goal_state in ('accepted', 'executing', 'canceling'):
                raise ValueError('请先取消导航并等待停止')
            if self.sensors():
                raise ValueError('缺少 ' + '、'.join(self.sensors()))
            self.velocity_sequence = -1
            self.control_id = uuid.uuid4().hex
            self.mode = 'manual'
            # 人工显式接管 = 已经挪车/重新确认现场，解除无位移锁存。
            self.stuck.manual_release()
            self.lease_at = time.monotonic()
            self.manual_at = time.monotonic()
            # 这是显式运动启用，允许作为人工急停后的复位路径；不能用 goals 隐式完成。
            # 传感器已恢复后，上一轮保护停车的 error 不应继续挂在页面上。
            self.estop.publish(Bool(data=False))
            self.lock_kind = 'idle'
            self.error = ''
            self.record('manual_enabled', '遥控已启用（新速度会话）', sessionId=session_id,
                        requestId=request.get('requestId'))
        elif op == 'reset_estop':
            # 人工急停的显式复位：只有控制页面可操作，且不能顺带启动运动。
            session_id = self.require_owner(request)
            if self.goal_state in ('accepted', 'executing', 'canceling'):
                raise ValueError('请先取消导航并等待停止')
            self.estop.publish(Bool(data=False))
            self.lock_kind = 'idle'
            self.error = ''
            self.record('estop_reset', '控制页面明确复位急停锁存；运动仍未启用',
                        sessionId=session_id, requestId=request.get('requestId'))
        elif op == 'velocity':
            session_id = self.session(request)
            if self.owner != session_id:
                self.record_throttled('request_denied', '非控制页面发送速度指令', sessionId=session_id)
                raise ValueError('控制权不属于本页面，速度指令被拒绝')
            if self.mode != 'manual' or request.get('controlId') != self.control_id:
                raise ValueError('遥控未启用（可能因保护停车已解除），请重新点“启用遥控”')
            if self.sensors():
                raise ValueError('车端数据未恢复：' + '、'.join(self.sensors()[:2]))
            sequence = request.get('sequence', self.velocity_sequence + 1)
            if not isinstance(sequence, int) or sequence <= self.velocity_sequence:
                raise ValueError('已丢弃过期速度指令')
            self.velocity_sequence = sequence
            x, z = float(request['linearX']), float(request['angularZ'])
            if not math.isfinite(x) or not math.isfinite(z):
                raise ValueError('速度必须为有限数值')
            msg = Twist()
            msg.linear.x = max(-.4, min(.4, x))
            msg.angular.z = max(-1.2, min(1.2, z))
            self.manual_at = time.monotonic()
            self.velocity.publish(msg)
        elif op == 'initial_pose':
            session_id = self.require_owner(request)
            if not self.running or self.goal_state in ('accepted', 'executing', 'canceling'):
                raise ValueError('请启动导航并取消当前目标后设置位置')
            if self.lifecycle.get('amcl', (0, 0))[0] != 3:
                raise ValueError('AMCL 正在启动，请稍后设置初始位置')
            self.nav_started = time.monotonic()
            self.last_progress_at = self.nav_started
            point = valid_points([request])[0]
            msg = self.send_initial_pose(point)
            requested_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.pose_requested = time.monotonic()
            self.pose_requested_stamp_ns = requested_ns
            self.localization_confirmed = False
            self.initial_pose_failed = None
            self.startup_failure = None
            self.localization_health.reset(requested_ns)
            # 首次已发送，剩余次数按重发间隔继续发（默认共 5 次、间隔 1s）。
            self.initial_pose_retries_left = self.initial_pose_retries - 1
            self.initial_pose_next_at = self.initial_pose_sent_at + self.initial_pose_interval
            self.error = ''
            self.record('localization_requested',
                        f"初始位置 ({point['x']:.2f}, {point['y']:.2f}, {math.degrees(point['yaw']):.0f}°)",
                        sessionId=session_id, requestId=request.get('requestId'), point=point)
        elif op == 'goals':
            session_id = self.require_owner(request)
            if not self.running:
                raise ValueError('导航尚未启动')
            if self.goal_state in ('accepted', 'executing', 'canceling'):
                raise ValueError('已有目标执行中，请先取消')
            points = valid_points(request.get('points'))
            self.validate_goals(points, request)
            if self.lock_kind == 'estop':
                reason = '人工急停锁存中：请先显式复位急停（或启用遥控）后再发送目标'
                self.record('goal_rejected', reason, sessionId=session_id,
                            requestId=request.get('requestId'))
                raise ValueError(reason)
            if self.stuck.latched:
                reason = ('无位移失败已锁存：' + (self.stuck.latch_reason or '车辆可能贴墙')
                          + '；请手动挪开车辆或重新设置初始位置，再点「启用遥控」解除')
                self.record('goal_rejected', reason, sessionId=session_id,
                            requestId=request.get('requestId'))
                raise ValueError(reason)
            missing = self.requirements()
            if missing:
                raise ValueError('导航尚未就绪：' + '、'.join(missing))
            self.points = points
            self.mode = 'auto'
            self.lease_at = time.monotonic()
            self.error = ''
            # 不在此处发 /emergency_stop=False：提交目标不得隐式清除人工急停锁存。
            self.pending_goal_request = request
            count = len(self.points)
            self.next_goal()
            self.record('goal_submitted', f'提交 {count} 个目标', sessionId=session_id,
                        requestId=request.get('requestId'), points=count)
            return
        elif op == 'shutdown':
            self.cancel(reason='服务已退出')
            self.record('bridge_shutdown', '控制服务退出，运动已锁定')
            self.shutdown_requested = True
            return
        else:
            raise ValueError('未知操作')
        self.respond(request, success=True, controlId=self.control_id, owner=self.owner)

    def manifest_start(self, session_id):
        """为本次导航启动写 manifest（best-effort；失败只记录事件，不阻断启动）。"""
        if run_manifest is None:
            return
        # 单元测试会直接实例化 Bridge 并调用 begin，不能让它往仓库里制造运行证据。
        if os.environ.get('PYTEST_CURRENT_TEST') or os.environ.get('ROBOT_RUN_MANIFEST', '1') == '0':
            return
        try:
            manifest = run_manifest.start_run('navigation', {
                'mapPath': self.map_path,
                'commandedBy': session_id,
                'nodeCount': len(NODES),
                'cmdVelAuthorities': '/cmd_vel 归 cmd_vel_gate 与 bridge(manual)',
                # 把本次生效的失败判定阈值写进证据，事后可复核"为什么判它卡死"。
                'stuckPolicy': self.stuck.snapshot()['options'],
            })
            self.manifest_run_id = manifest['runId']
            self.record('run_manifest', f'运行清单已建立: {self.manifest_run_id}',
                        sessionId=session_id, runId=self.manifest_run_id)
        except Exception as error:  # noqa: BLE001
            self.record('run_manifest_failed', f'运行清单写入失败: {error}', sessionId=session_id)

    def manifest_finish(self, status, reason):
        """结束本轮 manifest：写 summary，失败时保存 checks/localization 快照。"""
        run_id, self.manifest_run_id = self.manifest_run_id, None
        if run_manifest is None or not run_id:
            return
        try:
            checks = self.structured_checks()
            snapshots = {}
            if status != 'success':
                snapshots['checks-at-failure'] = checks
                snapshots['localization-at-failure'] = {
                    'localization': checks.get('localization'),
                    'mapPath': self.map_path,
                    'poseRequested': getattr(self, 'pose_requested', None),
                    'localizationConfirmed': bool(getattr(self, 'localization_confirmed', False)),
                    'error': self.error,
                    'lifecycle': dict(self.lifecycle),
                    'metrics': self.metrics.snapshot() if hasattr(self, 'metrics') else None,
                }
            run_manifest.finish_run(run_id, status, reason, snapshots=snapshots)
            self.record('run_manifest_finished', f'运行清单已结束: {status}',
                        runId=run_id, reason=reason)
        except Exception as error:  # noqa: BLE001
            self.record('run_manifest_failed', f'运行清单结束失败: {error}', runId=run_id)

    def initial_pose_stamp(self):
        """/initialpose 的 header.stamp：优先用最新 TF，其次 now-0.2s。

        直接写 now 会让 AMCL 认为变换在未来（future TF），发布后既不出位姿也不报错，
        页面只能等到 60 秒超时——这正是 2026-11-11 18:04:29 现场的原因。
        """
        # 只用整数纳秒比较：Time 对象带 clock_type，混用会抛
        # "Can't subtract times with different clock types"，把 TF 分支整体吞掉。
        now_ns = self.get_clock().now().nanoseconds
        newest_ns = None
        for parent, child in (('map', 'base_footprint'), ('map', 'odom'), ('odom', 'base_footprint')):
            try:
                if not self.buffer.can_transform(parent, child, rclpy.time.Time()):
                    continue
                transform = self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                stamp_ns = rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds
                age = (now_ns - stamp_ns) / 1e9
                # 过旧（>1s）的 TF 会被 AMCL 当成历史数据丢弃，宁可回退 now-0.2s。
                if -0.5 <= age <= 1.0 and (newest_ns is None or stamp_ns > newest_ns):
                    newest_ns = stamp_ns
            except Exception:  # noqa: BLE001 - TF 缺失/损坏时走回退分支
                continue
        chosen_ns = newest_ns if newest_ns is not None else now_ns - 200_000_000
        chosen_ns = min(chosen_ns, now_ns)
        return rclpy.time.Time(nanoseconds=chosen_ns).to_msg()

    def send_initial_pose(self, point):
        """发布一次 /initialpose，并保留消息用于按间隔重发（同一 stamp）。"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.initial_pose_stamp()
        msg.pose.pose.position.x = float(point['x'])
        msg.pose.pose.position.y = float(point['y'])
        msg.pose.pose.orientation.z = math.sin(point['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(point['yaw'] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25
        msg.pose.covariance[35] = .0685
        self.initial_pose_msg = msg
        self.initial_pose_sent_at = time.monotonic()
        self.pose.publish(msg)
        return msg

    def service_initial_pose(self):
        """初始位姿重发 + 超时判定（由 tick 每周期调用）。

        只有一个明确的失败态：initial_pose_timeout，带 code/detail/requestedStamp/
        lastPoseStamp，页面据此提示重新设点，而不是笼统的"导航启动超时"。
        """
        if not self.running or self.initial_pose_msg is None:
            return
        now = time.monotonic()
        if self.localization_confirmed:
            self.initial_pose_retries_left = 0
            return
        if self.initial_pose_retries_left > 0:
            if now < self.initial_pose_next_at:
                return
            self.pose.publish(self.initial_pose_msg)
            self.initial_pose_retries_left -= 1
            self.initial_pose_sent_at = now
            self.initial_pose_next_at = now + self.initial_pose_interval
            self.record('localization_resent',
                        f"重发初始位置（剩余 {self.initial_pose_retries_left} 次）",
                        requestedStamp=self.pose_requested_stamp_ns)
            return
        if self.initial_pose_failed is not None:
            return
        waited = now - self.initial_pose_sent_at
        if waited < self.initial_pose_timeout:
            return
        self.initial_pose_failed = {
            'code': 'initial_pose_timeout',
            'detail': f'初始位置发布后 {waited:.1f}s 内未收到新的 AMCL 位姿',
            'requestedStamp': self.pose_requested_stamp_ns or None,
            'lastPoseStamp': self.last_pose_stamp_ns or None,
            'elapsedSeconds': round(waited, 2),
            'retries': self.initial_pose_retries,
        }
        self.error = ('初始位置未被 AMCL 接受：请确认 AMCL 已激活、地图与激光数据一致，'
                      '然后重新设置初始位置')
        self.record('initial_pose_timeout', self.error, **self.initial_pose_failed)

    def start_localized_navigation(self):
        if not self.running or self.startup_done or self.startup_failed:
            return
        now = time.monotonic()
        if self.last_progress_at <= 0:
            self.last_progress_at = self.nav_started or now
        local_ready = all(self.lifecycle.get(name, (0, 0))[0] == 3 for name in NODES[:2])
        # 超时从"最后一次有效进展"起算，而不是从启动时刻：
        # 节点逐个激活、AMCL 出位姿都会刷新 last_progress_at。
        if now - self.last_progress_at > self.startup_timeout and (not local_ready or math.isfinite(self.pose_requested)):
            active = [name for name in NODES if self.lifecycle.get(name, (0, 0))[0] == 3]
            localization = self.localization_check()
            self.startup_failure = {
                'code': 'startup_timeout',
                'detail': (f'启动 {self.startup_timeout:.0f}s 无有效进展：'
                           f'{len(active)}/{len(NODES)} 节点激活，'
                           f'定位{"已确认" if self.localization_confirmed else "未确认"}'),
                'requestedStamp': self.pose_requested_stamp_ns or None,
                'lastPoseStamp': self.last_pose_stamp_ns or None,
                'localizationCode': localization.get('code'),
                'localizationDetail': localization.get('detail'),
                'activeNodes': active,
            }
            self.startup_failed = True
            self.error = f"导航启动超时：{self.startup_failure['detail']}（{localization.get('code')}）"
            self.lock(self.error + '；请停止导航后重试')
            self.record('startup_failed', self.error, **self.startup_failure)
            self.manifest_finish('timeout', self.error)
            return
        # Observe actual state after every transition. A lost RPC reply does not
        # hang startup or cause a duplicate transition based on stale state.
        if self.transition:
            name, future, expected, sent = self.transition
            reached = self.lifecycle.get(name, (0, 0))[0] == expected
            if not reached and now - sent < 3:
                return
            if not future.done():
                self.transition_clients[name].remove_pending_request(future)
                future.cancel()
            self.transition = None
        localization_ready = all(self.lifecycle.get(name, (0, 0))[0] == 3 for name in NODES[:2])
        localized = self.localization_check()['ok']
        candidates = NODES if localization_ready and localized else NODES[:2]
        for name in candidates:
            state, observed = self.lifecycle.get(name, (0, 0))
            if state == 3:
                continue
            if state not in (1, 2) or now - observed > 2 or not self.transition_clients[name].service_is_ready():
                return
            request = ChangeState.Request()
            request.transition.id = 1 if state == 1 else 3
            expected = 2 if state == 1 else 3
            self.transition = (name, self.transition_clients[name].call_async(request), expected, now)
            return
        if len(candidates) == len(NODES):
            self.startup_done = True
            self.last_progress_at = time.monotonic()
            self.record('startup_done', 'Nav2 全部节点已激活，等待定位或目标')
            self.request_effective_params()

    def tick(self):
        for _ in range(100):
            if self.shutdown_requested:
                break
            try:
                request = self.commands.get_nowait()
            except queue.Empty:
                break
            try:
                self.process(request)
            except Exception as error:
                self.respond(request, success=False, error=str(error))
        self.service_initial_pose()
        self.start_localized_navigation()
        now = time.monotonic()
        if now >= self._next_sample_at:
            self._next_sample_at = now + SAMPLE_INTERVAL_SECONDS
            self._write_sample()
        if self.running:
            self.runtime_profile.poll()
            amcl = self.runtime_profile.values.get('amcl', {})
            for parameter, attribute in [('update_min_d', 'update_distance'), ('update_min_a', 'update_angle')]:
                value = amcl.get(parameter)
                if isinstance(value, (int, float)) and math.isfinite(value) and value > 0:
                    setattr(self.localization_health, attribute, value)
            localization = self.localization_check()
            if localization['ok'] != self.localization_ok_prev:
                self.record('localization_recovered' if localization['ok'] else 'localization_invalid',
                            localization['detail'], code=localization['code'])
                self.localization_ok_prev = localization['ok']
        # 传感器状态变化事件（断流/恢复各记录一次）。
        missing_sensors = self.sensors()
        if missing_sensors and self.sensors_ok_prev:
            self.sensors_ok_prev = False
            self.record('sensor_dropout', '、'.join(missing_sensors) + ' 断流')
        elif not missing_sensors and not self.sensors_ok_prev:
            self.sensors_ok_prev = True
            self.record('sensor_recovered', '车端数据已恢复；运动保持锁定，需明确启用')
        # 控制页面心跳失联：释放租约并锁定运动；恢复后不自动续跑（P1-3/P1-4）。
        if self.owner is not None and now - self.lease_at > LEASE_TTL_SECONDS:
            self.owner = None
            self.record('lease_expired', '控制页面心跳失联，控制权已释放，运动停止并保持锁定')
            if self.mode != 'locked':
                self.cancel(reason='控制页面失联，运动已停止；重新控制需明确启用')
        elif self.mode != 'locked' and missing_sensors:
            self.cancel(reason='车端数据中断，已停止；请重新启用')
        if self.mode == 'auto' and self.requirements():
            reason = '导航节点或定位失效：' + '、'.join(self.requirements()[:4])
            self.record('navigation_fault', reason)
            self.cancel(reason=reason)
        if self.mode == 'manual' and now - self.manual_at > .3:
            self.zero()
        if self.mode == 'locked':
            self.zero()
        expired = [entry for entry in self.cancel_waiters if now > entry[1]]
        for entry in expired:
            request = entry[0]
            self.respond(request, success=False, error='取消尚未确认，运动已锁定')
            self.record('cancel_timeout', '取消请求 5 秒未确认终态，运动保持锁定',
                        sessionId=request.get('sessionId'))
            self.cancel_waiters.remove(entry)
        # 目标受理 future 不返回时，不能把 goal_state 永久卡在 canceling；
        # 5 秒后进入本地 canceled 终态，并让成功受理的晚到回调失效。
        if (self.goal_state == 'canceling' and self.cancel_deadline and
                now >= self.cancel_deadline):
            late_goal = self.pending_goal_request
            if late_goal:
                self.respond(late_goal, success=False, error='目标受理超时，已取消')
                self.pending_goal_request = None
            self.record('cancel_timeout_terminal', '取消未确认终态，本地进入 canceled',
                        goalId=self.goal_id)
            self.goal_state = 'canceled'
            self.cancel_deadline = 0.
            self.goal_id = None
            self.goal_handle = None
            self.current = None


def valid_points(points):
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ValueError('请提供 1 到 100 个目标')
    result = []
    for point in points:
        values = {key: float(point.get(key, 0) if key == 'yaw' else point[key]) for key in ('x', 'y', 'yaw')}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('目标坐标必须为有限数值')
        result.append(values)
    return result


def main():
    rclpy.init()
    node = Bridge()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.lock()
        node.manifest_finish('aborted', '导航桥接进程退出')
        node._append_log('bridge-samples', {'kind': 'session-end', 'at': time.time(),
                                           'mode': node.mode, 'lockKind': node.lock_kind,
                                           'lastStop': node.last_stop})
        node.close_logs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
