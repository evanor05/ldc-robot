#!/usr/bin/env python3
"""雷达数据规范化（主机侧补丁）：把固件下发的无效读数转成 inf。

背景（2026-11-18 实车实测）：
- /scan 每帧固定 360 点，其中 23 点恒为 0.0，而 range_min=0.02。
  0.0 是「无效/无回波」的占位值，不是「障碍在 0 米」。
- 未规范化的后果：
  * AMCL 把 0.0 当成贴脸障碍，似然场被污染，定位协方差虚高（实测 0.16）；
  * costmap 的 obstacle_layer 在车体原点周围标出虚假障碍，控制器无路可走；
  * 表现为「导航不动 / 规划失败 / 定位漂移」。
- 固件在 MCU 侧，本机无法修改，因此在此做主机侧规范化。

同时修正几何自洽性：angle_min=0、angle_max=6.2657、increment=0.017453 时
理论点数 = (max-min)/inc + 1 = 359，但消息带 360 点。按 360 点解释会让最后一束
角度超出 angle_max 一整个增量，AMCL 的束采样与 costmap 投影都会系统性偏一圈。
本节点按实际点数重写 angle_max，保证「点数 = 角度范围 / 增量 + 1」。

用法：
    ros2 run ldc_navigation lidar_sanitizer --ros-args \
        -r scan:=/scan_raw -r scan_out:=/scan
（把固件原始话题重映射成 scan_raw，节点输出标准的 /scan）
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class LidarSanitizer(Node):
    def __init__(self):
        super().__init__('lidar_sanitizer')
        # 默认：监听固件原始 /scan，输出 /scan_clean 给 AMCL 与 costmap 消费。
        # 这样不需要改固件话题名，也不会形成自环。
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_clean')
        self.declare_parameter('fix_angle_bounds', True)
        self.declare_parameter('drop_zero_readings', True)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.fix_angle_bounds = bool(self.get_parameter('fix_angle_bounds').value)
        self.drop_zero_readings = bool(self.get_parameter('drop_zero_readings').value)
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(LaserScan, output_topic, qos)
        self.create_subscription(LaserScan, input_topic, self.on_scan, qos)
        # 统计：便于现场判断「规范化是否真的在起作用」。
        self.frames = 0
        self.zero_fixed = 0
        self.bounds_fixed = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'雷达规范化：{input_topic} → {output_topic}')

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        ranges = list(msg.ranges)
        intensities = list(msg.intensities)

        if self.drop_zero_readings:
            # 0.0 与非有限值 → inf（ROS 约定：无回波用 inf）。
            # 注意：只处理「严格等于 0」与 NaN，真实近距离读数（>= range_min）保留。
            fixed = 0
            for index, value in enumerate(ranges):
                if value == 0.0 or not math.isfinite(value):
                    ranges[index] = float('inf')
                    fixed += 1
            self.zero_fixed += fixed

        if self.fix_angle_bounds and len(ranges) > 1 and out.angle_increment > 0:
            expected = int(round((out.angle_max - out.angle_min) / out.angle_increment)) + 1
            if expected != len(ranges):
                # 保持 angle_min/increment 不变，按实际点数重写 angle_max。
                out.angle_max = out.angle_min + (len(ranges) - 1) * out.angle_increment
                self.bounds_fixed += 1
                if self.bounds_fixed == 1:
                    self.get_logger().warning(
                        f'角度范围与点数不自洽（{expected} vs {len(ranges)}）；'
                        f'已按点数重写 angle_max={out.angle_max:.6f}')

        out.ranges = ranges
        out.intensities = intensities
        self.publisher.publish(out)
        self.frames += 1

    def report(self):
        if self.frames:
            self.get_logger().info(
                f'已规范化 {self.frames} 帧，修正零值 {self.zero_fixed} 个，'
                f'角度重写 {self.bounds_fixed} 帧')


def main(args=None):
    rclpy.init(args=args)
    node = LidarSanitizer()
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