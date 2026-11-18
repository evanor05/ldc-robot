#!/usr/bin/env python3
"""D 工单合成底盘测量入口（仅新增测量代码，不改产品文件）。

offline_robot.py 的 main() 硬编码守卫 ROBOT_SIMULATION=1 且 ROS_DOMAIN_ID=197
（offline_robot.py:100-101，共享验收环境入口策略），私有域 194 直起会立即退出。
本入口复用 offline_robot.Robot 类本身（类内无域检查，行为与产品完全一致），
在 COORDINATION.md §2 分配给 D 的私有域 194 运行，不占用 Domain197 共享环境。

用法：ROS_DOMAIN_ID=194 ROBOT_SIMULATION=1 python3 scripts/latency_robot.py
"""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy  # noqa: E402
from offline_robot import Robot  # noqa: E402


def main():
    rclpy.init()
    node = Robot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()