# agent_ws — micro-ROS Agent 工作空间

主机侧的 micro-ROS Agent，把底盘固件（ESP32-S3）的 micro-ROS 会话桥接到 ROS 2。

## 构成

- `src/micro-ROS-Agent`：上游 Agent 包。
- `src/micro_ros_msgs`：上游消息包。
- `lyrical-sources.json`：上游源码的仓库、commit、sha256 与归档名，用于可复现地重建。
- `lyrical-src/`：按 `lyrical-sources.json` 固定版本展开的上游源码（不提交构建产物）。

## 构建与运行

```bash
cd agent_ws
colcon build --symlink-install
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888   # 或 serial --dev /dev/ttyUSB0 -b 921600
```

底盘固件中的 Agent 地址与端口通过配网页设置；传感器数据在 ROS 2 侧的接口见
`robot_ws/src/leap_interfaces`。
