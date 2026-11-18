# robot_ws — 主机端 ROS 2 工作空间

ROS 2 主机端工作空间，包含 LDC 的功能包与上游 fork。

## 功能包

| 包 | 用途 |
| --- | --- |
| `ldc_bringup` | 总启动：拉起底盘通信、传感器、TF 等基础节点 |
| `ldc_navigation` | Nav2 配置与自定义规划器插件、导航行为 |
| `ldc_cartographer` / `slam_gmapping` / `openslam_gmapping` | 建图（Cartographer 与 GMapping 两套） |
| `ldc_exploration` | 探索式建图（前沿探索） |
| `ldc_vision` | 视觉（摄像头接入、目标检测节点） |
| `ldc_camera` | 相机数据源（HTTP/MJPEG）发布 |
| `ldc_description` | URDF 与网格模型 |
| `ldc_mavlink_bridge` | MAVLink 桥接 |
| `ldc_qt_gui` / `ldc_servo_gui` | 本地图形界面与舵机调试界面 |
| `leap_interfaces` | 线上接口契约（底盘固件与主机端共用的消息/服务定义） |
| `camsense_lidar` | Camsense 激光雷达驱动 |

## 构建

```bash
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

ROS 2 版本以仓库根目录 `NATIVE-LYRICAL.md` 记录的基线为准（Lyrical，Humble 迁移源码存档在 `legacy/` 与 `xuegeros_ws/`）。

## 说明

- 本目录同时保留了迁移基线时期的原始副本（`xuegeros_ws/`、`leap_demo/`、`lichtblick/`），
  仅供对照，不参与构建。
- 上游 fork 保留其原始 LICENSE。
