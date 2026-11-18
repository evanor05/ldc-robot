# Ubuntu 26.04 / ROS 2 Lyrical 本机移植

本机软件构建和模拟通信验证已通过；实车迁移仍在进行。当前分支为 `migration/lyrical-native`。原 Humble Agent 和固件源码保留用于对照，Lyrical Agent 使用另行固定的上游源码。

## 本次已实现

- 主工作空间 12 个功能包（`ldc_*` 及上游 fork）使用 Lyrical 导出 CMake targets、C++20 和新的头文件/QoS 接口构建；旧 `xuegeros_demo` 已存档至顶层 `legacy/`，不参与构建。
- 恢复镜像误排除的 GMapping、XRCE 客户端 `log` 源码，并修正镜像生成器和 Git 排除规则。上游版本、匹配依据及文件哈希见 `migration-tools/source-recovery.json`。
- TF 按里程计测量时间发布，不再以 1000 Hz 重发旧姿态；拒绝无效姿态和未知坐标系。`base_link` 输入按 URDF 固定变换换算为 `base_footprint`，高度参数默认 `0.06205 m`。固件里程计的高度语义仍需实车确认。
- GMapping 实际读取 YAML 参数，保留用户指定的量程，地图使用 transient-local QoS，迟连接的界面或地图保存器也能收到地图；修复小数 TF 延迟被截断的问题。
- Nav2 使用项目自己的组件启动文件，无需 `nav2_bringup` 的 Gazebo 依赖。更新插件标识、参数接口，控制器/行为/平滑器统一使用 `geometry_msgs/msg/Twist`；里程计订阅兼容 best-effort 与 reliable 两种发布端。
- 网页支持固件 `/battery_state` 的 BatteryState CDR，电压按 V、百分比按 0–1 解释；未知电量显示未知。保留旧 UInt16 的显式话题配置能力。
- 网页和 Qt 移除旧主机工作空间路径；Qt 支持 Ctrl+C 退出；舵机界面打开时不自动发送回正命令。

## 环境与重建

本机已安装依赖。后续检查依赖计划：

```bash
cd /path/to/ros-car-migration
./scripts/install_runtime_deps.sh --simulate
```

新机器需要先按 ROS 官方方法配置 Ubuntu resolute 的 ROS 2 APT 源，再执行 `./scripts/install_runtime_deps.sh`。本脚本不安装 Node.js；网页当前使用本机 Node 22，依赖由 package-lock.json 固定。

```bash
./scripts/build_host.sh
./scripts/build_agent.sh
./scripts/build_legacy_test_client.sh  # 仅软件验收需要，不是 ESP32 固件编译
./scripts/prepare_runtime.py
cd web_console
npm ci --ignore-scripts
npm run build
cd ..
source scripts/env.bash
./scripts/check_host.sh
```

构建脚本显式使用 `/usr/bin/python3`，避免 CMake 选中本机 uv 的 Python 3.12；Lyrical Python 模块对应系统 Python 3.14。使用新 shell，避免叠加其他 ROS 版本。`env.bash` 加载当前仓库的 Agent 私有库和两个工作空间，不修改 `.bashrc`。

**不要在仓库根目录直接运行无范围的 colcon/rosdep，也不要对保留的 Humble Agent 执行默认构建。** 固件包含大量 vendored ROS 包；Agent 脚本只构建 `agent_ws/lyrical-src`，主机脚本只构建 `robot_ws/src`。

Agent 依赖锁定在 `agent_ws/lyrical-sources.json`，下载时检查 SHA256。使用 Lyrical Agent、XRCE Agent 3.0.2、spdlog 1.9.2 内置 fmt，全部安装在仓库内，不覆盖系统库。已生成的构建结果包含本机绝对路径；移动仓库后应重建。

## 本机启动入口

以下实车命令留待连接信息确认后使用。每个终端均先运行：

```bash
cd /path/to/ros-car-migration
source scripts/env.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

micro-ROS 模式：车端 Agent IP 必须指向本机实际可达的局域网地址，端口与车端配置一致，默认 UDP 8888。不能仅凭仓库默认值认定实车模式或 IP。

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 4
```

UART MAVLink 模式另选，不能与 micro-ROS 底盘模式同时启动。仓库固件 UART 为 230400，主机默认配置已对齐；实际烧录版本可能不同，连接时显式指定：

```bash
ros2 run ldc_mavlink_bridge mavlink_bridge --ros-args \
  -p port:=/dev/serial/by-id/实际设备 -p baudrate:=230400
```

现有 MAVLink 主机桥仅支持串口。固件 MAVLink UDP 14550 模式尚无主机端适配，不能把它当作上述串口桥的网络模式。

模型与里程计 TF：

```bash
ros2 launch ldc_bringup ldc_bringup.launch.py
```

网页或 Qt 界面任选。网页启动脚本只自动启动 Foxglove，建图/导航由页面操作触发：

```bash
./web_console/start_console.sh  # 浏览器 http://localhost:5173
# 或
ros2 run ldc_qt_gui ldc_qt_gui
```

网页地图默认存放本仓库 `maps/`，首次准备时从基线复制，不覆盖已有地图；该目录不提交 Git。自定义路径可设置绝对路径环境变量 `ROBOT_MAPS_DIR`。

摄像头默认通过固件 UDP 33333 自动发现（`ldc_camera` 与网页后端都实现了该协议），无需写死 IP；也可在 `web_console/ros_config.json` 的 `camera.stream_url` 或相机节点的 `url` 参数显式指定。手动启动图传：

```bash
ros2 launch ldc_camera http_video_publisher.launch.py
```

摄像头只用于远程图传/录制，不参与 GMapping、AMCL 或 Nav2 控制闭环；相机掉线不应影响导航。

建图和定位导航分别运行，不同时启动两个 `map -> odom` 发布者：

```bash
ros2 launch slam_gmapping slam_gmapping.launch.py
# 停止建图后，再使用已保存地图定位导航：
ros2 launch ldc_navigation navigation2.launch.py map:=/绝对路径/地图.yaml
```

默认输出是桥接输入 `/cmd_vel_auto`；没有网页控制台/桥接时无人转发，车辆不会运动。仅台架直连测试才显式追加 `cmd_vel_output:=cmd_vel`（会绕过控制权/急停门控）。

不要执行旧 `legacy/xuegeros_demo/launch/bringup_all.launch.py`：其中残留旧 sudo/socat/YDLIDAR 流程。

## 验证边界与下一步

`./scripts/check_host.sh` 在 ROS domain 197、LOCALHOST 发现范围内验证真实可执行程序；测试不连接车辆，使用专用话题或合成的 odom/scan/TF。

| 检查 | 已验证的范围 |
| --- | --- |
| 主机工作空间 | 12/12 包构建，自定义 PID 服务及相机模块可导入 |
| Lyrical Agent | 2/2 包构建，仓库 XRCE 2.4.2 在 amd64 上经 loopback 双向传输 ROS String/Twist，并删除会话 |
| MAVLink | 虚拟串口速度编码、CRC、零速度、ATTITUDE 到 ROS IMU |
| TF | 无初始虚假 TF、姿态校验/归一化、坐标换算、保留原时间、无旧数据重发 |
| GMapping | 合成激光产生地图、YAML 参数生效、迟连接订阅者收到地图 |
| Nav2 | 9 个生命周期节点 active，加载实际插件并生成路径，Twist 类型和 odom QoS 匹配 |
| 网页 | Vite 构建，真实 Foxglove 电池 CDR 接收与 JSON Twist 发送，地图列表 API |
| Qt | 两个界面 offscreen 启动及 SIGINT 退出；屏幕交互/GPU 未验收 |

下一步需要实车开机后的通信模式、IP/端口或 USB 设备信息，然后依次验证：

1. 车端运行版本、通信模式和 NVS 配置；先读取和记录，不直接擦除或烧录。
2. 真正的 odom/IMU/scan/BatteryState 频率、时间同步、单位、方向、坐标系和 Get/SetSpeedPid 服务。
3. 停止保护：控制消息中断、网页关闭、Agent 退出、Wi-Fi 断开时车辆能否自动归零。固件运动任务目前未发现明确的速度指令超时；主机侧停止不能保证断网后的停止，实车运动前必须补齐并验收。
4. 架空车轮确认速度方向和零速度，再验证限速运动、遥控与导航控制权切换。
5. 实际相机视频、建图质量、AMCL 定位、导航目标/取消、动态避障和急停。

Cartographer 外部运行依赖、Lichtblick 独立应用和 ESP32 固件工具链构建尚未完成本机验收；本次可用建图路径是 GMapping。备份/Release 完整性及 Flash/NVS 保全仍按原迁移状态分别跟踪，软件测试通过不代表整车迁移完成。

上游参考：[Lyrical Agent 构建依赖](https://github.com/micro-ROS/micro-ROS-Agent/blob/8cf77415e64a9a95ace719f14a020b7a0745e19e/micro_ros_agent/cmake/SuperBuild.cmake)、[Nav2 Lyrical](https://github.com/ros-navigation/navigation2/tree/lyrical)。本机验证日志位于仓库外 `../ros-car-migration-audit/native-runtime/`。
