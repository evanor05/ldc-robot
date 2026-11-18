# ldc-robot — LDC 自主导航小车

LDC 是一台以 ESP32-S3 为底控、ROS 2 为主机端的差速两轮小车：底盘固件跑 micro-ROS/MAVLink，
主机端用 ROS 2 做建图、导航与视觉，网页中控台负责遥控、建图与导航的人机交互。

本仓库是小车全车代码的公开镜像：**主机端 ROS 2 工作空间 + micro-ROS Agent + 网页中控台 +
底盘/图传固件 + 迁移基线存档**，已做脱敏与构建产物清理（见文末「公开前的处理」）。

> **开源发布清单**：[README.md](README.md) · [docs/固件改动说明.md](docs/固件改动说明.md) · [docs/迁移基线README.md](docs/迁移基线README.md) · [docs/迁移过程说明.md](docs/迁移过程说明.md)

> 状态：主机端 12 个包可构建，Agent、模拟建图/导航与界面协议已本机验证；实车验收仍在进行。
> 具体边界见 [NATIVE-LYRICAL.md](NATIVE-LYRICAL.md) 与 [docs/migration/migration-status.json](docs/migration/migration-status.json)。

## 架构

```
              ┌──────────────────────────── 主机（Ubuntu + ROS 2）───────────────────────────┐
 网页中控台    │  web_console（Vue 3 + Node/Express）                                        │
 (浏览器)  ⇄   │        │ REST / WebSocket                                                 │
              │        ▼                                                                   │
              │  scripts/ 桥接节点（控制权仲裁、目标 UUID 账本、诊断与自检）                  │
              │        │  ROS 2 话题 / 动作                                                      │
              │  robot_ws：ldc_bringup · ldc_navigation(Nav2) · gmapping/cartographer 建图     │
              │            · ldc_vision · ldc_camera · ldc_description · leap_interfaces …     │
              │        │  micro-ROS 会话（UDP 8888 或 UART 921600）                            │
              │  agent_ws：micro-ROS Agent                                                  │
              └────────┼───────────────────────────────────────────────────────────────────┘
                       ▼
              ┌────── ESP32-S3 ──────┐        ┌──── ESP32-S3 图传 ────┐
              │ firmware/chassis     │        │ firmware/camera_stream │
              │ 电机/里程计/IMU/雷达  │        │ MJPEG 视频流           │
              │ 超声波/电池/状态灯    │        └───────────────────────┘
              └──────────────────────┘
```

## 目录

| 目录 | 内容 |
| --- | --- |
| `robot_ws/` | 主机端 ROS 2 工作空间（`ldc_*` 功能包 + 上游 fork：gmapping、Camsense 雷达、`leap_interfaces` 接口契约） |
| `agent_ws/` | micro-ROS Agent 工作空间（源码版本锁定于 `lyrical-sources.json`） |
| `web_console/` | 网页中控台（Vue 3 前端 + Node 后端） |
| `firmware/chassis/` | ESP32-S3 底盘固件（运动/里程计/IMU/雷达/超声波/电池/网络，ESP-IDF + micro-ROS） |
| `firmware/camera_stream/` | ESP32-S3 图传固件（MJPEG） |
| `scripts/` | 构建、检查、仿真、桥接与运维入口 |
| `maps/` | 建图产物样例（`.pgm` + `.yaml`） |
| `docs/` | 架构、启动、标定、验收与历次修复记录 |
| `xuegeros_ws/`、`leap_demo/`、`Ros_Vue_x86/`、`lichtblick/`、`legacy/` | 迁移基线归档与上游第三方源码，仅供对照，不参与构建（`leap_demo/` 是底盘固件早期快照，现行固件看 `firmware/chassis/`） |
| `docs/migration/`、`migration/`、`migration-tools/` | 迁移审计档案、依赖清点与校验脚本（见 [docs/迁移过程说明.md](docs/迁移过程说明.md)） |

## 快速开始

### 1. 主机端（ROS 2）

```bash
# 依赖：ROS 2（本机基线为 Lyrical，见 NATIVE-LYRICAL.md）
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

一键启动与模块入口见 [docs/LDC一键启动与模块入口.md](docs/LDC一键启动与模块入口.md)。

### 2. micro-ROS Agent

```bash
cd agent_ws
colcon build --symlink-install && source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 3. 网页中控台

```bash
cd web_console
npm ci
bash start_console.sh        # 前后端一起起；浏览器访问 http://localhost:5173
```

测试：`npm test`（单元）、`npm run test:integration`（后端接口集成）、`npm run test:ros`（离线联调）。

### 4. 固件（ESP-IDF）

```bash
cd firmware/chassis
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

- 首次构建会依据 `managed_components/micro_ros_espidf_component/micro_ros_src/` 源码编译 micro-ROS，
  耗时较长；预编译库与构建产物不入库。
- 出厂默认 Wi-Fi 凭据为空：设备进入配网模式后连接其热点，访问 `http://192.168.4.1`
  填写 SSID/口令与 micro-ROS Agent IP，或事先修改 `main/config/network_config.h`。
- 相对上游的固件改动与已知限制见 [docs/固件改动说明.md](docs/固件改动说明.md)。

## 文档索引

- 架构与代码阅读入口：[docs/LDC仓库架构.md](docs/LDC仓库架构.md)
- 使用与演进计划：[docs/LDC使用与演进计划.md](docs/LDC使用与演进计划.md)
- 标定与精度验收：[docs/LDC标定与精度验收.md](docs/LDC标定与精度验收.md)
- 视觉接入与使用：[docs/LDC视觉接入与使用.md](docs/LDC视觉接入与使用.md)
- 运行日志诊断与故障回放：[docs/LDC诊断标定与故障回放.md](docs/LDC诊断标定与故障回放.md)
- 运动安全（急停授权与来源仲裁）：[docs/LDC-MCU统一运动安全修复.md](docs/LDC-MCU统一运动安全修复.md)
- 迁移基线与恢复：[docs/迁移过程说明.md](docs/迁移过程说明.md)、[docs/迁移基线README.md](docs/迁移基线README.md)

## 公开前的处理

- 配置中的敏感值（Wi-Fi 口令、令牌等）在迁移时已替换为占位符；固件默认 STA 凭据置空。
- 删除所有构建产物、依赖缓存、运行时地图/日志与预编译库（`libmicroros.a`、micro-ROS 340 MB 构建产物等）。
- 迁移基线中的密钥与加密备份**不在本仓库**，需单独安全传递。

## 许可证

- 本仓库自有代码：见各目录说明；引用与再分发前请确认。
- 第三方内容（Lichtblick MPL-2.0、micro-ROS、gmapping BSD 等）保留其原始 LICENSE，以上游条款为准。

## 致谢

底盘固件上游为 LEAP_ROS（`czu963889306-dev/leap_ros_mcu_driver`）；视觉与调试工具链参考
Lichtblick 与 ROS 2 社区项目。
