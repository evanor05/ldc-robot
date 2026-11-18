# LDC 一键启动与模块入口

更新：2026-11-08。工程目录 `/home/<user>/ros-car-migration`。本批新增统一启动/清理入口，网页默认观察 YOLO 叠框图；标定操作见 [LDC 标定与精度验收](LDC标定与精度验收.md)。

本机依赖与构建已经验证。源码更新后可先运行 `bash scripts/prepare_vision.sh` 和 `bash scripts/build_host.sh --packages-select ldc_camera ldc_vision ldc_qt_gui`；启动入口本身不安装软件或自动下载权重。

## 最常用的启动方式

```bash
cd /home/<user>/ros-car-migration
# 实车：默认 micro-ROS UDP 8888；需要设备的通信模式与之匹配
bash scripts/start_all.sh --real
# 手动指定实际 MJPEG 地址，跳过相机自动发现
bash scripts/start_all.sh --real --camera-url 'http://<CAMERA_IP>:81/'
# 桌面终端：同时打开 Qt 观察窗口和 RViz 雷达/TF 观察窗口
bash scripts/start_all.sh --real --qt --rviz
```

网页地址是 **http://localhost:5173/**。API 默认 3000，Foxglove 默认 8765，视觉网关只绑定本机 18320。启动主机节点不代表实车已连接，页面以雷达、里程计和摄像头的新数据判定健康。

默认启动 Agent、底盘 TF/描述、Foxglove、Node、Vue 和摄像头/YOLO。**不会自动解除急停、启用遥控、开始建图、发送导航目标。** Qt 默认为只读观察；启动时使用 `/vision/annotated/compressed` 观察叠框图。建图、初始位置和导航目标仍由网页统一管理。

若车端使用 MAVLink，选择 `--transport mavlink`，不能与 micro-ROS Agent 叠加。该分支使用既有 `ldc_mavlink_bridge` 配置，不在本批修改串口或网络参数。已有外部通信节点时可用 `--transport none`；已有外部底盘 TF 时加 `--no-chassis`。未确定实机通信模式时先检查车端设置，不凭历史默认认定已连接。

已有控制台占用端口时，本入口会拒绝启动，不杀旧进程。先在原终端关闭，或选择另一组明确的端口；不要叠加多套同 Domain 的 Agent/TF/相机驱动。

## 不接实车的完整演示

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_all.sh --sim
# 可换成自己的 JPEG 测试图
bash scripts/start_all.sh --sim --image /ABS/PATH/image.jpg
```

仍打开 **http://localhost:5173/**；视觉卡片标明“仿真”。使用 Domain 197、localhost-only，API 18310、Foxglove 8766、视觉网关 18321、模拟相机 18322。底盘/雷达由隔离模拟节点提供，图像来自固定测试图，GPU 执行真实 YOLO 推理；这不是实机画面。地图写入 `/tmp/ldc-sim-*/maps/`，不进入实车地图目录。

默认样例复用本机已有 `/home/<user>/ros-car-migration-audit/yolo-integration/bus.jpg`；其他机器请传 `--image`。此入口不下载图片/模型、不安装依赖、不构建或刷写固件。

## 看状态、停止和日志

```bash
bash scripts/start_all.sh --real --status
bash scripts/start_all.sh --sim --status
# 在启动终端 Ctrl+C，或另一个终端执行同模式停止
bash scripts/stop_all.sh --real
bash scripts/stop_all.sh --sim
# 只查看命令计划，不启动节点或写入运行目录
bash scripts/start_all.sh --real --dry-run
```

重复启动同一模式返回当前实例状态。状态包含实际 PID、运行阶段和本次日志目录；停止只清理本次进程树，不接管外部会话。使用 Linux 子进程回收机制处理 Node/launch 异常退出后脱离原进程组的 ROS/YOLO worker；后台关键服务退出时总入口结束并清理。Qt/RViz 关闭不会自动停止其他模块。

日志分别位于启动时打印的 `/tmp/ldc-real-*/` 或 `/tmp/ldc-sim-*/`。查看 `backend.log`、`foxglove.log`、`chassis.log` 等对应文件；视觉 ROS/worker 日志由 Node 记录在 `backend.log`。Ctrl+C 是退出，不替代网页的“保存并停止建图”；需要保留地图时先在网页保存。

## 每个模块的独立 sh

以下命令都从 `/home/<user>/ros-car-migration` 执行。默认 `--real`，除旧 RViz/Qt/vision 入口外也可显式指定；建议始终写出模式。

| 模块 | 命令 | 说明 |
| --- | --- | --- |
| micro-ROS Agent | `bash scripts/start_agent.sh --real` | UDP 8888；不用于 MAVLink 模式 |
| MAVLink 桥 | `bash scripts/start_mavlink.sh --real` | 与 Agent 二选一，复用原配置 |
| 车体与 TF | `bash scripts/start_chassis.sh --real` | 描述、关节状态与 odom TF，未启用 EKF |
| 外接串口雷达 | `bash scripts/start_lidar.sh --real` | **仅主机直连雷达用**；MCU 已发布 `/scan` 时不启动 |
| 旧网页完整栈 | `bash scripts/start_web.sh --real` | 原 Node + Vue + Foxglove 入口；不启动 Agent/底盘/视觉 |
| 仅 Node | `bash scripts/start_backend.sh --real` | 默认 API 3000 |
| 仅 Vue | `bash scripts/start_frontend.sh --real` | 默认网页 5173 |
| 仅 Foxglove | `bash scripts/start_foxglove.sh --real` | 手动拆分网页栈时使用；不要与旧网页入口的桥重复 |
| 摄像头 + YOLO | `bash scripts/start_vision.sh --real` | 推荐的统一视觉入口，自动发现或 `url:=...` |
| 仅摄像头 | `bash scripts/start_camera.sh --real 'url:=http://<CAMERA_IP>:81/'` | 发布原图与 CameraInfo，不推理 |
| 仅 YOLO | `bash scripts/start_yolo.sh --real` | 订阅已有相机图像并启动网关 |
| Qt 观察 | `bash scripts/start_qt.sh --real` | 默认叠框；不取得驾驶控制权 |
| Qt 原图观察 | `bash scripts/start_qt.sh --real --camera-topic /camera/image_raw/compressed` | 仍共享 ROS 相机数据，不直连 ESP32 |
| Qt 遥控 | `bash scripts/start_qt.sh --real --enable` | 明确启用，仍经既有统一授权/取消/心跳门控；先在网页释放控制权 |
| RViz 传感器 | `bash scripts/start_rviz.sh --real --diagnostics` | 雷达、里程计、TF |
| RViz 图像 | `bash scripts/start_rviz.sh --real --vision` | 原图和叠框图；诊断终端另看 `/diagnostics` |
| 建图节点 | `bash scripts/start_mapping.sh --real` | 经同一 API；不启用运动，不绕过他人控制权 |
| 导航节点 | `bash scripts/start_navigation.sh --real --map room` | 仅启动已保存地图的导航节点，等待定位/目标 |
| 标定 | `bash scripts/calibrate.sh --help` | 审计、记录、尺度/角度拟合、棋盘标定与像素射线 |
| 模型对照 | `bash scripts/compare_yolo.sh --help` | 本地权重、标注图像、固定阈值比较 |

相机启动方式二选一：①统一 `start_vision.sh`；②分别启动 `start_camera.sh`、`start_yolo.sh`。不要先启动相机，再让一键入口重复启动整组视觉。相机节点按 Domain 与图像话题持有采集锁，重复启动会拒绝，不额外连接设备。

建图/导航 sh 使用临时控制会话并在操作结束后释放，只启动任务节点；没有启用遥控或发送目标。已有网页控制者时明确拒绝，不接管。建议日常直接在网页操作；若使用脚本，先在网页释放控制权，脚本返回后再在网页取得控制权，完成遥控/初始定位/目标。建图停止及地图保存走网页现有流程。

## 5173 的摄像头与识别框

一键入口会自动请求视觉启动，页面默认“YOLO 识别图”。在“摄像头与 YOLO”卡片可切换原图/识别图、全部类别/交通灯与 STOP，查看摄像头和推理各自状态。所有画面经本机网关与 Node 代理，浏览器不直接连接 ESP32。

只启动旧网页入口时，需要在卡片点击“启动视觉”。相机正常但 YOLO 加载中或故障时，可切“原图”；识别无效时不会把旧检测继续当成有效结果。更新后的后端必须重启，Vite 热更新不能代替重启 Node。

Qt 默认同样订阅 ROS 叠框图；本批未修改 Qt 的现有布局与运动安全流程。当前自动测试环境没有桌面显示，Qt/RViz 窗口仍需在桌面实际打开核对；这里没有把“构建通过”当成窗口验收。

## 模型选择

默认继续用 **YOLO26n + PyTorch + RTX 4050**，已有真实 GPU 长稳基线。YOLO26s 是可比较的候选；官方 COCO 指标显示 s 的总体精度更高、计算量也更大，但官方其他硬件/TensorRT 数据不能替代本机 PyTorch 或实拍精度。[Ultralytics 模型说明](https://docs.ultralytics.com/models/yolo26/)

本批用现有 n/s 权重对同样 5 张带标注图片实际推理，`imgsz=640`、confidence=0.25、IoU=0.5：

| 模型 | IPC 推理延迟中位数 | 交通灯 TP / FP / FN | STOP TP / FP / FN |
| --- | --- | --- | --- |
| YOLO26n | 16.82 ms | 3 / 0 / 10 | 1 / 0 / 1 |
| YOLO26s | 17.99 ms | 3 / 1 / 10 | 2 / 0 / 0 |

这是小样本固定阈值的结果，**不是 mAP，也不是实车准确率**。它支持保留 n 为默认、把 s 留作实拍对照，不能说明换成 s 就解决远处小灯漏检。下一步优先标定相机、采集实际光照/距离样本，再比较召回率与误检；灯色分类仍未实现，不能直接控制红绿灯行驶。

本机可手动试 s（先停止原视觉会话，其他行为相同）：

```bash
bash scripts/start_vision.sh --real \
  'url:=http://<CAMERA_IP>:81/' \
  'model_path:=/home/<user>/ros-car-migration-audit/yolo-integration/yolo26s.pt'
```

本批备份、测试和回滚清单在 `/home/<user>/ros-car-migration-audit/calibration-startup-20261108/`，测试摘要见 [本批验收记录](LDC标定与启动测试记录.md)。
