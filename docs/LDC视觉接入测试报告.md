# LDC 视觉接入测试报告（2026-11-08）

本报告针对主机 ROS、独立 YOLO worker、Node 网关和网页观察功能。摄像头固件、导航算法与 MCU 安全策略未改动。实机传感器型号待确认；本轮未以实机画面替代离线结果。

## 验证环境与证据

审计目录：`/home/<user>/ros-car-migration-audit/vision-20261108/`。

- Ubuntu 26.04、ROS 2 Lyrical、系统 Python 3.14.4，系统 OpenCV 4.10.0。
- 独立 Python 3.12.15，PyTorch 2.6.0+cu124、Ultralytics 8.4.183、CUDA 12.4、RTX 4050 Laptop GPU。
- 新安装标准消息包 `ros-lyrical-vision-msgs=4.2.0-3resolute.20260915.050641`。系统 Python 未安装 PyTorch。
- 权重 YOLO26n SHA256：`9b09cc8bf347f0fc8a5f7657480587f25db09b34bf33b0652110fb03a8ad4fef`。
- 长稳：Domain 197、localhost-only、模拟 MJPEG 18322、视觉网关 18321；故障注入独立使用 Domain 196、网关 18323、模拟源 18324。测试不会发现或控制实车。

## 已完成的自动验证

| 项目 | 结果与证据 |
| --- | --- |
| 摄像头 / 视觉 / URDF 构建 | 3 包构建通过，`ros-build.log` |
| 前端生产构建 | 通过，`frontend-build.log` |
| 现有 + 新增网页测试 | 45 项通过，`web-tests.log` |
| 现有 Python 回归 | 32 项通过，`python-regression.log` |
| 底盘核心回归 | 通过（ASan / UBSan，含真实 motion_task 仲裁测试），`chassis-regression.log` |
| 视觉核心单元测试 | 15 项通过，`unit.log` |
| 包内测试 | 5 项通过，`package-tests.xml` / `package-tests.log`；两个 ROS 包的 `colcon test` 通过，`colcon-test.log` |
| 新增网页专项 | 5 项通过，`vision-web-tests.log`，含组件编译、状态恢复、断线清空、参数边界与线程创建的 worker 清理 |
| 故障集成 | `integration-final/report.json`，见下文场景 |
| 原图 / 叠框代理并发 | 三个正常客户端在 12 秒各收到约 120 帧，一个慢客户端未阻塞采集；最多一路源连接，`proxy.json` |
| 重复 CLI 启动 | 返回同一会话状态，未启动第二采集，`repeated-start.json` |
| 仿真边界 | 多个 URL 参数的最终有效值也必须 loopback；非本地地址拒绝，`isolation-rejection.log` |
| 消息契约变异验证 | 故意反转帧 ID 校验后测试失败，随后恢复源码，`mutation.log` |
| 安装位置与 Python 隔离 | 原 camera_joint 的 origin/parent/child 均保持；标准光学旋转正确，系统 Python 无 torch，`architecture-check.json` |

`colcon test` 当前使用 unittest 运行器，不生成 JUnit XML，因此 `colcon test-result` 会显示 0 条 XML 记录；不能将其当成“执行了 0 条单元测试”。本批另用 pytest 执行同一包内测试并生成 `package-tests.xml`，实际为 5 项通过。最初检测到两个包没有包内测试而返回 code 5，已补齐可发现的测试后重新执行通过。

故障集成实际启动 ROS 节点和 GPU worker，验证了：

1. 原始 JPEG、raw RGB、CameraInfo、Detection2DArray、叠框图使用相同来源 Header，尺寸一致，正常帧不重复发布。
2. 模拟 UDP 发现使用真实发现协议、仅 loopback；MJPEG 分片和多个完整帧可被解析，截断 / 超大帧拒绝。
3. 摄像头冻结、断开、截断、超大帧、分片模式后的恢复；冻结时不重复发布缓存图，过期后网关清空旧目标并由 ROS 发空检测。
4. 交通类别过滤在 bus 样例上返回“服务健康且无目标”。
5. 四路预览含慢客户端，摄像头采集连接保持一条。
6. worker 超时、SIGKILL 退出、GPU 不可用：旧框和目标清空，原图可用，不静默降级 CPU。
7. 摄像头 ROS 节点退出时整组停止，worker 和 HTTP 连接清理；网关关闭可结束 launch。
8. 隔离 ROS 图中没有速度或导航目标发布者。检测仅为二维；没有持续追踪 ID 或有效三维定位输出。

## 真实模型样例

输入为已有 `bus.jpg` 和 COCO128 中选出的 5 张代表图片；只用于软件链路验证，没有训练，也没有完整训练集下载。来源和标签索引保存在 `fixtures/sources.json`。

| 输入 | GPU 真实推理结果 |
| --- | --- |
| `bus.jpg`（810×1080） | bus、person |
| `traffic_light_000000000359.jpg`（500×332） | 两个 traffic light，约 0.51 / 0.44；另有 car |
| `traffic_light_000000000257.jpg` | traffic light 及其他通用类别 |
| `traffic_light_000000000510.jpg` | 未检出 traffic light；存在其他类别输出 |
| `stop_sign_000000000151.jpg`（480×640） | stop sign，约 0.84；另有 train / truck |
| `stop_sign_000000000064.jpg` | 未检出 stop sign；存在其他类别输出 |

每张图均保存 `.original.jpg`、`.json`、`.annotated.jpg`，位于 `samples/`；`summary.json` 记录实际模型 SHA256。另在显式 CPU 配置下对 bus、交通灯 359、STOP 151 三张图片实际推理通过，结果位于 `samples-cpu/`。框坐标经过原图宽高边界检查；未用人工框替代 YOLO 输出。

这些结果说明接入与类别输出可用，**不能据此承诺实机精度、召回率或交通安全能力**。交通灯颜色没有推断，网页明确显示“灯色未判定”；不宣称支持中文“停”字或其他交通标志。

## 连续运行与统一入口

真实连续运行 **1800.0 秒**，180 次采样全部健康：最低 **9.91 FPS**，各采样所报 p95 的最大值 **138.64 ms**，最新待处理队列最多 0 帧（配置上限 1 帧），摄像头源同时连接数最多 1。最大采样帧年龄 0.179 秒，长稳期间源累计连接计数从 3 到 3，没有发生重连。

三个持续存在的采集 / ROS 协调 / GPU worker 进程，RSS 首尾变化分别约 -100 KiB、+1352 KiB、+228 KiB，没有观察到持续增长。原始采样是 `soak/samples.jsonl`，统计是 `soak/report.json` / `soak/memory.json`。本次长稳会话在网页和退出逻辑补齐期间持续运行，验证采集与推理核心的长期性能；最终安装版本另通过以下启动、退出和代理验证。没有用短测试充当 30 分钟。

最终版统一 API 实际验收通过（`api.json`）：重复启动保持同一 detector 会话、交通过滤的有效空结果、刷新后查询状态、停止后源连接归零、重复停止幂等、重启建立新会话而不复用旧目标。

最终版离线统一入口实际启动成功（`offline-entry.json`）：前端 HTTP 可访问、Domain 197 仿真、真实 GPU 推理正常，运行中返回的模型 SHA256 与记录一致。最终版代理并发结果见 `proxy-final.json`，含三个正常客户端与一个慢客户端。端口预检拒绝已有会话且不停止它，见 `preflight.json`。本批测试进程在验收后清理，不接管实车会话。

## GUI 与实机验收边界

- 浏览器工具未找到 Chrome 可执行文件，未安装额外系统浏览器；本批完成 Vue 组件编译 / 状态测试、生产构建和实际 HTTP 多路流验证，**未声称完成真实浏览器视觉验收**。
- RViz 启动尝试受当前会话没有 X 显示限制，Ogre 无法创建 GLX 窗口，记录于 `rviz-smoke.log`。配置已提供，使用内置 Image 显示，无额外检测插件；**仍需在桌面打开后核对两个图像窗口**。结构化诊断通过终端 `/diagnostics` 查看。
- 未连接实机相机验证实际画面、上下朝向、网络延迟、未知传感器型号或真实标定。
- 未把视觉结果接入导航、避障、刹车或运动授权；没有刷固件，也没有调整车辆动力参数。

## 复现与回滚

操作与复现命令见 [视觉接入与使用](LDC视觉接入与使用.md)。测试统一入口：`bash scripts/test_vision.sh --unit`、`--integration IMAGE.jpg`、`--soak IMAGE.jpg`。长稳入口运行完整 1800 秒并自动清理进程。

`before/`、`before-sha256.json`、`changes.json` 与 `rollback.py` 位于审计目录。支持逐文件回滚，拒绝覆盖交付后又修改的文件；新增文件移动隔离保留，不使用 Git 重置或批量删除用户改动。
