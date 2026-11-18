# LDC 标定与启动测试记录（2026-11-08）

本批实车离线。交付标定工具、各模块 sh、总启动/停止入口、网页默认叠框、Qt 叠框观察；不把模拟或合成数据当成真实标定。不修改固件运动参数、URDF、Nav2 YAML、雷达参数，不刷写或自动授权小车。

审计与备份：`/home/<user>/ros-car-migration-audit/calibration-startup-20261108/`。

| 验证项 | 实际结果与证据 |
| --- | --- |
| ROS 构建 | ldc_camera、ldc_vision、ldc_qt_gui 三包通过，`build.log` |
| 前端生产构建 | 通过，`frontend-build.log` |
| Python 回归 | 46 项通过（原 32 项＋标定/启动新增 14 项），`python-tests.log` |
| 网页回归 | 45 项通过，含默认识别图、原图切换、故障清空，`web-tests.log` |
| 视觉核心 | 15 项通过，`vision-unit.log` |
| 底盘核心 | 现有 ASan/UBSan 测试通过，含 65 周期 motion_task/授权故障注入，`chassis-tests.log` |
| 几何审计 | 检出 131.7/162mm 轮距差、轮轴参考点偏移、100mm 圆小于 CAD 水平包络；`geometry-audit.json` |
| 空白标定 | 尺寸 null、CSV 仅表头、报告 awaiting_measurements；`calibration-session/`、`calibration-pending.json` |
| 数学验证 | 合成已知内参恢复、留出视角误差、单位射线/尺寸拒绝、尺度公式方向、累计 360°、时间跳变拒绝；均在 Python 回归中 |
| 变异验证 | 反转尺度方向校验导致测试失败，随后恢复正确源码，`mutation.log` |
| 一键启动 | Domain197 本机仿真，网页 **localhost:5173**、API18310、真实 GPU 推理、单相机连接通过；`one-click-ready.json` |
| 幂等与状态 | 重复启动返回同实例，状态与实际 PID 对应；正常 stop 后 running=false/phase=stopped，`repeated-start.json`、`final-status.json` |
| 多路预览 | 3 个正常客户端约 12 秒各 121 帧，另一个慢客户端不拖慢采集，源同时连接=1；`preview.json` |
| 只读采集 | 实际订阅模拟 odom/scan/TF/CameraInfo/JPEG，保存原始帧、生成 0 位移/0转角总结；`sim-capture/`、`sim-summary.json`。模拟数据明确不作为真实标定 |
| 建图脚本 | 经 API 启动、临时租约释放，地图保存并停止，只写隔离目录；`mapping-start.log`、`mapping-test.json` |
| 导航脚本 | 经 API 启动并等待定位，mode=locked、goalId=null，停止成功；`navigation-start.log`、`navigation-test.json` |
| 控制权保护 | 长任务期间只续期临时租约，结束释放；已有控制者不强行接管；未调用 manual_enable/velocity/goals。慢启动心跳用例已加入回归 |
| 视觉故障回归 | 分片/断流/重连、worker 超时/退出、GPU 不可用、ROS 节点退出、旧结果清除、原图保留，`vision-regression/report.json` |
| 后端异常 | 隔离后端 SIGKILL，监督进程回收脱离原父进程的 ROS/worker 并退出，`backend-crash.json` |
| Qt | `QT_QPA_PLATFORM=offscreen` 实际启动 C++ Qt，只读观察；参数查询确认 `/vision/annotated/compressed`，`qt-offscreen.log`、`qt-camera-topic.txt` |
| 最终清理 | 测试端口关闭、Qt/摄像头/YOLO 进程清理；物理控制/几何配置哈希与初始快照一致，`cleanup.json` |

## 模型对照的边界

使用本地已有 YOLO26n / YOLO26s、现有 Python 3.12 / PyTorch / RTX4050。仅 5 张带完整 YOLO 标签的 COCO128 代表图片，imgsz640、confidence0.25、IoU0.5；真实模型推理，未用人工框替代结果。每模型延迟统计 15 次 IPC 请求。

- n：交通灯 TP/FP/FN = 3/0/10；STOP = 1/0/1；延迟中位数约 16.82ms。
- s：交通灯 TP/FP/FN = 3/1/10；STOP = 2/0/0；延迟中位数约 17.99ms。

完整输出与权重 SHA256：`model-comparison/report.json` / 每模型预测文件。这个结果不代表 mAP、实车准确率或交通决策可靠性；只能说明该小样本下 s 改善了一个 STOP 漏检，未改善交通灯召回。默认仍保留已完成过长稳验证的 n。本批没有重新运行 30 分钟长稳，上批性能记录见 [视觉接入测试报告](LDC视觉接入测试报告.md)。

## 发现并处理的问题

- 不同启动方式可能重复打开同一图像采集：新增 Domain/图像话题采集锁。
- 建图/导航脚本的 POST 不能依赖 urllib 自动跟随 307：改用原始 API 路径。
- 控制桥刚启动时还没收到第一帧传感器：请求前等待真实 lidar/odom/TF 就绪，不伪造在线。
- 慢启动可能超过临时租约寿命：增加心跳；操作结束释放，不持有驾驶权，不解除急停。
- Node 被异常终止时 worker 可能脱离原父进程：总入口使用本进程树的 Linux child subreaper，再按身份回收子进程，不按进程名杀全局服务。
- 参数输出、标定目录拒绝覆盖；反号、NaN、重复试验 ID、混合分辨率及不可靠时间数据拒绝。

## 未验收项

实车尺寸、物理轮径/CPR、有效轮距、左右轮不对称、真实制动距离、相机内参与安装外参、激光/IMU方向、真实导航与识别精度均待现场测量。

Qt 的无显示启动和话题参数已经验证，但桌面窗口布局与实际画面仍需人工打开核对。Qt 既有话题查询在日志中出现一次 QProcess 结束警告，本批未重构该 C++ 查询逻辑；未把无显示启动当作视觉布局验收。RViz 本批沿用现有入口，缺少 X 显示的运行限制仍在，未声称完成 RViz 窗口验收。浏览器自动工具仍缺少 Chrome，本批用 Vue 回归、真实 HTTP 和 MJPEG 流验证，未补写虚假浏览器截图。

复现命令见 [一键启动指南](LDC一键启动与模块入口.md)、[标定说明](LDC标定与精度验收.md)。逐文件备份、差异、源码清单及回滚工具位于本批审计目录，不使用 Git 硬重置。
