# LDC 诊断、标定向导与故障回放

更新：2026-11-10。本批对应《下一阶段优化计划》的第一批软件交付。实车离线，本文的阈值和软件测试不代表已完成实车标定。

## 日常入口

桌面 → **LDC启动工具 → 标定向导与诊断**（文件名仍是 `07_标定工具.desktop`）。自动打开日志终端及 <http://localhost:5173/#calibration>。

- 已有实车网页：直接复用，关闭本入口不关闭它。
- 网页未运行：启动网页栈，在终端显示日志；Ctrl+C 停止本次创建的进程。
- 端口占用或当前为仿真网页：显示原因，不终止其他程序，不混用资料。
- 该入口不启动 Agent、底盘或导航，不授予运动权限。接入实车仍使用“一键实车启动”。

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_calibration.sh
# 仅检查网页可用性，不启动或打开窗口
bash scripts/start_calibration.sh --check
```

原 `scripts/calibrate.sh` 和桌面“单独模块”的 CLI 标定入口继续保留。详细测量原理见 [标定与精度验收](LDC标定与精度验收.md)。

## 导航异常先看哪里

网页导航面板区分六项：雷达 `/scan`、原始里程计 `/odom`、TF、AMCL、Nav2 和控制页面。每项显示具体缺失原因；“最近停车”保留最后一次取消或锁定的原因，不会被正常轮询清除。

展开“实际算法与定位质量”：查看地图、从 ROS 参数服务读取的规划器/控制器插件、关键参数 SHA256、启动参数文件 SHA256、参数读取完整度和新鲜度、AMCL 不确定度。读不到实际插件时显示等待，不猜测。导航停止后显示“上次运行配置与定位记录”。

固件仍标为“尚未实车核对”。源码不证明车内已刷入同一版本。默认算法仍为 **AMCL + NavFn + DWB**，本批没有启用 EKF、Smac 或 MPPI。

### AMCL 规则

| 检查 | 当前实现 |
|---|---|
| 内容 | `frame_id=map`；位置、四元数和协方差为有限数；四元数归一化；协方差对称、半正定，x/y/yaw 方差大于零 |
| 来源时间 | 正时间戳，到达时年龄 -0.25～1.5 秒；属于本次初始位姿会话，且比上次接受的时间更新；旧包不续期 |
| 时间配对 | AMCL 与原始 odom 时间差 ≤0.25 秒 |
| 静止 | 未超过 AMCL 运动更新阈值时，允许 AMCL 不发布；不按接收间隔误报 |
| 运动 | 相对上次有效定位的 odom 位移/转角超过 `update_min_d/a` 后，1.5 秒内应有新定位；启动暂用 0.25m/0.2rad，随后读实际参数 |
| 不确定度 | 位置 sigma 诊断上限 0.75m，朝向 0.75rad |
| 异常修正 | 扣除同段 odom 运动后，AMCL 额外修正超过 0.75m 或 60°，锁存并要求重新设置初始位置 |
| 时间回拨 | ROS 时间比已接受定位落后超过 0.25 秒，锁存并要求重新设置初始位置 |
| TF/原始输入 | 原雷达、odom 时效检查仍生效；需有效 `map → base_footprint` |

这些是初始工程阈值，不是实测定位精度。定位失效会取消自动任务并锁定；数据恢复不续跑旧目标。跳变/回拨先重新设置初始位置，其他故障先排查原因，再明确重试目标或启用遥控。

## 标定向导

1. 新建会话，写明日期、地面、载荷与目的。尺寸初始为 `null`，CSV 不填虚构测量。
2. 按顺序选择：**车体与原点 → 编码器 → 直行 → 转角 → IMU → 雷达 → 相机 → 停车距离**；最后做独立导航验收。
3. 设置采集时长、容量、相机采样间隔，点击“开始只读采集”。关闭向导后，从原控制页面做试验或复现故障，后台继续记录。
4. 停车、结束采集，查看测量总结；把本次 odom 读数和独立尺量真值填入。后退和顺时针使用负值，不用 AMCL/odom 自己当真值。
5. 生成候选。网页显示原始误差、样本量、候选值与警告；JSON 保留基线、来源哈希和报告 ID。
6. 选择准备验证的候选报告，核对“独立复测将关联报告”中的版本，再点击“新建当前报告的独立复测”。可以选择历史报告；不会自动改为最新报告。记录另一批试验与实际应用的固件/参数版本，复测会话不复制旧试验数据。

打开向导会结束本页持续发送的手动方向指令。弹窗内禁止 WASD 行驶，Esc 仍使用原急停；关闭向导后再人工遥控。打开向导本身不取消正在执行的自动导航。

所有候选为 `applied=false`，没有自动写入固件、URDF、Nav2 或相机配置的接口。使用流程为：候选 → 核对来源 → 实车到位后单独实施参数变更 → 新会话复测 → 人工核对版本和结果。无实物时不能完成后两项。

直行复测使用原始相对误差的最大值，目标 ≤1%；转角使用每 360° 的平均绝对误差，目标 ≤2°。正反方向各至少 5 次才满足样本量。“数值达标”“样本足够”“实际版本已核对”分别判断，不自动放行。拟合残差不替代独立复测。

### 相机

选择相机内参，下载 9×6 内角点 A4 棋盘，按实际大小打印，尺量格长填入 mm。固定分辨率、镜头、翻转及安装方式，采集 25–40 张覆盖四角、边缘和倾斜角的棋盘；相机采样间隔设为 1 秒。

须先启动现有摄像头/视觉服务。采集器订阅 `/camera/image_raw/compressed`，不增加 ESP32 连接；候选及留出验证复用现有工具。型号未知时保持未知。二维检测框和像素方向不能直接换算车到目标的米制距离。

## 故障记录

推荐复现前开启 60 秒、128MiB 的采集；需画面时采样间隔 1 秒。原日志导出继续使用：文本日志看错误，新 CDR 记录保留消息原始字段与 Header。

```bash
cd /home/<user>/ros-car-migration
# output 必须是新目录，不覆盖旧数据
bash scripts/diagnose.sh record --real \
  --output /home/<user>/ldc-recordings/problem-001 \
  --seconds 60 --max-mb 128 --camera-every 1
bash scripts/diagnose.sh report /home/<user>/ldc-recordings/problem-001
bash scripts/diagnose.sh summarize /home/<user>/ldc-recordings/problem-001
```

记录 `/scan`、`/odom`、`/imu`、AMCL、地图、路径、TF/static、初始位姿、速度、Action 状态、诊断、电池、CameraInfo，以及抽样的原始 JPEG。每 0.5 秒尝试获取导航状态、目标/反馈、事件和参数快照。原 CDR 保留雷达 `Inf/NaN`，不经 JSON 改写。

| 文件 | 内容 |
|---|---|
| `capture.json` | 状态、停止原因、话题数量、容量、时间、Domain、SHA256 |
| `recording.sqlite3` | 已收到的 ROS CDR 与导航快照/事件 |
| `images/` | 抽取的原始 JPEG，可用于相机标定 |
| `summary.json` | 显式生成的位移、累计角度、IMU 与墙面摘要 |

订阅为 Best Effort、depth=1，不承诺无丢包；相机抽样；快照队列最多 1 个。队列满时不提前推进事件游标，后端重启/事件超出缓冲会标明缺口。网页和 `report` 显示最后 120 个快照及总数，全量留库。

默认 60 秒/128MiB，上限 600 秒/512MiB。网页开始采集前检查“已有资料＋本次预算 ≤2GiB”及磁盘余量。达到容量后结束，不删旧资料。写盘失败为 `failed`，命令返回非零；后台重启遗留的 `recording` 显示 `interrupted`，不自动继续。仅收到后台零速度等消息时，界面提示“无雷达/里程计/IMU数据”，录制结束不代表实车正常。

实车会话在 `~/.local/share/ldc/calibration/`，开发资料在 `~/.local/share/ldc/simulation/calibration/`。尺寸/CSV 修改前留 `revisions/`；报告保存来源快照。接口返回最近 100 条测量和准确总数，完整 CSV 保留。

## 隔离回放与 RViz（开发用）

网页折叠区“开发回放（独立环境）”或：

```bash
cd /home/<user>/ros-car-migration
bash scripts/diagnose.sh replay /home/<user>/ldc-recordings/problem-001 --rate 1
```

强制 **Domain 196、本机发现**，清除自定义 DDS peers/discovery server。校验完整归档 SHA256，再发布保留原 Header 的历史传感器与 `/clock`，倍率 0.1～4。

历史 `/cmd_vel`、`/cmd_vel_auto`、`/initialpose` 和 Action 状态只发 `/replay/recorded/...` 观察话题，不提交目标、不调用运动服务。中断或哈希不符的记录拒绝回放，仍可先查看报告与摘要。

另开终端观察 RViz：

```bash
cd /home/<user>/ros-car-migration
source scripts/env.bash
export ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_STATIC_PEERS ROS_DISCOVERY_SERVER CYCLONEDDS_URI
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
rviz2 -d scripts/rviz/diagnostics.rviz --ros-args -p use_sim_time:=true
```

默认实车 RViz 入口会选择实车 Domain，不能直接用于这次回放。回放不重新执行 Nav2 闭环，主要用于复现传感器/TF 与故障时间线；算法对照仍需独立实验。仿真入口继续放在独立目录。

## 代码与接口

| 代码 | 职责 |
|---|---|
| `scripts/localization_health.py` | AMCL 内容、时间、运动更新与跳变 |
| `scripts/navigation_profile.py` | 运行中 Nav2 参数白名单与指纹 |
| `scripts/navigation_bridge.py` | Action/控制权管理，接入定位故障锁定 |
| `web_console/navigationRoutes.js` | 兼容扩展导航状态、启动文件指纹 |
| `web_console/src/components/ReadinessPanel.vue` | 六项检查、停车原因、配置 |
| `scripts/calibration/workspace.py` | 会话、修订、候选、复测、相机工具 |
| `scripts/diagnostics/` | CDR 采集、报告、总结、隔离回放 |
| `web_console/calibrationRoutes.js` | API 与专属子进程清理 |
| `web_console/src/components/CalibrationPanel.vue` | 四步向导与报告 |
| `scripts/start_calibration.sh`、`open_calibration.py` | 网页复用、桌面入口 |

接口前缀 `/api/calibration`：

- `GET/POST /sessions`、`GET /sessions/:id`：查询/创建/恢复。
- `POST /sessions/:id/dimensions|measurement|observations`：保存尺寸/试验/观测。
- `POST /sessions/:id/analyze|camera`：仅生成候选。
- `POST /sessions/:id/captures`、`POST /stop`：开始/结束采集。
- `POST /sessions/:id/captures/:capture/report|summarize|replay`：报告/摘要/回放。
- `GET /sessions/:id/board`：棋盘。

`/api/navigation_status` 保持原字段，新增详细 `checks.localization`、`runtimeProfile`、`lastStop`、`configuration`。事件接口继续提供完整目标执行时间线。修改型标定请求拒绝其他站点 Origin；工具参数采用结构化 argv/stdin。

## 后续放行顺序

| 阶段 | 当前状态及条件 |
|---|---|
| 第一批诊断/向导/记录 | 软件已实现，待实车验证消息与测量流程 |
| 第二批几何与数据 | 待实测整车包络/原点，核对固件、编码器、传感器和扫描时间 |
| 第三批二维 EKF | 未启用；同批数据对照应优于原 odom；唯一 `odom → base_footprint` 发布者；持续监测原始输入 |
| 第四批导航基线 | 标定后整定 AMCL/DWB、进度检查、近目标转向、加减速、障碍层和一致 footprint |
| 第五批算法对照 | 未替换；逐层比较 GMapping/slam_toolbox、NavFn/Smac State Lattice、DWB/MPPI DiffDrive |
| 视觉质量 | 保留 YOLO26n；固定相机后准备 ≥300 张本车分批标注图，对照 n/s 与 640/960 的精度、延迟和资源 |
| 灯色与交通观察 | 待实现；红/黄/绿/未知及冲突、遮挡测试，先记录建议，不控制车 |
| 交通任务 | 未启用；先建立路口/灯具/停止线关联；STOP 停稳 3 秒；急停后绿灯不恢复旧任务 |

未来 Collision Monitor 放在授权仲裁/平滑之后、MCU 门控之前；本批没有接入。精度、导航成功率、停车距离、实拍视觉精度仍是待验证目标。视觉继续仅识别显示。

证据与回滚见 [首批测试报告](LDC诊断标定首批测试报告.md)。2026-11-10 后续修复了定位门限、采集/回放互斥及复测报告关联，见 [复核问题修复记录](LDC复核问题修复记录-20261110.md)。
