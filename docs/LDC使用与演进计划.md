# LDC 使用说明与后续计划

新增：[Arena 候选规划器修复与验收](LDCArena规划器修复与验收.md)，包含地图快照、线段碰撞和终点朝向修复及独立测试入口。日常默认算法仍为 AMCL + NavFn + DWB。

更新：2026-11-10。新增 [诊断、网页标定向导与故障回放](LDC诊断标定与故障回放.md)及[本批测试报告](LDC诊断标定首批测试报告.md)。桌面“标定向导与诊断”直接打开网页；AMCL 消息、运动更新和异常跳变检查已接入。实测标定未完成，默认 AMCL + NavFn + DWB 保持，EKF 与视觉驾驶决策未启用。

已有 [一键启动与模块入口](LDC一键启动与模块入口.md)、[标定与精度验收](LDC标定与精度验收.md) 继续适用。工程实际目录仍为 `/home/<user>/ros-car-migration`，功能目录为 `firmware/`、`robot_ws/`、`agent_ws/`、`web_console/`。不要使用旧路径 `Ros_Vue_x86`、`xuegeros_ws` 或旧 `xuegecar_*` 包名启动。

最新补充：[数据有效性修复记录](LDC数据有效性修复记录.md)。新一批已通过完整 ESP-IDF 构建及22项离线集成验收；下文所述首批固件构建阻塞是当时记录，已由明确指定现有工具路径解决。固件仍未刷入实车。

## 一、完成情况和验证边界

| 部分 | 已实现 | 验证边界 |
|---|---|---|
| 网页遥控 | 控制权、速度档位、按住行驶/持续行驶、松开或失焦停车、心跳及数据失效锁定 | 有软件测试；实际停车距离需实车测量 |
| 在线状态 | 区分数据桥接在线和车端数据正常；过期数据隐藏，恢复不自动启用遥控 | 软件回归通过；桥接在线不代表底盘可动 |
| 建图与地图保存 | GMapping；保存成功后停止、失败可重试；地图选择与最近保存记录 | 已有离线闭环验收；地图质量依赖实车标定 |
| 导航 | AMCL + Nav2，后端 NavigateToPose Action 管理，目标受理/结果/取消、多点队列、就绪检查 | 离线闭环已有通过记录；实车存在定位、规划与雷达断流问题，尚未稳定验收 |
| RViz | 基础诊断、导航观察两套配置与启动脚本 | 提供观察入口；不代替网页控制权和安全门控 |
| 无车测试 | 独立 Domain 与端口，模拟底盘和传感器，运行真实 GMapping/AMCL/Nav2 | 二维模拟通过不代表实际避障通过 |
| 2026-11-07 MCU 改动 | 持续采样、真实 dt、编码器中点里程计、输入校验、PCNT 累加、部分超时保护 | 宿主 ASan/UBSan 通过；完整 IDF 构建受环境缺失阻塞，未刷车 |
| 融合定位 | 已有 EKF 配置文件 | 当前标准 bringup 未启用 EKF；不能宣称已使用融合定位 |

2026-11-07 历史批次为 25 项 Python 回归及控制器宿主/平滑器测试通过，记录在 `/home/<user>/ros-car-migration-audit/safety-20261107/RESULTS.md`。最新导航诊断批次的回归、网页和隔离 ROS 证据以[本批测试报告](LDC诊断标定首批测试报告.md)为准。

## 二、日常启动：实车模式

每条持续运行的命令使用独立终端；不要重复启动已有实例。当前 UDP 配置要求车端 Agent 地址指向这台电脑在同一网络上的 IP，端口 8888。换 Wi-Fi 后电脑 IP 可能变化，不能一直使用历史 IP。

### 终端 1：Agent

```bash
cd /home/<user>/ros-car-migration
source scripts/env.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888 -v 4
```

这是 micro-ROS UDP 模式入口。若固件选的是 UART 或 MAVLink 模式，不适用这个命令；先确认车端通信模式，不要同时叠加多个驱动。Agent 有客户端会话和实体建立日志才说明车端开始接入。

### 终端 2：底盘与 TF

```bash
cd /home/<user>/ros-car-migration
source scripts/env.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
ros2 launch ldc_bringup ldc_bringup.launch.py
```

此入口启动底盘主机节点、robot_state_publisher、joint_state_publisher。当前并未启动 EKF。只有 Agent 不足以生成完整主机 TF。

### 终端 3：网页

```bash
cd /home/<user>/ros-car-migration
bash web_console/start_console.sh
```

打开 http://localhost:5173/。API 为 3000，Foxglove 为 8765。该脚本不启动 Agent 或底盘 bringup。端口占用报错时，检查原终端或下文的 `ss`，不要不断重复启动。

页面应显示“车端数据正常”；首次打开或程序更新后可 Ctrl+Shift+R。若只显示“桥接在线”，先查 Agent、雷达、odom 与 TF，暂不操作运动。

### 终端 4：基础体检（按需）

```bash
cd /home/<user>/ros-car-migration
bash scripts/ros_doctor.sh --real --log /tmp/ldc-real-diagnosis.log
```

没有启动导航时，“等待数据/等待定位”可能是当前正常状态，要看具体缺失条件。诊断脚本会检查端口、节点、话题/QoS、TF、生命周期与 Action；在特定守护进程缓存异常时会停止并重建 ROS CLI daemon，因此它并非完全无副作用的纯读取。

## 三、遥控、建图与保存

1. 先确认数据正常，取得页面控制权。多开页面时仅由一个页面控制。
2. 选慢速档和“按住行驶”；点击启用遥控后，短按方向验证。WASD 同样遵循按住/松开。换档后需要重新启用。
3. 点击“开始建图”，缓慢覆盖周围可通行区域；转弯尽量平稳，观察墙面有无重影。仅看见红色雷达点不代表地图已生成。
4. 完成后先停车，再“保存并停止建图”。等待页面确认保存名称和时间；保存失败时保留建图并按提示重试，不要直接杀掉进程。
5. 实车地图在项目 `maps/` 下，通常是一组 `.yaml` 与 `.pgm`；两者共同构成一张地图。不要覆盖历史图，包括 `live_scan_20261103_1733`。
6. 后续导航必须明确选择本次保存的地图。最近保存地图和当前加载地图可能不是同一张。

取消当前导航目标、停止导航栈、急停是不同操作。需要手动接管时走网页启用遥控，由后端取消并确认；不要再开一个直接发布 `/cmd_vel` 的遥控程序。

## 四、定位和导航

1. 停止建图，选择已保存地图，启动导航。
2. 等待“请设置初始位置”。点击“设置初始位置”，在地图中按住拖动：落点对应实际位置，拖动方向对应实际车头方向。
3. 等待 AMCL 更新、TF 有效和“导航就绪”。不要以进程存在或按钮变色代替就绪检查。
4. 看实时雷达与地图墙面是否大致重合。不重合时先修正初始位姿，检查地图是否选错；不要反复发送远距离目标。
5. 双击可通行空白区域，检查目标预览的位置和朝向，点击“发送目标”。首次实车验收先选约 0.3–0.5 m 的近目标。
6. 以 Action 的“已到达”为成功；反馈中的剩余距离小不等于任务已成功。
7. 单点稳定后再开启多点模式，添加航点并点击“执行航点队列”。失败或取消后队列停止，不应自行续跑。

白色通常为已知可通行区域，深灰为未知；蓝色通常是代价地图叠层及障碍膨胀区域，不是新生成的墙。蓝色覆盖狭窄通道可能令规划无路，不能为了过图直接缩小真实车体 footprint。

当前链路：网页 HTTP → Node 后端 → Python Bridge → NavigateToPose → Nav2 规划/控制 → velocity smoother → 后端运动门控 → `/cmd_vel` → MCU。Foxglove 负责遥测。当前默认导航使用 AMCL、NavFn 和 DWB；自定义 ArenaPlanner 是独立候选，不应把其改动当作默认导航已启用。

## 五、RViz 入门

### 1. 先用基础诊断视图

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_rviz.sh --real --diagnostics
```

Fixed Frame 是 `odom`，主要看 `/scan`、`/odom` 和 TF。此模式无需先开建图，适合检查底层链路。odom 也不存在时应先解决 Agent/bringup。

### 2. 再用建图/导航观察视图

先在网页开始建图或启动导航，然后运行：

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_rviz.sh --real
```

Fixed Frame 是 `map`，已配置 `/map`、全局/局部 costmap、`/scan`、`/amcl_pose`、`/plan` 和 TF。建图阶段没有 AMCL 或规划路径属于正常情况。页面桥接未开放的某些图层在 RViz 仍可直接订阅；代价图层只有对应节点运行时才有数据。

### 3. 窗口里怎么操作

- 左侧 Displays：勾选图层显示/隐藏，展开状态查看具体报错。
- Grid 是参考网格；LaserScan 是障碍回波；Map 是地图；Path 是规划路径；TF 是坐标轴关系。
- 导航默认 TopDownOrtho 俯视视角：滚轮缩放，使用顶部 Move Camera 拖动视图；Views 面板可调整 Scale 和中心。实际鼠标组合随视图模式不同，先看底部状态栏提示。
- RobotModel 默认可关闭；没有模型不必然意味着雷达或导航故障。
- 初始位置、目标、取消和急停统一使用网页。这两套 RViz 配置刻意没有 2D Pose Estimate/Nav2 Goal 工具，避免绕过后端状态和控制权。
- 不要为了消除红色提示随意把 `map` 改成雷达坐标系；固定坐标系应按当前任务选择。

### 4. 常见提示

| 现象 | 检查与处理 |
|---|---|
| Fixed Frame [map] does not exist | 尚未建图/定位时用 diagnostics；已启动则查 SLAM 或 AMCL 与 map→odom |
| Fixed Frame [odom] does not exist | 查底盘 bringup、odom 数据和 odom→base TF |
| 雷达 No messages | 查 `/scan` 发布者、Domain、QoS 和 Agent；先确认消息真的在更新 |
| Message Filter dropping / timestamp earlier | 检查时间戳、TF 缓存、重复 TF 发布及雷达延迟；不能只增大 timeout |
| 地图有但雷达错位 | 核对地图、初始位置朝向、雷达安装 TF、时间戳与里程尺度 |
| 地图不显示但话题存在 | 核对 Map 图层 Topic 和 Durability；静态地图通常需要 Transient Local |
| 路径为空 | 先确认目标已受理和导航就绪，再看规划失败原因；未下目标时本就可能没有路径 |

预期坐标链是 `map → odom → base_footprint → base_link → sensors`。建图的 map→odom 由 SLAM 提供；已知地图导航由 AMCL 提供。未来切换 EKF 时 odom→base 必须只有一个责任节点。

## 六、无车时练习与软件验收

```bash
cd /home/<user>/ros-car-migration
bash scripts/offline_console.sh
```

打开 http://localhost:5174/，确认“仿真”标志。用同样的建图→保存→选择新图→初始定位→目标流程练习。初始模拟位姿 `(0,0,0)`，移动后不能继续假设仍在原点。

仿真 RViz：

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_rviz.sh --sim --diagnostics
# 开始建图或导航后，可改用：
bash scripts/start_rviz.sh --sim
```

实车 Domain 0 / 网页5173 / API3000 / WS8765；仿真 Domain197 / 网页5174 / API18310 / WS8766。终端环境不要混用。退出仿真入口用 Ctrl+C，脚本清理其子进程和服务；不要杀全系统 ROS 进程。

自动验收请先关闭手动仿真实例：

```bash
cd /home/<user>/ros-car-migration
bash scripts/test_chassis_core.sh
source scripts/env.bash
python3 -m unittest discover -s scripts/tests -p 'test_*.py'
npm --prefix web_console test
npm --prefix web_console run build
bash scripts/offline_console.sh --test --output "/tmp/ldc-acceptance-$(date +%Y%m%d-%H%M%S)"
```

每次用新输出目录，查看 `acceptance.json` 和 `backend.log`。固件宿主测试通过不等于 ESP-IDF 完整构建通过，更不等于运行中固件已经更新。

## 七、排障与停止顺序

先在网页停车/取消任务、停止导航或完成地图保存，再退出网页启动终端、bringup 和 Agent。未确认停车时使用独立电源停止手段，不依赖关闭浏览器。

用于读取状态的常用命令（新终端先 source scripts/env.bash，并指定实车 Domain0、SUBNET）：

```bash
ros2 node list
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 topic info /cmd_vel --verbose
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
curl -fsS http://127.0.0.1:3000/api/navigation_status
ss -ltnup | rg ':(3000|5173|8765|8888)\b'
```

`ros2 topic hz /scan` 和 `/odom` 可采集一段时间后 Ctrl+C；实际频率受通信模式影响，不预设所有环境相同。单独的 ping 成功不能证明 ROS 实体、传感器与运动控制链路正常。

## 八、接下来按什么顺序推进

按交付和验收条件推进，不提前承诺缺少实车数据支撑的参数或工期。

### 阶段 A：完成 MCU 安全闭环（下一优先级）

1. 修复 IDF 环境，先完整构建、保存版本与产物校验值，再考虑刷写。
2. 统一四种传输模式的安全契约：锁存急停、显式 reset/arm、会话与命令年龄、可靠安全事件、来源切换废弃旧命令。
3. 用同步状态替代共享 volatile flags，在最终 PWM 写入前复核授权；补独立驱动禁止/看门狗路径验证。
4. 落实前馈 PI 总输出限幅和抗饱和、反转互锁；完成 yaw 参数独立通信及持久化，保留旧参数可回滚。
5. PCNT 读错误改为可诊断的锁存故障，补长期 32 位溢出维护与硬件跨限验证。

验收：离地先验证上电不动、急停期间持续命令不解锁、解除后旧目标不恢复、断流停车、手柄释放不复跑、NaN/Inf、超期、传感器失败。记录开始停车与实际停稳时间，100 次急停/切换无非预期运动是放行目标，尚未完成。

### 阶段 B：数据契约、时间与融合定位

1. 编码器与 IMU 保存真实采样单调时刻，建立 Agent epoch 映射；重连废弃旧会话和旧时间映射。
2. 校准 wheel-only odom、IMU 轴向与噪声、雷达安装外参及首束时间；检查 QoS/MTU 与发布失败计数。
3. 先用 bag 证明 `/wheel/odometry` 和 `/imu/data_raw` 的单位、时间、协方差正确，再接入 EKF。
4. EKF 统一输出 `/odometry/filtered` 并独占 odom→base；同步修改 bringup、导航、仿真、网页与诊断，禁止只开 EKF 留双 TF。

验收：正反各 5 次 2m，顺逆各 5 次 360°，保存原始与融合结果；尺度误差≤1%、旋转平均≤2°/圈作为目标；100 次重连不续跑旧命令，时间配对误差和资源占用需实测。

### 阶段 C：稳定建图与导航

1. 固定一份基线 bag 和地图，不同时改底盘、时间与 SLAM。
2. 两个 costmap 使用统一的实测 footprint、安全余量和正确参考点。
3. A* 与平滑器增加完整边段/车体扫掠碰撞检查、不可变地图快照、原始路径及最终路径校验。
4. 定位稳定后整定 DWB；对已有 RotateToGoal、无合法轨迹、NavFn 无路、雷达断流建立可复现案例。
5. GMapping 作为基线，用同 bag 对照 Cartographer 后再决定切换。

验收：先近距离单点，再多点、窄道、取消、接管、断流、重启。20 个目标各 3 次成功率≥95%、到位≤5cm/5°是阶段目标，不是当前性能承诺。实际制动距离必须独立测量。

### 阶段 D：视觉与语义导航

视觉观察、CameraInfo 和主机检测已接通；相机内参/外参仍待实测。完成底盘与导航放行后，再接入深度关联、语义 TTL 和交通决策。二维框不能直接作为地图米制位置。语义模块只能提目标或代价，最终运动仍经过几何规划、碰撞检查和安全门控。

## 九、代码入口索引

- 环境与日常启动：`scripts/env.bash`、`web_console/start_console.sh`。
- RViz 与体检：`scripts/start_rviz.sh`、`scripts/rviz/`、`scripts/ros_doctor.sh`。
- 导航状态与 Action：`web_console/navigationRoutes.js`、`web_console/navigationBridge.js`、`scripts/navigation_bridge.py`。
- 导航启动与配置：`robot_ws/src/ldc_navigation/launch/` 以及该包的参数目录；以 launch 实际引用文件为准，不套用旧品牌参数路径。
- 固件：`firmware/chassis/main/tasks/control/motion_task.cpp`、`main/control/motion_controller.cpp`、`components/encoder_driver/`。
- 离线说明：`web_console/OFFLINE_NAVIGATION.md`。审计备份在工程外的 `ros-car-migration-audit/`。
