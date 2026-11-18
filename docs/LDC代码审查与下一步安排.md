# LDC 代码审查、优化方向与下一步安排

审查日期：2026-11-07。结论针对当前源码，不表示车上烧录版本相同。本轮仅新增分析文档和运行回归，没有修改运行代码、启动实车或发送运动命令。

## 结论与排序

当前应优先补齐“能可靠构建、能可靠停止、能识别真实有效数据”三项能力，再推进融合定位与导航整定。已有 Vue→后端→ROS Action 的闭环可保留；不需要为了优化重新搭建整套系统。

下一轮按以下批次推进：

1. **A：固定可复现基线，并补主机数据有效性测试。** 无车可执行，是最近的一批任务。
2. **B：统一 MCU 安全状态机、控制源和运动输出。** 完整 IDF 构建通过后才进入刷写候选。
3. **C：传感器采样时间、通信与轮速执行器。** 建立可以回放、标定和解释的数据。
4. **D：EKF 与 TF 交接。** 数据契约通过后接入，不能只启用旧 YAML。
5. **E：导航和规划稳定性。** 固定 bag/地图复现，再整定 DWB、验证碰撞几何。
6. **F：维护性和视觉扩展。** 随所属模块逐步拆分，视觉排在底盘与导航放行后。

## 已完成并值得保留的部分

- 后端 NavigateToPose Action 目标受理、反馈、成功/失败、取消与多点推进机制。
- 页面控制权、心跳、手动接管、锁定和断流恢复不自动续跑的主机策略。
- 地图保存成功再停止、失败保留建图、地图选择与保存记录。
- 独立 Domain/端口/地图目录的离线闭环环境，RViz 观察与链路诊断入口。
- MCU 的命令超时、持续采样、实测 dt、原始编码器中点积分、PCNT 驱动累加，以及位置 PID 不覆盖 yaw 的首批修改。

这些是不同层次的成果；宿主算法通过不等于真实任务、固件镜像或实车通过。

## 关键发现与优化方向

### R1：运动安全状态仍分散（P0，优先重构）

证据：[SystemFlags](../firmware/chassis/main/core/system_globals.h) 第 25 行起，急停、忙碌、手柄覆盖仍用共享 `volatile` 标志；[micro-ROS 订阅](../firmware/chassis/main/tasks/network/microros/microros_subscribers.cpp) 第 40 行直接按 Bool 改急停，MAVLink 与配网路径也能修改该标志。

[motion_task](../firmware/chassis/main/tasks/control/motion_task.cpp) 第 158 行把授权读取成一次布尔值传给控制器；[MotionController](../firmware/chassis/main/control/motion_controller.cpp) 第 311 行实际写电机时没有再次取得统一安全状态。跨任务急停在计算期间发生时，现有结构无法保证本次输出已经取消。`volatile` 不构成跨核同步协议。

方向：运动任务独占控制器和电机输出；网络、配网、手柄只提交事件。统一 `disarmed/armed/fault/estop` 与故障原因，区分停止、复位、授权；最终输出前复核状态。硬件驱动禁止、任务卡死后的停车路径需要独立验证。仅换成 atomic bool 不能解决完整状态切换。

验收：急停与解除在任意控制时序发生，不恢复旧动作；普通速度无法清故障；网络路径不直接调用控制器。100 次实车急停/切换无非预期运动是后续放行目标。

### R2：命令年龄与来源切换契约不完整（P1）

证据：[MotionMsg](../firmware/chassis/main/msg/motion_msg.h) 没有接收时间、会话代号、序号、动作 ID；motion_task 第 84 行按“出队时刻”刷新时间。积压或延迟命令因此可能被当作新命令。速度、位置和相对动作共用覆盖队列，后者没有确认与动作去重。MAVLink 在第 841 行仍直接调用 `robot.ResetOdometry()`，可能与控制任务采样竞争。

方向：入口保存单调接收时间和会话；按来源仲裁，切换废弃旧命令。速度用最新值邮箱；相对动作、reset、arm、cancel 用有界事件队列和 ACK。四种传输入口统一校验有限数、单位、边界；不能只有 micro-ROS 入口做校验。

验收：延迟出队、断线重连、重复动作 ID、手柄释放和旧会话重放均不产生意外运动。reset 由运动任务在约定条件下执行。

### R3：主机把收到数据等同于数据有效（P1，A批可先处理）

证据：[navigation_bridge.py](../scripts/navigation_bridge.py) 第 129–134 行：scan 只检查 ranges 非空，odom 无内容检查，都会刷新接收计时。第 381 行自动速度也没有有限数/边界校验。TF 新鲜度是额外条件，但无法证明传感器数值有效。

本轮运行实际回调函数的最小探针：时间戳 sec=1 的旧 odom，以及非空但含 NaN 的旧 scan，仍会刷新相应的接收计时。该探针只证明回调行为，不证明绕过了全部运动门控。

方向：区分链路收包、采样更新、数据有效三种状态。检查 stamp 年龄/未来偏差、序号推进、frame、数值与扫描元信息；定义无回波、全 Inf、NaN 的不同含义，不把全 Inf 一律判故障。对自动速度同样实施有限数与硬边界检查；给出可见的拒绝原因。

验收：旧包持续重放、无效四元数、异常扫描元数据、自动速度 NaN/Inf 不会维持可运动状态。消息新鲜但机器人静止的数据应正常通过。

### R4：固件可重复发布缓存并赋新时间（P1）

证据：[microros_publishers.cpp](../firmware/chassis/main/tasks/network/microros/microros_publishers.cpp) 第 95–132 行 peek odom/IMU 队列，随后使用发布时 epoch 盖章；[MotionMsg](../firmware/chassis/main/msg/motion_msg.h)、[ImuMsg](../firmware/chassis/main/msg/imu_msg.h) 未带采样时间。motion_task 对 IMU 和电池也反复 peek，缺少源样本年龄。

影响：传感器任务停住而通信任务还活着时，主机可能继续收到貌似新鲜的数据；只在主机增加 stamp 检查也不足以解决。

方向：样本携带 `sample_time_us/sequence/valid`，采样时生成；发布时映射为 ROS 时间，不改写成“现在”。低电压计数按新样本或明确持续时间判定，不能把同一旧样本重复计成多次。IMU 超龄时禁止依赖它的相对转向动作。

验收：冻结某个采样任务但保持网络正常，主机和 MCU 都能发现，而其他传感器可继续上报。

### R5：LaserScan 时间契约仍不正确（P1）

证据：[microros_runtime.cpp](../firmware/chassis/main/tasks/network/microros/microros_runtime.cpp) 第 51 行在设置 scan_time 之前计算 time_increment；发布端第 179–190 行对齐的是扫描完成时刻，尚不是首束时刻。序号、数据、时间还分属不同共享载体，需检查是否能取得同一帧快照。

方向：先核对真实扫描方向、角度数组索引和每束采样顺序，再确定 header 与 time_increment。不能仅交换两行、硬减 0.1 秒就宣称消除运动畸变。将帧数据、序号和时间绑定为一致快照；记录发布失败、丢帧和时间同步状态，核对实际 transport 的 MTU/可靠流设置。

验收：固定墙面 bag 对照，直行/旋转时检查雷达与 TF 对齐；用采样证据验证首束时间和数组语义。

### R6：执行器限幅与 PID 内部状态仍不统一（P1）

证据：[MotionController](../firmware/chassis/main/control/motion_controller.cpp) 第 129 行先把前馈和 PID 相加、施加最小 PWM，再最终限幅；PID 在第 301 行独立计算，不知道前馈已占用的输出余量。

方向：统一前馈+反馈总输出的限幅与抗饱和，反转经过零参考并检查实际轮速；参数按左右轮、正反向标定。当前首批优化没有完成这项重构，不能沿用旧 PID 数值并声称已经整定。

验收：低速、负载变化、长时间饱和、反转和停车后的输出均可解释；记录参考/实测 RPM、P/I、前馈、实际 PWM 和电压。

### R7：EKF 配置不可直接作为生产配置（P1，依赖C批）

证据：[ekf.yaml](../robot_ws/src/ldc_navigation/config/ekf.yaml) 仍为 `two_d_mode: false`、base_link，融合 odom 位置及 IMU 姿态；固件在 microros_runtime 第 62–64 行把 orientation、angular velocity、linear acceleration covariance 的首元素全设为 -1。即使发布了 gyro 数值，消息仍把该部分标成不可用。当前标准 bringup 未启用 EKF，并自行广播 odom→base_footprint。

方向：先明确 wheel-only 里程计与原始 gyro 的单位、轴、有效性、协方差和采样时间；再用二维 EKF 融合独立速度信息。协方差来自测量，不能把 -1 随意改为 0。切换时同步迁移 bringup、Nav2、网页、诊断和仿真，确保 odom→base 单一发布者。

验收：同一 bag 可复现，2m/360°原始标定达标，TF 无重复发布；保留无 EKF 的明确调试模式。

### R8：DWB 停止判定与当前速度范围不协调（P1，需复现整定）

证据：[ldc_nav2.yaml](../robot_ws/src/ldc_navigation/param/ldc_nav2.yaml) 第 80 行 max_vel_x=0.3，第 100 行 trans_stopped_velocity=0.25，当前 xy_goal_tolerance=0.25m、yaw_goal_tolerance=0.25rad。

影响：接近目标阶段可能过早进入停止/旋转相关判断，值得针对先前 RotateToGoal 无合法轨迹复现；仅凭参数不能断言是唯一根因。当前到达容差也不能支持“到位5cm”的性能宣称。

方向：固定地图、位姿与近目标场景，记录 DWB critic 原因；按实测停止速度、里程噪声和加减速能力整定。先解决数据质量，再缩容差；不要同时改所有参数。

### R9：自定义规划器碰撞保障不完整（P1，当前非默认规划器）

证据：[path_smoother.hpp](../robot_ws/src/ldc_navigation/include/ldc_navigation/path_smoother.hpp) 的 safe_update 只查候选点；未查前后整段、扫掠车体，也未验证短路径与最终 fallback。refinement 收敛量已修，不能推导为碰撞检查已完成。

[ArenaPlanner](../robot_ws/src/ldc_navigation/src/arena_planner.cpp) 第 168 行用自有 mutex 锁逐格读取；它不是 Costmap2D 自身提供的 getMutex（已核对本机头文件），也没有建立整个搜索/平滑期间不可变快照。

方向：锁 costmap 的共享写入契约后复制一次一致快照，统一 A* 邻边、平滑边段与实测 footprint 检查；验证原始路径后才允许 fallback，最终再验整条路径。默认 NavFn 和自定义 ArenaPlanner 分开验收。

### R10：构建与验证层次尚未闭合（P1）

证据：本机 `/home/<user>/.espressif/python_env/idf5.5_py3.12_env` 当前不存在；前次 IDF 调用已因此失败。现有宿主测试编译实际 MotionController，但替换电机/编码器，不执行 FreeRTOS motion_task、真实 PCNT 或 transport。

方向：建立统一检查入口和分层结果：算法→真实任务适配→完整固件→隔离 ROS 集成→离地→低速落地。锁定 SDK、依赖、配置，输出构建产物 SHA256 和可回滚版本，读取运行版本用于对照。安装环境与刷机分别处理。

验收：干净构建可复现；任务适配测试覆盖超期、命令过期、采样失败、取消/接管；实车验收有原始数据，不用绿色单测替代。

### R11：维护性应随功能重构，不先大拆（P2）

当前 `navigation_bridge.py` 772 行，App.vue 885 行，server.js 482 行。Bridge 同时负责授权、生命周期、任务与协议；已有测试中部分直接替换 requirements/sensors，这些用例不能证明真实就绪检查正确。

方向：先增加真实边界回归，再沿控制权/任务状态、数据有效性、协议边界拆模块；App 抽出遥控会话与目标交互逻辑。Bridge 请求队列目前无界，增加有界容量、忙碌响应与积压指标；优先级停止通道不能被普通请求排队淹没。

验收：行为保持可追踪、每个状态转换有测试，重复/乱序请求有明确结果。按职责减少复杂度，不以文件变短作为唯一目标。

## 下一轮具体安排

| 批次 | 可执行任务 | 交付物 | 放行条件 |
|---|---|---|---|
| A1 | 固定当前版本与配置；整理 SDK 环境缺项及完整构建入口 | 依赖清单、构建日志、产物/配置校验值、回滚方法 | 完整构建成功；缺环境明确失败，不伪通过 |
| A2 | 给主机加入旧包重放、无效 odom/scan、非法自动速度回归，再实现有效性分层 | 实际回调/门控测试、状态原因、修复 | 新数据正常，旧包和无效数据不维持授权 |
| B1 | 定义状态、事件、来源、会话、时间、动作ID契约 | 接口设计、模式兼容矩阵、状态转换表 | 四种 transport 有同一语义 |
| B2 | 运动任务单一所有者；安全事件与速度邮箱分离 | 固件安全实现、真实任务宿主适配测试 | 超时/急停/接管/复位/重连矩阵通过 |
| C1 | 传感器一致快照、采样时间、epoch映射、IMU有效性 | ROS数据契约、发布/丢帧诊断、bag | 采样冻结可识别，时间与数组语义可证明 |
| C2 | 总输出抗饱和、反转互锁、PCNT长期维护 | 控制日志、离地SOP与标定参数 | 仿真/台架通过后再低速落地 |
| D | EKF和TF统一迁移 | 新launch、配置、消费者迁移、回放对照 | 无双TF、无重复融合，原始标定可追溯 |
| E | DWB近目标复现、统一footprint、规划边段检查 | 固定bag/地图、碰撞回归、目标执行统计 | 单点稳定后再多点与复杂环境 |

A2 不依赖车端重刷，可先开发；完整端到端新鲜度验收依赖 C1。B、C 可以按清晰接口分批实现，但安全闭环未通过前不进入实车自动导航放行。没有实车时完成构建和仿真，不编造机械性能或交付日期。

## 验证结果与复现

本轮已经实际执行：

- `bash scripts/test_chassis_core.sh`：通过，MotionController 宿主断言和 6 个平滑器案例，ASan/UBSan。
- ROS环境下 Python unittest：25项通过。
- `npm --prefix web_console test`：39项通过。
- 真实 Bridge 的 on_scan/on_odom 函数最小探针：确认旧消息可以刷新接收计时。

日志与源码指纹在 `/home/<user>/ros-car-migration-audit/optimization-review-20261107/`。本轮未执行完整 IDF 构建、完整离线集成、实车动作或性能测量；不得把上述结果扩大解释为全链路验收。

复现普通回归：

```bash
cd /home/<user>/ros-car-migration
bash scripts/test_chassis_core.sh
source scripts/env.bash
ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m unittest discover -s scripts/tests -p 'test_*.py'
npm --prefix web_console test
```

参考：[仓库架构](LDC仓库架构.md)、[使用与演进计划](LDC使用与演进计划.md)。本报告是代码优化安排的较新依据；使用说明中的操作边界仍适用。
