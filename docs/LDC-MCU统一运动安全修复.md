# LDC MCU 统一急停、授权与控制源仲裁

日期：2026-11-07。此批先检查现有入口，再修改固件与必要的 MAVLink 主机兼容代码，完成宿主故障注入和编译。未刷写小车，未发送实车运动命令。

## 现有代码的问题与本批结果

| 原问题 | 处理方式 |
|---|---|
| 多任务直接读写 volatile 急停/手柄标志 | 移除这些控制标志，所有转换通过同一安全模块串行执行 |
| 初始默认允许运动，普通命令缺少来源授权 | 上电 Disarmed；显式授权后仅接受所选来源的新命令 |
| 不同入口解除急停时也能清掉硬件故障 | 外部急停与故障锁存分开；普通 enable 不清故障 |
| 接收/排队时刻不明确，旧命令可能续期 | 入口记录单调接收时间、授权代号；失权后待执行命令和输出失效 |
| 网络任务直接调用 ResetOdometry | 改为停稳且 Disarmed 时申请，由运动任务执行 |
| 控制计算期间发生急停，最后仍可能写旧输出 | 最终电机写入与 stop/arm/reset 使用同一互斥量，写入前再核对代号和时间 |
| 运动任务自己卡住时，任务内超时检查无法执行 | 增加独立高优先级软件监护，每10ms检查周期更新；超过40ms锁存计时故障并制动 |
| MCU需要ARM后，主机MAVLink桥仍只发速度 | 主机转发 /emergency_stop 为 MAVLink ARM/DISARM，并用真实ROS+PTY验证 |

## 状态与规则

状态值：0=Disarmed、1=Armed、2=Estop、3=Fault。

- **Disarmed**：禁止运动。reset 只进入此状态，不启动电机。
- **Armed**：显式授权一个来源，但没有新的有效命令时仍不输出运动PWM。
- **Estop**：外部急停锁存。普通速度、动作完成、手柄释放、disarm不能清掉它。
- **Fault**：低电压、过期电池、计时或相对动作IMU故障锁存；原因消失、轮速低于停止阈值后，必须显式 reset，再 arm。

旧接口的“解除急停/enable”保留为兼容入口：可解除外部Estop并授权当前网络来源，不能清除任何故障位。重复授权同一来源不会延长速度命令寿命；不能直接把另一个已授权来源挤掉。

来源值沿用内部枚举：2=MAVLink UDP、3=ESP-NOW、4=MAVLink UART、6=micro-ROS（UDP/UART均为此来源）、7=手柄。1和5为内部预留来源。

### 切换来源

采用显式交接，不让摇杆非零值自动抢占：

1. disarm 当前来源，立即制动并废弃其命令。
2. 等待轮速反馈达到停止阈值（左右轮绝对值均小于2 RPM）。
3. arm 目标来源，再发送新指令。

手柄中位或掉线会释放手柄控制；之后不会恢复旧导航，也不会凭再次拨动摇杆自行授权。需要明确重新arm。手柄与网络不存在隐式优先级竞争。

首次arm后尚未收到命令时允许等待操作，电机保持禁止；一旦开始速度流，超过300ms没有新有效命令会撤销授权，后续普通速度不自动恢复。

### 故障与时序

- 控制周期超过40ms或非正间隔：Timing故障。监护任务每10ms检查；这些是软件判定阈值，不是机械停稳时间保证。
- 电池样本必须有效且年龄不超过2s，才能授权。已收到有效电池后出现无效或过期样本会锁存故障。
- 电压低于6.4V持续100ms触发低电压故障；按持续时间判断，不把同一500ms电池样本重复计为多次独立采样。
- 相对动作依赖的IMU无效或超过200ms：锁存IMU故障；原因持续时reset拒绝。
- 速度模式0/3/6的命令租约为300ms；模式1/2动作总时限15s，且来源心跳间隔不得超过1.5s。MAVLink客户端应至少每秒2次发送HEARTBEAT。
- 停止和失权会使授权代号递增；电机写入时旧代号不再有效。电机写入与停止在同一优先级继承互斥量下执行，不在SMP自旋临界区里调用可能等待的LEDC API。

## 代码位置与输出边界

- [motion_safety.h](../firmware/chassis/main/control/motion_safety.h)：无RTOS依赖的状态机、来源/代号、时限、命令校验、动作去重。
- [motion_authority.cpp](../firmware/chassis/main/core/motion_authority.cpp)：线程同步、最新速度邮箱、停止/复位/授权、输出提交和独立监护。
- [motion_task.cpp](../firmware/chassis/main/tasks/control/motion_task.cpp)：唯一改变MotionController状态的任务；持续采样、故障输入、执行新命令、重置里程计和遥测。
- [motion_controller.cpp](../firmware/chassis/main/control/motion_controller.cpp)：先计算两轮PWM，再统一提交；授权拒绝时清除参考与PID缓存。

安全请求直接在互斥量内完成状态转换和制动，不通过可能被普通速度覆盖的队列。速度邮箱只保留最新值；待执行的有限动作不能被后来的速度覆盖，拒绝会返回失败。里程计复位单独排队，等待处理期间不会接受arm。

所有活跃运动入口（micro-ROS、MAVLink、手柄）均走统一模块；未启用的ESP-NOW入口也改为同一入口，并修正其原来失效的队列变量。未增加对ESP-NOW的默认启动或自动授权。

## 接口和兼容变化

### micro-ROS与网页

- `/cmd_vel` 保持 geometry_msgs/Twist，latest-value订阅深度改为1。
- `/emergency_stop` 保持 std_msgs/Bool：true=急停，false=上述兼容enable；普通速度绝不解锁。
- 没有自动刷写或自动授权。故障时可通过车端HTTP查询与复位；当前网页不会替操作者自动清故障，也没有新增MCU故障状态的直接ROS反馈订阅。

### 车端HTTP

GET `/api/status` 新增 `motion_safety`：state、owner、generation、faults、active_faults、reason。原 emergency_stop/motion_busy 字段保留，但由安全模块快照生成。

POST `/api/estop` 保留 `state=true/false`，新增表单action：

| 参数 | 行为 |
|---|---|
| `action=stop` | 急停，任何来源可用 |
| `action=disarm` | 全局撤销授权；不会解除Estop/Fault |
| `action=reset` | 条件满足后清故障/急停，保持Disarmed |
| `action=arm&source=network` | 授权当前配置的网络来源 |
| `action=arm&source=gamepad` | 授权手柄 |
| `action=arm&source=espnow` | 授权ESP-NOW（本批不默认启动其任务） |

处理前完整读取有界请求体，分片/截断请求不会提前执行arm或误用默认来源。拒绝返回HTTP 409，ok=false；响应含实际state/owner/generation/faults/active_faults及兼容emergency_stop字段。action与state同时提供时优先处理action。

故障位：1=Timing、2=BatteryLow、4=BatteryStale、8=ImuStale。reason值：0上电、1启用、2急停、3释放、4超时、5来源失联、6计时、7低电压、8电池失效、9IMU失效、10非法命令、11完成、12复位。

示例仅供后续台架使用，本批没有执行这些车端请求：

```bash
curl -fsS 'http://<小车IP>/api/status'
curl -fsS -X POST 'http://<小车IP>/api/estop' -d 'action=stop'
# 原因已消失、确认停稳后，分两步执行：
curl -fsS -X POST 'http://<小车IP>/api/estop' -d 'action=reset'
curl -fsS -X POST 'http://<小车IP>/api/estop' -d 'action=arm&source=network'
```

### MAVLink

ARM/DISARM命令继续使用MAV_CMD_COMPONENT_ARM_DISARM；ARM是否受理由安全模块决定，拒绝返回MAV_RESULT_DENIED。主机ldc_mavlink_bridge新增订阅 `/emergency_stop`，只转发显式操作，不在启动或串口重连时重放ARM。

为防有限动作重复执行，收紧两项接口：

- 模式1位置动作：SET_POSITION_TARGET_LOCAL_NED 的 time_boot_ms 必须为非零、递增动作ID；同一来源、同一动作类型的旧ID不因重新arm而变新。
- 自定义MAV_CMD_USER_2相对动作：param1为距离mm，param2为角度deg；param7新增必填递增动作ID，整数范围1..16777215。旧客户端param7=0将被拒绝，不会偷偷生成ID再执行。

位置与相对动作分开维护已消费ID，均保留至MCU重启。客户端重启后仍需使用更大的ID。模式1完成前可由disarm/stop取消，否则受15s上限和来源心跳限制，不宣称本批新增了位置动作到位结果协议。

速度与动作的有限数、差速轴向、限幅在统一入口再检查；MAVLink目标system/component不匹配会拒绝。单轮调试时另一轮先停；双轮模式还检查换算后的角速度不能超过1.5rad/s。

## 验证与复现

已实际完成：

- 原有控制器、平滑器、样本时间与真实IMU任务测试继续通过，ASan/UBSan。
- 新安全核测试：默认禁止、显式reset/arm、故障拒绝恢复、来源冲突、旧代号、晚到命令、重复动作、100轮停止/切换软件循环。
- 编译并运行真实motion_task、MotionController和motion_authority，仅替换硬件/FreeRTOS适配：65个控制周期，计算中急停、计算超期、低电压、过期电池、IMU失败、里程计复位和独立监护；另测试并发停止与输出提交，以及HTTP分片、截断、超长请求体读取。
- SDK5.5.3 ESP32-S3完整底盘构建通过，最终评审修正后增量构建通过；最终源码与固件产物SHA256校验保留。
- ldc_mavlink_bridge构建通过；真实ROS Bool消息经实际桥接进程输出到PTY，核对ARM/DISARM目标与param1，并确认没有启动/定时自动ARM。
- Python回归32项、网页回归40项通过。

```bash
cd /home/<user>/ros-car-migration
bash scripts/test_chassis_core.sh
source scripts/env.bash
ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m unittest discover -s scripts/tests -p 'test_*.py'
npm --prefix web_console test
bash scripts/build_host.sh --packages-select ldc_mavlink_bridge
# 新终端重新source后，测试编译好的桥接程序（只打开脚本创建的PTY）：
source scripts/env.bash
ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 scripts/tests/mavlink_estop_integration.py
bash scripts/build_chassis.sh --output "/tmp/ldc-mcu-authority-$(date +%Y%m%d-%H%M%S)"
```

证据目录：`/home/<user>/ros-car-migration-audit/mcu-authority-20261107/`。最终固件使用 `firmware-final/`，不要把早期 `firmware-build/` 当作最后版本。测试采用Domain198和模拟硬件/PTY，无实车动作；未重复运行与本批MCU实现无直接绑定的完整Nav2仿真。

## 实车放行边界

软件监护需要调度器可运行、输出互斥量和LEDC可返回；不能覆盖中断永久关闭、底层驱动永久阻塞、供电和机械故障。仍须验证独立电源切断/驱动禁止、复位时引脚状态、制动电平、带载停车距离与实际监护时延。100轮软件状态转换不是100次实车急停。

授权代号保护MCU内部待执行命令与输出。现有无头Twist/Bool及MAVLink来源表示通信渠道，不能证明跨网络排队消息的原始年龄，也不是远端用户身份认证；本批不宣称已经实现完整网络级重放防护。后续若要求该能力，应整体升级为带会话、源时间与序号的线上契约。

## 备份与逐文件回滚

修改前文件在审计目录 `before/`，逐文件清单与校验值在 `changes.json`，本批独立差异在 `changes.diff`。

先停车、退出相关控制进程，再按相同相对路径恢复。例如：

```bash
cp /home/<user>/ros-car-migration-audit/mcu-authority-20261107/before/firmware/chassis/main/tasks/control/motion_task.cpp /home/<user>/ros-car-migration/firmware/chassis/main/tasks/control/motion_task.cpp
```

本批MCU消息结构、安全模块、任务和适配入口必须按清单成组回滚，不能只恢复一个任务。新增文件可保留停用；恢复原CMakeLists和测试入口后不会被原固件引用。MAVLink主机源文件、CMakeLists、package.xml同时恢复后重新构建该包；不要git reset，以免丢失其他既有修改。车上固件未刷写，无需回滚车端Flash。
