# LDC 超声波采样有效性修复与验收

日期：2026-11-11。本批只改固件与固件宿主测试，未刷机、未操作实车、未修改默认导航算法、速度或网页。

## 背景

`publish_ultrasonic` 原来每次发布都 `xQueuePeek` 最新缓存，并用发布时刻的 `stamp_ms` 给 `sensor_msgs/Range` 盖章。采样任务每 100 ms 覆盖一次缓存，通信任务按 100 ms 慢周期重复读取同一个样本时，会把它伪装成新的采样；采样任务停住而网络仍正常时，主机无法从消息时间判断数据已经冻结。

另外两个事实必须同时处理：

- `UltrasonicMsg` 被 `UdpTelemetryPacket` 引用，是旧 UDP 遥测的二进制布局，不能直接在它里面增加 `SampleMetadata`。
- `/ultrasonic` 的发布者初始化此前被注释掉，原因注释是“硬件为 -1 无效、静态库最多 5 个发布者”。核对 `app-colcon.meta` 后确认 `RMW_UXRCE_MAX_PUBLISHERS=6`，且预编译 `libmicroros.a` 的 `custom_publishers` 符号大小为 `0x438`（6 × 0x180），足够覆盖六个发布者。

## 改动

### 1. 内部采样元数据与旧线协议隔离

`main/msg/ultrasonic_msg.h`：

- `UltrasonicMsg` 保持 5 字节 packed 布局（`float distance_cm` + `bool is_obstacle_detected`），不增加字段；`sizeof(UltrasonicMsg)==5` 已有宿主静态断言。
- 新增内部队列载荷 `UltrasonicStateMsg { UltrasonicMsg reading; SampleMetadata sample; }`，只用于 FreeRTOS 队列，不进入 UDP 结构。

`main/app_runtime.cpp` 的 `ultrasonic_state` 队列按 `sizeof(UltrasonicStateMsg)` 创建。

### 2. 采样任务记录真实完成时间

`main/tasks/sensors/ultrasonic_task.cpp`：

- `GetDistanceCm()` 返回后立即读取 `esp_timer_get_time()`，以回波结束时刻作为 `sample_time_us`，不再使用发布时刻。
- `sample.valid` 要求距离有限且大于 0；失败时 `distance_cm=-1`、`valid=false`。
- 只有成功采样才推进 `sequence` 与 `sample_time_us`；失败保留上一次有效样本的序号和时间，明确表示“没有新样本”。
- 所有消费者统一读取 `UltrasonicStateMsg`，并按 `sample.valid` / `sample_time_us` 判断。

### 3. micro-ROS `/ultrasonic` 发布

`main/tasks/network/microros/`：

- 恢复 `sensor_msgs/Range` 发布者初始化，发布者总数与 `app-colcon.meta` 的 6 个槽位一致。
- 新增纯策略头 `ultrasonic_publish.h`：拒绝无效、非有限、超出 `0.02–4.0 m`、超过 300 ms、旧会话和已发布过的样本。
- 使用 `SampleClock` 将 `sample_time_us` 映射为 ROS epoch 时间，使用 `SamplePublishGate` 去重；只有 `rcl_publish` 返回成功才推进发布游标，失败可用同一源时间重试。
- `publish_ultrasonic` 在发布者未初始化时直接返回，避免此前对零初始化发布者的重复告警。

### 4. 其他消费者

- MAVLink `DISTANCE_SENSOR`：改为发送 `sample.sample_time_us/1000` 作为 `time_boot_ms`，并用 `esp_timer_get_time()` 检查 300 ms 新鲜度；无效或过期样本不发送，不再反复发送同一缓存。
- 配网页面 `/api/status` 的 `ultrasonic.valid`：由“队列存在”改为“样本有效且不超过 1 s”，`distance_cm` 来自 `reading.distance_cm`。
- `firmware/chassis/doc/protocols.md`：补充 `/ultrasonic` 只在采样有效时发布、同一采样不重复盖章、当前未接入 Nav2 代价地图。

## 验证

| 检查 | 结果 |
|---|---|
| 宿主核心回归 `bash scripts/test_chassis_core.sh` | 通过，包含新增 `ultrasonic_sampling_test` 与 `ultrasonic_publish_test`；ASan/UBSan 开启 |
| 新增采样测试 | 4 次采样：有效→失败→失败→有效，序号 1→1→1→2，失败不推进时间，距离 -1 |
| 新增发布策略测试 | 未同步不发布、重复样本不发布、新样本可发布、无效/越界/超 300 ms/旧会话全部拒绝 |
| 旧 UDP 线协议 | 修改前后 `sizeof(UltrasonicMsg)=5`、`sizeof(UdpTelemetryPacket)=956`、`offsetof(UdpTelemetryPacket,ultrasonic)=200` 完全一致 |
| 完整 ESP-IDF 构建 | 通过；ESP32-S3，构建目录 `/tmp/ldc-firmware-ultrasonic-20261111-1120`，产物 `ldc_chassis.bin`、`merged-binary.bin`、`ldc_chassis.elf` 及 SHA256 保存在审计目录 `artifacts-sha256.json` |
| Python 回归 | 66 项通过 |
| 变异验证 | 临时副本分别破坏“仅成功采样推进序号”和“重复样本门控”，新增测试均以断言失败检出；工作树源码未被变异修改 |
| 实车/硬件 | 未执行。超声波硬件最后已知为 -1 无效，本批未上电、未刷机；`/ultrasonic` 是否真正有数据需实车恢复后确认 |

## 五维复核

- **正确性**：采样完成时间取自 `GetDistanceCm()` 返回后的单次 `esp_timer_get_time()`；失败样本不推进序号和时间；发布门控覆盖未同步、无效、越界、过期、旧会话和重复样本；MAVLink 与配网状态同样检查有效性/年龄。
- **可读性**：采样任务与发布策略分离为 `ultrasonic_task.cpp` 和 `ultrasonic_publish.h`，`UltrasonicStateMsg` 明确标注为内部队列载荷，旧线协议注释保留在 `UltrasonicMsg` 上。
- **架构**：保持 `UltrasonicMsg` 的 UDP 布局不变，元数据只进入内部队列；发布策略为无副作用的纯函数，便于宿主测试；未把超声波塞进 `ObstacleLayer`，也未提前改变 Nav2 配置。
- **安全**：无效/过期数据不发布，避免把缓存值伪装成新鲜观测；没有新增外部输入路径、密钥或权限；配网 JSON 的 `valid` 不再仅表示“队列存在”。
- **性能**：每 100 ms 一次采样、每 100 ms 一次慢发布，队列仍为长度 1 的覆盖邮箱；无新增动态分配、无阻塞循环；策略判断只做常数级浮点检查。

## 边界与后续

- 本批没有把超声波接入 `RangeSensorLayer`，也没有修改默认 `ldc_nav2.yaml`。原因：`RangeSensorLayer` 必须是独立层，不能塞进只接受 `LaserScan`/`PointCloud2` 的 `ObstacleLayer`；同时 HC-SR04 的“无回波超时”与“传感器断线”在当前驱动上不可区分，直接把超时映射为 `max_range` 清障会制造虚假安全。需先完成实车传感器健康检查、安装位置/视场核对，再单独做代价地图接入和避障验收。
- `is_obstacle_detected` 仍是旧 UDP 字段；本批没有定义障碍距离阈值，保持 `false`，ROS `/ultrasonic` 只使用 `range`。
- 轮距、轮轴参考点、footprint、停车距离、DWB `trans_stopped_velocity` 等实车标定不在本批范围；未改任何速度或动力参数。
- EKF、MPPI、Smac、交通灯决策和 YOLO 控制均未启用。

## 回滚

审计目录：`/home/<user>/ros-car-migration-audit/ultrasonic-sampling-20261111-095000/`。

`rollback.py` 默认只预览，`--apply` 才恢复；恢复前校验当前文件与备份哈希，拒绝覆盖交付后又被修改的文件。回滚后需重新构建并烧录固件（本批未烧录，车端 Flash 无需处理）。

```bash
python3 /home/<user>/ros-car-migration-audit/ultrasonic-sampling-20261111-095000/rollback.py
python3 /home/<user>/ros-car-migration-audit/ultrasonic-sampling-20261111-095000/rollback.py --apply
```
