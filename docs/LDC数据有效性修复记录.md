# LDC 数据有效性修复记录

日期：2026-11-07。本批修复“缓存数据假新鲜”，覆盖 MCU 采样、micro-ROS 发布、主机门控与网页状态。未刷车、未进行实车运动。

## 改动与行为

### 固件

- 内部 MotionMsg、ImuMsg 增加 SampleMetadata：单调采样时间、序号、有效标识。FreeRTOS 队列按 sizeof 创建，ROS 线上类型与话题不变。
- 运动任务每次成功更新状态时记录采样时刻。IMU 必须 Update 返回成功且关键数值有限才推进样本；读取失败或出现 NaN 时标记无效，不给缓存盖新采样时间。
- IMU 无效或超过 200ms 时，不继续执行依赖 IMU 的相对动作，停止后不会自动重启该动作。
- micro-ROS 为本次会话建立单调时间→ROS epoch 映射；未同步时暂停状态发布。odom/IMU 使用样本时间盖章，超过 300ms、无效、旧会话样本不发布；同一样本不重复发布。发送失败可用原始时间重试，只有成功发送才推进发布游标。
- 本批没有改变 IMU covariance 语义、切换 EKF，或重建 LaserScan 首束时间。sample.valid 表示这次采样有效，不代表姿态估计或噪声已标定。

### 主机

- 新的 sensor_health 模块校验 odom 的坐标系、有限数、四元数，以及 scan 的坐标系、角度/数组一致性、量程和扫描元数据。
- 全 Inf 扫描允许作为“无回波”正常帧；全 NaN/无可用束拒绝。正常静止数据只要实际时间推进便可通过。
- 有效样本年龄上限 1.5s，未来容差 0.25s；重复/乱序时间戳不续期。传输收包、有效样本更新和 ROS 采样时间分开统计。导航桥接默认 `ROBOT_SCAN_MAX_AGE=1.5`，仅调试时可显式放宽，生产不得用 3s 静默覆盖。
- GET /api/navigation_status 的 checks.lidar / checks.odometry 保留 ok/detail/ageSeconds，新增 receiptAgeSeconds、sampleAgeSeconds。原接口路径和已有字段不变。
- 自动速度校验六轴有限数，仅允许差速轴向，硬边界 |vx|≤0.4m/s、|wz|≤1.2rad/s；非法输入不转发，锁定并取消导航。该上限是接口硬边界，不是新的导航速度档。

### 网页

- 在线状态同时需要本地雷达/odom 收包正常和后端对应检查通过；后端状态超过 2s 不更新则视为不可用。
- 失效时隐藏缓存的电量、odom 和扫描，禁用目标和遥控；恢复不会自动启用。
- 后端就绪面板显示具体失效原因；不再仅凭 WebSocket 接通判断小车正常。

## 验证结果

| 检查 | 结果与边界 |
|---|---|
| 修复前新增主机回归 | 7项中5失败、1错误，证明旧包/无效数据/非法速度保护不足 |
| 修复后 Python 测试 | 32项通过，包含真实 Bridge 回调和取消/锁定流程 |
| 网页测试 | 40项通过，覆盖后端拒绝与状态轮询失效 |
| 前端生产构建 | 通过 |
| C++ 宿主测试 | 控制器、平滑器6案例、样本时间映射/发布游标、真实 imu_task 适配；ASan/UBSan 通过 |
| ESP-IDF 底盘构建 | SDK 5.5.3、ESP32-S3，隔离副本构建通过；生成 bin/elf/merged-binary 和 SHA256 |
| 隔离集成验收 | 22项通过，真实 GMapping/AMCL/Nav2；新增旧样本持续重放、TF仍更新时取消锁定和恢复不续跑 |

固件构建使用仓库现有 managed component 与依赖缓存，不等于所有第三方依赖从网络全新拉取重建。编译过的 main 源码指纹与当前工作树逐文件一致。真实编码器、IMU硬件、网络延迟、时间漂移和停车距离仍须实车验收。

## 使用与复现

源码根目录为 `/home/<user>/ros-car-migration`。主机改动在正常停止任务、重启控制台后生效；网页 Ctrl+Shift+R。固件端修复需要后续刷入并进行离地验证，本批未刷写。

旧固件如果把缓存重新盖成新时间，主机无法单独证明实际采样仍在运行；要获得完整保护，必须配套新固件。新主机不会因仅检测到旧固件而擅自刷写。

```bash
cd /home/<user>/ros-car-migration
bash scripts/test_chassis_core.sh
source scripts/env.bash
ROS_DOMAIN_ID=198 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m unittest discover -s scripts/tests -p 'test_*.py'
npm --prefix web_console test
npm --prefix web_console run build
bash scripts/offline_console.sh --test --output "/tmp/ldc-data-check-$(date +%Y%m%d-%H%M%S)"
```

新增固件构建入口只构建隔离副本，不安装系统软件、不刷机。输出目录必须在仓库外且不存在：

```bash
bash /home/<user>/ros-car-migration/scripts/build_chassis.sh --output "/tmp/ldc-firmware-$(date +%Y%m%d-%H%M%S)"
```

默认使用已有 `/home/<user>/esp/esp-idf-v5.5.3`、`/home/<user>/esp/.espressif`、`/home/<user>/esp/idf-python-env`。可分别用 LDC_IDF_PATH、LDC_IDF_TOOLS_PATH、LDC_IDF_PYTHON_ENV 覆盖。之前构建失败是默认工具目录不匹配，这次通过明确指定现有环境解决，没有重装 SDK。

完整证据：`/home/<user>/ros-car-migration-audit/data-contract-20261107/`。包含修复前失败测试、最新回归日志、offline-acceptance/acceptance.json、firmware-build/build.log、产物校验值、修改前备份与本批变更清单。

## 回滚

先停车、停止导航并退出本轮控制台。使用审计目录的 changes.json 查看 changed / added 文件；before/ 保留每个改动前文件，按相同相对路径逐文件复制回项目。示例：

```bash
cp /home/<user>/ros-car-migration-audit/data-contract-20261107/before/scripts/navigation_bridge.py /home/<user>/ros-car-migration/scripts/navigation_bridge.py
```

固件消息头、生产任务和发布任务需成组恢复，避免内部结构不一致；主机与网页也应按本批清单成组恢复。新增文件可以保留但停用，不要 git reset。未刷固件，因此没有修改车上 Flash，回滚源码无需处理车端。

## 后续

下一批继续 MCU 统一急停/授权/来源状态机与真实运动任务故障矩阵，随后推进执行器总输出抗饱和。EKF与TF交接、雷达首束/逐束时间、MAVLink全链路采样时间、长期漂移校验和完整路径扫掠碰撞仍按审查报告推进，本批没有宣称完成这些内容。
