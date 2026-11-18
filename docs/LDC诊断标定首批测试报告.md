# LDC 导航诊断与标定首批测试报告

日期：2026-11-10。范围：导航定位健康、诊断/配置、网页标定会话、只读采集、隔离回放和桌面入口。

## 结论与边界

第一批软件已完成；实车不在线，未刷写固件、未改车体/动力参数、未切换导航默认算法。真实尺寸、固件版本、里程计尺度、外参、停车距离及导航成功率仍待实车验收。EKF、slam_toolbox、Smac、MPPI、灯色和交通控制均未在本批启用。

使用方法见 [诊断、标定与回放](LDC诊断标定与故障回放.md)。日常双击桌面“标定向导与诊断”；仿真保留在独立入口。

## 测试结果

证据目录：`/home/<user>/ros-car-migration-audit/navigation-diagnostics-20261110/`。

| 检查 | 结果 | 证据 |
|---|---|---|
| Python 回归 | 62 项通过 | `python-tests-final.log` |
| 网页/后端单元与组件检查 | 48 项通过 | `web-tests-final.log` |
| 原有后端集成 | 3 项通过：电池话题、地图列表、仓库相对路径 | `backend-integration-final.log` |
| 前端生产构建 | 通过，35 个模块 | `frontend-build-final.log` |
| 完整隔离导航流程 | 22 项通过，真实 GMapping/AMCL/Nav2，合成传感器/运动响应 | `offline-acceptance/acceptance.json` |
| ROS CDR 采集/回放 | 通过，源 Domain 195，回放 Domain 196 | `capture-acceptance-final/acceptance.json` |
| 变异验证 | 翻转 map frame 校验后，9 项定位测试失败；原文件逐字节恢复，最终 Python 回归通过 | `mutation-test.log` |
| 桌面脚本 | bash 语法、desktop-file-validate 通过；trusted=true | `delivery-checks.log` |
| 网页交互 | 建立会话、保存合成测量、生成候选、关联独立复测、刷新恢复、采集开始/结束已操作验证 | `ui-check/` 内会话、报告、采集状态及日志 |

网页测试数据明确标为合成数据，全部放在审计目录，没有进入实车标定目录。已重新打开网页并验证保存的独立复测报告可恢复，最终截图为 `ui-check/calibration-final.jpg`。截图中的轮径、误差均为合成输入的结果，不是实车标定数据。桌面启动器沿用已有终端分派器，本轮检查配置与信任标记，未再次用桌面图标实际启动实车栈。

### 定位与控制

覆盖无效 frame、时间戳、非有限位置、非法四元数、协方差、旧/重复定位、初始定位会话、静止 AMCL 不更新、位移/转角触发更新、大幅跳变、时间回拨及里程计时间无法配对。

真实 Bridge 回调测试验证定位故障会取消执行目标并锁定；恢复后不重启旧队列。完整 Nav2 隔离验收还覆盖建图保存失败重试、单点/多点 Action 成功、取消、人工接管、急停、失权、断流、旧传感器包、不可达、控制节点退出和后台重启。

弹窗阻止 WASD 传播，并在 App 层保留独立的弹窗状态检查，覆盖按钮被禁用后焦点回到 body 的情况。Esc 急停仍有效。

### 采集与回放

最新真实 DDS 采集为 5.01 秒：odom 96、IMU 95、scan 95、初始位姿 95、速度 95、JPEG 5。磁盘产物 431,023 字节，小于 4MiB 测试上限。回放观察者收到 odom 96、JPEG 5、观察速度 95；原 `/cmd_vel` 和 `/initialpose` 消息数均为 **0**。

其他回归覆盖容量达到后停止、旧记录不覆盖、`Inf/NaN` 的 CDR 保真、报告最多 120 快照、事件队列拥塞不跳游标、写盘故障明确失败、完整性哈希校验、后台中断状态、启动中取消、重复启动拒绝和主动停止不报故障。采集器只订阅 ROS；相机复用已有原图话题。

### 标定结果可信度

新会话无预填真值；尺寸/CSV 修改有修订，报告有来源哈希。独立复测关联原报告但不复制旧数据，计算新试验的原始误差；测试专门构造“拟合残差为零、原始误差 10%”的数据，确认不能被当作达标。1% 边界浮点误差已修复。即使数值达标，也继续提示样本量不足或实际版本未核对。

## 复现命令

```bash
cd /home/<user>/ros-car-migration
source scripts/env.bash
export ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -m unittest discover -s scripts/tests -p 'test_*.py'
npm --prefix web_console test
npm --prefix web_console run build

# 仅开发使用，输出目录必须是新的
python3 scripts/tests/diagnostic_integration.py --output /tmp/ldc-capture-check-new
bash scripts/offline_console.sh --test --output /tmp/ldc-navigation-check-new
```

后端集成可用 `ROBOT_SIMULATION=1 LDC_CALIBRATION_DIR=/tmp/ldc-backend-check-new npm --prefix web_console run test:integration`，使用 18300 端口。不要与其他同端口测试同时运行。

## 五项代码复核

| 维度 | 结果 |
|---|---|
| 正确性 | 定位故障取消、不自动恢复；独立误差不混入拟合；采集退出与取消状态经过回归 |
| 可读性 | 定位、实际参数、标定会话和采集各有独立模块，原桥接器只接入检查与状态 |
| 架构 | 原 Action/控制权管理继续负责运动；标定与回放不写线上参数；保留标准导航配置 |
| 输入与数据保护 | 会话标识/路径校验、拒绝符号链接、结构化子进程参数、修改请求 Origin 检查、文件哈希、备份无全局重置 |
| 性能 | 一帧/一快照队列、单次时长/容量限制、有限报告窗口、参数白名单与 3 秒轮询；高频采集使用 Best Effort，不能视为无损记录 |

本批未新增系统依赖或 npm 依赖。现有 Python/OpenCV/numpy/YAML、rclpy/rcl_interfaces 等环境已用于测试。未重新执行未改动的 MCU 固件构建或 30 分钟 YOLO 稳定性试验；本批软件测试不替代这些历史证据或后续实车验证。

## 备份与逐文件回滚

本批备份以修改前的工作区为基准，保留原有未提交改动；没有用 Git HEAD 重置文件。

- `before/`：8 个修改前的仓库文件。
- `before-desktop/`：4 个修改前的桌面文件。
- `changes.json`：每个文件的绝对路径、修改前/后哈希、备份位置和新增/修改类别。
- `changes.diff`：仅本批变动。
- `source-after.tar.gz`：本批交付源码/文档快照。
- `rollback.py`：默认仅预览；`--apply` 才改文件；拒绝覆盖交付后又改动的文件。

```bash
# 预览并核对当前文件、备份的哈希
python3 /home/<user>/ros-car-migration-audit/navigation-diagnostics-20261110/rollback.py

# 只预览一个文件；也支持 desktop/... 的桌面文件标识
python3 /home/<user>/ros-car-migration-audit/navigation-diagnostics-20261110/rollback.py \
  --file scripts/navigation_bridge.py

# 停止相关控制台/采集后，再明确执行整批回滚
python3 /home/<user>/ros-car-migration-audit/navigation-diagnostics-20261110/rollback.py --apply
```

单文件恢复可以加 `--file 文件标识 --apply`，但导入模块和调用方应成组回滚，避免半套版本。回滚前会另存当前文件；本批新增文件移到审计目录的回滚保留区，不永久删除。标定会话、地图、采集记录不在回滚目标中。回滚后若使用生产 `dist`，重新执行前端构建；开发页面刷新即可加载恢复的源码。
