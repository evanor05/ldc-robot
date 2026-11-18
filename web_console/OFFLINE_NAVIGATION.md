# 建图与导航：离线运行及验收

在项目根目录 `/home/<user>/ros-car-migration` 操作。需要已有的 ROS Lyrical、项目工作空间和网页 `node_modules`，入口不安装系统软件。

## 启动仿真网页

```bash
cd /home/<user>/ros-car-migration
scripts/offline_console.sh
```

打开 <http://localhost:5174/>，标题显示“仿真”。终端会显示本次地图和日志目录；`Ctrl+C` 结束并清理该次仿真服务。

- 仿真固定使用 ROS Domain 197、API 18310、Foxglove 8766、网页 5174；实车默认 Domain 0、API 3000、Foxglove 8765、网页 5173。
- 模拟雷达、里程计、TF 和底盘响应速度指令；实际运行 GMapping、AMCL 和 Nav2。使用真实时间，无须 Gazebo。
- 测试地图放在独立目录，不进入实车 `maps/`；仿真不发现实车摄像头。
- 端口已被占用时直接报错，不接管其他实例。
- 启动入口会重新构建 `ldc_navigation` 包，使安装目录中的 launch 与当前代码一致。

网页流程：开始建图 → 启用遥控并移动 → 保存并停止建图 → 选择新地图开始导航 → 设置初始位置和朝向 → 等待“导航就绪” → 双击目标。多点模式先添加航点，再点击“执行航点队列”。

仿真初始位姿为 `(0, 0, 0)`；手动移动后，应在地图上按实际位置重新设置。正常默认导航容差由既有 Nav2 参数决定，成功不代表位置误差为零。

## 自动验收

```bash
cd /home/<user>/ros-car-migration
npm --prefix web_console test
npm --prefix web_console run test:integration
npm --prefix web_console run build
source scripts/env.bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -s scripts/tests -v
scripts/offline_console.sh --test --output /tmp/leap-nav-acceptance
```

最后一项运行完整软件闭环并退出，结果为输出目录中的 `acceptance.json`，详细 ROS 日志为 `backend.log`。使用普通用户执行，以验证无写权限时的保存失败与重试。

覆盖：保存失败保留建图、重试、地图文件完整性、重复停止、新图排序；定位前拒绝目标；真实 Action 单点、多点及模拟底盘实际位置；取消确认、手动接管、急停、心跳失联、传感器断流、恢复不续跑、不可达目标、控制节点退出、后台重启及保存信息恢复。单元测试另覆盖目标拒绝、旧回调隔离、低剩余距离不提前切点、速度会话失效、地图旋转/缩放/平移点击与拖拽朝向。

## 接口与控制行为

- `GET /api/navigation_status` 返回进程状态、`phase`、`ready`、缺失条件、错误、地图、控制模式、目标 ID、Action 状态、反馈和队列。进程启动不等于就绪。
- `POST /api/navigation/goals`：`{"points":[{"x":0.8,"y":0,"yaw":0}]}`，最多 100 点。等待首个目标实际受理后返回；只在 Action 成功后推进下一点。
- `POST /api/navigation/initial_pose`：`{"x":0,"y":0,"yaw":0}`。AMCL 更新并产生有效地图 TF 后开放目标。
- `POST /api/navigation/cancel`：清空队列，锁定运动，等待取消的终态确认；取消未确认时不会允许遥控接管。
- `POST /api/control/manual_enable`：先取消导航并确认，再返回新的 `controlId`。
- `POST /api/navigation/velocity`：携带 `controlId`、递增 `sequence`、`linearX`、`angularZ`。旧会话或乱序指令拒绝；手动上限保持 0.4 m/s、1.2 rad/s。
- `POST /api/navigation/heartbeat`：可见且桥接连接的网页每 500 ms 发送；超过 2 秒无心跳会锁定并取消任务。手动速度超过 300 ms 无新指令则归零。
- `POST /api/navigation/estop`：锁定、清队列、取消目标并发布急停。重新启用需明确操作；新导航目标提交也属于明确启用。
- `POST /api/stop_navigation`：先取消确认，再结束 Nav2 进程；`POST /api/start_navigation` 只接受地图目录中的地图。
- 网页的运动和目标操作走后端；Foxglove 用于遥测。后端从 `/cmd_vel_auto` 接收 Nav2 输出，统一发布 `/cmd_vel`。

独立运行 ROS 导航 launch 默认继续输出 `cmd_vel`，并使用原 Nav2 生命周期管理器。控制台显式传入 `managed_by_console:=true` 和 `cmd_vel_output:=cmd_vel_auto`：后端顺序配置/激活地图与 AMCL，等待用户初始位姿，再激活其他导航节点。每次转换都通过实际 GetState 结果确认，服务回复丢失时重查并重试，避免无限等待。

控制台模式关闭节点的配对 bond 心跳，由同一后端检查服务存在、节点实际生命周期（最长 6 秒过期）、传感器与动态 TF，并掌握唯一速度出口；节点退出触发整栈关闭。后端失联或传感器断流会先锁定运动，不自动恢复。等待用户初始定位不限时，真正的节点启动阶段超时为 60 秒。

网页桥接只开放遥测话题和连接图，运动和目标都由后端处理，避免反复枚举隐藏 Action/服务类型。仿真采用 localhost 发现及 UDPv4。测试过程中曾在系统 Fast DDS 3.6.2 的 TypeLookup 回调遇到崩溃；没有升级系统库，已增加节点退出时的整体关闭与完整子进程清理，最终验收在当前配置下通过。这不等于修改了 Fast DDS 源码。相关机制参考：[Fast DDS 官方环境配置](https://fast-dds.docs.eprosima.com/en/stable/fastdds/env_vars/env_vars.html)、[ROS 2 发现范围配置](https://github.com/ros2/ros2_documentation/blob/rolling/source/Get-Started/Configuring-ROS2-Environment.rst)。

## 地图保存与重启

保存先写入同一目录下的临时目录，确认 YAML/PGM 非空后发布，YAML 最后出现；同名文件不覆盖。失败清理临时输出并允许重试。保存成功才停止 GMapping，并等待进程退出后允许切换导航。

`.console-last-saved.json` 保存最近一次地图及时间，后台重启后恢复显示；重启不恢复运动或旧目标。`live_scan_20261103_1733` 保留为既有初始扫描，未恢复的历史扫描不会被伪装成新图。

## 回滚和实车边界

本轮修改前备份在 `/home/<user>/ros-car-migration-audit/offline-navigation-20261103/before/`。停止本轮控制台和仿真后，按相对路径逐文件恢复对应备份，重新构建 `ldc_navigation` 并重启原网页入口；新增模块可保留不用。不要 `git reset`，以免清除用户原有修改；不要删除地图目录。

实车入口仍为 `bash /home/<user>/ros-car-migration/web_console/start_console.sh` 和 <http://localhost:5173/>。没有实车时显示等待雷达和里程计，不能启用运动。

原入口会检查端口占用，重复启动直接报错；Vite 使用严格端口，不会悄悄换到仿真端口。退出时清理完整项目子进程树。

此测试确认软件流程及二维模拟底盘闭环。实车轮向、里程计尺度、雷达外参、定位稳定性、机械惯性和真实避障停车距离，需要实车恢复后验收。
