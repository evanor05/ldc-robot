# ldc_qt_gui

ROS2 + C++ Qt5 上位机，用于显示地图、Nav2 costmap/path、雷达扫描点、机器人位置、摄像头图像、电池电压和电量，并发布速度控制、Nav2 目标点和初始位姿。

## 构建

```bash
cd /path/to/ros-car-migration
./scripts/build_host.sh
source scripts/env.bash
```

## 当前推荐：通过网页后端管理控制权

先运行唯一 Agent、bringup 和 `web_console/start_console.sh`。只观察：

```bash
bash /home/<user>/ros-car-migration/scripts/start_qt.sh --real
```

驾驶：先在网页停车并“释放控制权”，然后：

```bash
bash /home/<user>/ros-car-migration/scripts/start_qt.sh --real --enable
```

W/S 前后、A/D 左右，松开/空格/切出窗口停车。窗口标题和终端显示启用状态；默认 0.10 m/s。关闭窗口或 Ctrl+C 释放本客户端控制权。其他页面持有控制权时拒绝启动，不自动抢占；传感器失效、GUI断流或后端拒绝后退出，不自动恢复。

该入口将 Qt 输出重定向到独立私有话题，适配器通过 HTTP 携带 sessionId、controlId、sequence 请求现有后端，由后端发布 /cmd_vel。默认只读；`--enable` 才明确解除遥控锁定。仿真用 `--sim`，对应5174/18310，先启动隔离仿真。

受管理模式下地图点击不提交目标，定位/导航/建图和急停请用网页；原生任务启动按钮禁用。遥控模式禁用会阻塞GUI的 ROS话题查询，只读模式仍可查询。使用导航前退出Qt遥控，再在网页取得控制权。

## 旧原生启动（勿与网页同时驾驶）

```bash
ros2 launch ldc_qt_gui ldc_qt_gui.launch.py
```

也可以直接运行：

```bash
ros2 run ldc_qt_gui ldc_qt_gui
```

## 旧原生模式操作

- `W/S`: 前进/后退。
- `A/D`: 左转/右转。
- `Space`: 发布零速度停车（不是锁存急停）。受管理模式下急停请用网页“立即急停”。
- 可在右侧设置最大线速度和最大角速度。
- 地图左键拖拽发布 `/goal_pose` 导航目标，拖拽方向就是目标朝向。
- 点击“设置位姿”后，在地图左键拖拽发布 `/initialpose` 初始位姿。
- 地图右键或中键拖动平移，滚轮缩放，点击“地图复位”恢复自适应视图。
- “启动命令”面板可启动或停止 Micro-ROS、底层驱动、摄像头、建图和导航；面板内的建图与导航互斥。
- “ROS数据”面板可刷新所有话题，查看 `ros2 topic info -v`，并对任意话题执行一次 `ros2 topic echo --once`。
- 上位机启动的 launch 命令会放入独立 session；关闭窗口或点击停止时会清理整个进程组，避免 launch 子进程残留。

默认话题：

- 摄像头：`/camera/image_raw/compressed`
- 雷达：`/scan`
- 地图：`/map`
- 里程计：`/odom`
- 速度控制：`/cmd_vel`
- 导航目标：`/goal_pose`
- 初始位姿：`/initialpose`
- 电池：优先订阅 `/battery_state`，并兼容 `/voltage`、`/battery_voltage`、`/battery_percent`
- 路径：`/plan`、`/global_plan`、`/local_plan`、`/received_global_plan`、`/transformed_global_plan`
- Costmap：`/global_costmap/costmap`、`/global_costmap/costmap_raw`、`/local_costmap/costmap`、`/local_costmap/costmap_raw`
