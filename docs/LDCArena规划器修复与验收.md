# ArenaPlanner 修复与验收

本批修复自研候选规划器的地图并发读取、线段碰撞和终点语义。日常完整导航继续使用 `navigation2.launch.py` 与 `ldc_nav2.yaml` 的 AMCL + NavFn + DWB；本批没有把候选算法切换为实车默认算法。

## 修复行为

- 使用 `Costmap2D::getMutex()` 获取一致的地图副本。尺寸、原点、分辨率和栅格来自同一快照，搜索、平滑、坐标换算都读取此副本。实际地图随后更新或调整尺寸不会混入当前搜索；下一次规划重新获取快照。
- 新线段检查遍历闭线段触及的栅格，拒绝穿障碍、穿未知区（默认策略）、切角、沿障碍格边界擦行和触及地图外部。它检查膨胀代价图上的中心线，不声称完成随朝向变化的多边形车体扫掠检测。
- 平滑每次更新都检查前后两条线段。原始路径、精确端点连接线和最终输出均校验；非法短路径也不能作为回退结果返回。取消可在搜索、平滑和最终检查阶段生效。
- 保留请求目标的精确位置与四元数，中间点按路径方向生成朝向。同格目标可以成功规划；原地转向不再绕到栅格中心。拒绝非有限坐标、非法四元数和非法搜索上限，支持生命周期重新配置。

## 配置来源

| 入口 | 默认配置 | 用途 |
|---|---|---|
| `navigation2.launch.py` | `param/ldc_nav2.yaml` | 完整导航，当前基线为 NavFn Dijkstra + DWB |
| `planning_only.launch.py` | `param/nav2_planning.yaml` | 仅规划实验，NavFn A*，不含控制器 |
| 上述仅规划入口显式指定参数 | `param/nav2_planning_arena.yaml` | 自研 Arena A* 对照，不含控制器 |

Arena 参数仅使用 `allow_unknown`、`weight_cost`、`use_smoother`、`max_expand`；删除该实验文件中并未被插件读取的 `use_astar` 与 `tolerance`。两份仅规划文件里的半径和膨胀值是实验值，已去掉“实测”及“保证 DWB 绕行”的错误注释。膨胀半径是软代价场范围，不是整圈禁行区。

## 启动测试

```bash
cd /home/<user>/ros-car-migration
bash scripts/test_navigation_planner.sh
```

该入口构建当前源码并运行 CTest，ROS 包装层测试固定使用本机独立 Domain 194。测试覆盖精确目标与非零朝向、同格位移/原地转向、共享写锁、规划期间地图改尺寸/原点、取消、非法输入和重配置；线段检查与独立矩形相交判据进行 800 组对照，并保留 5 cm 栅格的原平滑穿障碍反例。

手动运行仅规划对照时，必须在独立 Domain 中使用该入口：

```bash
source scripts/env.bash
export ROS_DOMAIN_ID=194 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
ros2 launch ldc_navigation planning_only.launch.py \
  map:="$PWD/robot_ws/src/ldc_navigation/maps/test_arena.yaml" \
  params_file:="$PWD/robot_ws/src/ldc_navigation/param/nav2_planning_arena.yaml"
```

该 launch 的默认恒等 TF 只用于独立规划实验，不应用于实车导航。日常桌面入口无须更换。

## 尚需实测或后续批次处理

真实 footprint、轮轴参考点、有效轮距、停车距离、DWB 停止阈值和避障权重仍需独立实测。超声波采样时间/序号与 RangeSensorLayer 接入应单列固件与传感器批次，本批未刷写固件。Smac、Theta*、MPPI、EKF 和视觉控制均未因本批修复而启用。

修改前备份和测试日志位于 `/home/<user>/ros-car-migration-audit/arena-fixes-20261110-200349/`。`changes.json` 记录逐文件前后哈希，`changes.diff` 以本批工作区备份为基线。`rollback.py` 默认仅预览，使用 `--file` 可逐文件选择，`--apply` 才恢复；若交付后再次修改文件则拒绝覆盖。回滚后重新构建 `ldc_navigation`，使安装副本和动态库同步。

## 2026-11-11 软件验收结果

- 修复前新增的终点、同格目标和共享锁测试三项均失败；修复后全部通过。
- `bash scripts/test_navigation_planner.sh`：16 项 CTest 全部通过，含 7 项实际 ROS 包装层场景、3 项纯算法程序与 6 项检查。纯算法包含独立矩形相交判据的 800 组对照。
- 独立 Domain 193 启动真实 Nav2 planner_server 并通过 Action 调用：非零目标朝向、同格位移、原地转向全部成功；输出路径以独立采样检查未穿越地图障碍/未知区域。此测试使用合成地图，没有底盘运动。
- Python 回归 66 项通过，MCU 宿主 ASan/UBSan 回归通过。网页与视觉代码未改动，本批没有重复其构建或性能验收。
- 在临时头文件副本中翻转新碰撞条件，测试按预期失败；工作区源码未被变异操作修改。
- 五维复核：正确性由反例与真实包装层/Action 覆盖；可读性上复用线段检查并合并平滑迭代；架构上保留纯算法与 ROS 适配边界；没有新增外部写入接口；地图只复制一次，线段遍历使用常量额外空间。

详细日志、Action 复现脚本及 `action-acceptance.json` 均在本批审计目录。实车几何和运动性能验收仍待实测。
