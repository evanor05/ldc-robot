# LDC 标定与精度验收

更新：2026-11-08。本轮实车离线，完成的是**只读审计、标定工具和空白记录模板**。未测量真实轮径/轮距、车体尺寸、相机内参或制动距离，也没有把估计参数写入固件、URDF、Nav2 或相机配置。

统一入口：`/home/<user>/ros-car-migration/scripts/calibrate.sh`。一键启动、Qt 和网页说明见 [启动指南](LDC一键启动与模块入口.md)。

## 当前参数与发现

| 项目 | 当前配置 / 模型值 | 结论 |
| --- | --- | --- |
| 固件共同轮径 | 65.0 mm | 配置值，非本批实测 |
| 固件有效轮距 | 131.7 mm | 用于差速正/逆运动学，待标定 |
| 编码器每轮一圈计数 | 4680 | 需核对减速比、计数倍率和实际计数，不能当已验证 CPR |
| URDF 轮心间距 | 162.0 mm | 与固件不同；机械轮距与有效轮距不能盲目互相覆盖 |
| URDF 轮轴中点 | base_link.x = -50.75 mm | 要核对里程计参考点是否实际在轮轴中心 |
| CAD 可视模型包络 | 约 158.14 × 162.00 × 155.20 mm | 仅模型零位变换后的包围盒，不能替代带载实车测量 |
| CAD 水平最远点 | 距 base_link 约 114.76 mm | 大于当前 Nav2 圆半径 100 mm，需实测 footprint |
| URDF 基座碰撞盒 | 144.5 × 95 × 80.5 mm | 只描述基座局部，不等于整车含轮子包络 |
| Nav2 地图分辨率 | 0.05 m | 栅格尺寸，不是定位精度 |
| Nav2 目标容差 | 0.25 m / 0.25 rad（约 14.32°） | 允许判定到达的阈值，不代表实际误差，也不是已达 5cm/5° |
| 相机 | 型号未知，未标定时 K[0]=0 | 不猜 OV2640/OV3660，不根据框大小直接给米制距离 |

源文件、模型哈希及自动审计结果在本批审计目录 `geometry-audit.json` / 标定目录 `baseline.json`。MCU 源码当前 `MotionController::Update` 用原始编码器增量积分 wheel-only 位置和 yaw，IMU 相对 yaw 单独用于相对转角控制；**车上是否已刷入同版本必须另外确认**，不能仅凭话题名 `/odom` 推断数据来源。

先依次解决：坐标原点与整车尺寸 → 编码器方向/计数 → 直行尺度 → 转角/有效轮距 → IMU、雷达外参 → 相机内参/外参 → 停车距离 → 独立导航/识别精度验收。不要同时改轮距、PID、激光角度和定位参数，再试图判断哪一项改善了误差。

## 建立一份自己的记录

```bash
cd /home/<user>/ros-car-migration
bash scripts/calibrate.sh init /ABS/PATH/my-calibration-session
bash scripts/calibrate.sh audit
```

本轮已准备的空白记录：`/home/<user>/ros-car-migration-audit/calibration-startup-20261108/calibration-session/`。其中 `dimensions.yaml` 全部实测值为 null，六个 CSV 只有表头，没有混入模拟测量。`init` 和结果输出均拒绝覆盖已有文件。

每次测量记录地面、载荷、轮胎状态、电池、运行固件/主机版本、传感器安装状态。不同参数版本或轮胎/地面变化后建立新记录，不把多版本数据混合拟合。

## 1. 车体大小与坐标原点

工具：卷尺/卡尺、直尺、胶带、直角基准。约定车体 x 向前、y 向左、z 向上，逆时针 yaw 为正。先在地面标记左右驱动轮接地点中心连线及中点；确认 base_link 在实车上的投影位置，再量包络。

`dimensions.yaml`：

- `body_extents_from_base_link_m.front/back/left/right`：从 **base_link 投影点** 向四个方向量到整车最外缘，单位 m，均为正距离；包括轮胎、摄像头、线缆及固定载荷。
- `height`：整车地面至最高点。
- `left_wheel_diameter_m/right_wheel_diameter_m`：带载有效轮径的物理测量。
- `mechanical_track_m`：两驱动轮接地中心间距。
- `axle_center_x_m`：轮轴中心相对 base_link 的带符号 x 偏移；不要直接填模型的 -0.05075 当实测。
- `sensor_extrinsics_from_base_link`：激光、camera_link、IMU 的安装 x/y/z（m）与 roll/pitch/yaw（deg）。没量的项保留 null；标准 optical 旋转不是实测安装外参。

如果当前 base_link 还不能在实车上定位，先以轮轴中点记录原始尺量和照片，保持上述字段未完成，待统一参考点后再换算。填完包络后，`analyze` 只生成矩形 footprint 候选，不自动更改 costmap。膨胀层半径与 footprint 是不同的量，不能通过增大 inflation 代替正确车体轮廓。

## 2. 只读 ROS 采集

先运行一键入口或原有驱动。采集器只订阅数据，不启动实车驱动，不发布速度、急停解除或导航目标；实际移动由操作者低速遥控。

```bash
cd /home/<user>/ros-car-migration
# 实车：一次独立试验；先让车和传感器数据稳定，再开始记录
bash scripts/calibrate.sh record --real --seconds 40 --output /ABS/PATH/trial-forward-01
# 相机标定采样：间隔 1 秒保存原始 JPEG，期间移动标定板到不同位置/倾角
bash scripts/calibrate.sh record --real --seconds 40 --camera-every 1 --output /ABS/PATH/camera-session
# 离线采集流程测试（必须先 start_all.sh --sim；结果只能验证工具）
bash scripts/calibrate.sh record --sim --seconds 5 --output /ABS/PATH/simulation-record
# 解析里程计、静态 IMU 统计、前方墙面近似
bash scripts/calibrate.sh summarize /ABS/PATH/trial-forward-01/samples.jsonl \
  --output /ABS/PATH/trial-forward-01-summary.json
```

记录包含 `/odom`、`/scan`、`/imu`、TF、CameraInfo 和选取的相机原始帧，保留来源 Header 和主机接收时间。非有限 scan 数据保存为 null，不伪装成有效距离。拍照复用 ROS 图像，**不会额外连接 ESP32**。

里程计总结给出初始朝向下的纵向/横向位移和解包累计角度；360°/720°不会用最后一个四元数的零角度代替。时间重复、回拨、间断超过 0.5s、位姿跳变或超过当前固件运动上限时拒绝累计，不能静默拼接重启前后数据。

记录窗口应覆盖一次完整试验且停稳开始、停稳结束；不能在整场记录中挑多个运动再把整段累计值当成单次距离。原始数据仍保留，可以另行选段，不自动补齐丢失样本。

## 3. 距离与有效轮距

先在平整地面，使用足够长的直线基准；历史计划的阶段目标为正反各 5 次 2m、顺逆各 5 次 360°，现场空间不足可缩短并明确测量分辨率。参考值由卷尺/地面起止标记或独立角度基准提供，**不能用 `/odom`、IMU 自己验证自己**。

`distance.csv` 表头：

```text
trial_id,direction,reference_m,odom_m,end_yaw_deg,valid,notes
```

`direction=forward/reverse`，前进距离为正、后退为负；`reference_m` 是实际投影距离，`odom_m` 取总结的 `forward_delta_m`，`end_yaw_deg` 取累计偏航。若明显走弧线、打滑、抬轮或参考测量失败，记录原因并设 `valid=false`，不直接删除。`valid` 必须显式填 true/false；重复 ID、NaN、方向反号均拒绝。

`turn.csv`：

```text
trial_id,direction,reference_deg,odom_deg,odom_source,valid,notes
```

`direction=ccw/cw`，逆时针为正、顺时针为负，360°保持 ±360。`odom_deg` 使用解包累计值；仅在确认原始轮式里程计及固件配置一致后填 `odom_source=wheel_only`。填 `imu`/`ekf`/未知值时工具不反推轮距。

拟合使用过原点最小二乘：距离尺度 k = Σ(odom×reference)/Σ(odom²)，共同轮径候选 `D_new=D_old×k`。转角尺度 q 同样拟合；在轮径按 k 校正的前提下，有效轮距候选 `B_new=B_old×k/q`。不能把这个式子反写，也不能拿融合 yaw 校准原始轮距。

这些参数存在可辨识性限制：距离偏差可能来自 CPR、轮胎压缩或打滑，不一定是轮径；共同尺度不能单独标定左右轮径。报告同时给原始误差、拟合残差、方向计数与不足次数提示，拟合数据上的低残差不等于实际精度。修正参数后仍要另采独立试验，目标可先设距离≤1%、转角平均≤2°/圈，是否达到由新数据决定。

## 4. 雷达距离、方向与 IMU

`laser.csv`：`trial_id,reference_m,measured_m,valid,notes`。先用正前方同一面平墙，在多个独立距离测量；参考必须从激光测量原点沿正确方向量，不能从保险杠量。记录系统偏差与重复性；先确认单位、镜像/方向，再考虑调整量程或安装 TF。

采集总结的 `front_wall` 使用 laser_frame 前方 ±20° 做直线拟合，仅保留残差≤2cm 的近似平面。它输出激光原点到平面的距离及法向角，不是车头净空；角域内有门框、多面墙或斜放物体时，应换目标/选区域，不把统计结果盲目当标定真值。

`bearing.csv`：`trial_id,sensor,reference_deg,measured_deg,valid,notes`。用正前、左/右已知角度位置的标志测方向，先确认左右符号及正前方向，工具报告带符号环绕误差。各 sensor 必须使用自己的明确同一角度约定；ROS 激光/车体 yaw 左正，相机射线工具输出右正，不能混合。

IMU：先记录至少 30 秒静止数据，再做六面重力方向检查和独立转角试验。总结输出 gyro 均值/样本标准差（rad/s），只有确认全程静止时均值才可作为零偏候选；运动中的均值不是零偏。若 `/imu` 缺失，结果为 null，不捏造统计；本批模拟底盘没有发布真实 IMU。不要把这份短时统计直接当 EKF 协方差。

## 5. 相机内参与角度

每份标定目录有 `checkerboard_9x6_20mm_A4.svg`：A4 横向、9×6 **内角点**、20mm 方格、底部 100mm 校验线。打印选择 100%/实际大小，不能“适应纸张”；用尺验证实际格长和 100mm 线，粘在平整刚性板上。格长如果实际不是 20mm，命令必须填实际值。

保持相机分辨率、裁剪、vflip/hmirror、焦距/对焦状态固定，采集 25–40 张不同位置、倾角和距离的清晰画面，覆盖中心、边缘和四角；不能只连拍同一个视角。

```bash
bash scripts/calibrate.sh camera /ABS/PATH/camera-session/images \
  --cols 9 --rows 6 --square-mm 20 --output /ABS/PATH/camera-fit-01
```

工具使用 OpenCV 棋盘角点与 `calibrateCamera`，拒绝混合分辨率、去除完全重复文件，至少 15 个有效视角。约 20% 视角留出，不参与内参拟合，使用 PnP 计算保留视角重投影误差；报告包含覆盖九宫格数和倾角范围。初步质量检查阈值：训练 RMS≤1px、最差保留视角≤1.5px、中心覆盖≥6/9、倾角跨度≥15°。这些阈值是筛查条件，不是测距精度承诺。

输出 `camera.candidate.yaml`、`images.json`、`report.json`；不会覆盖当前配置。参数模型为 plumb_bob，不假装已做鱼眼标定。经过独立检查后才手动试用：

```bash
bash scripts/start_vision.sh --real 'url:=http://<CAMERA_IP>:81/' \
  'calibration_file:=/ABS/PATH/camera-fit-01/camera.candidate.yaml'
```

**重投影误差不等于米制测距误差，内参标定也不等于 camera→base 外参。** 二维框没有深度。已知物体尺寸/PnP、地面约束或雷达关联都需要额外条件和实测验证，不在此自动输出距离。

```bash
# 使用原始图像像素坐标与原始尺寸，不能用网页缩放后的坐标
bash scripts/calibrate.sh ray /ABS/PATH/camera.candidate.yaml \
  --u 420 --v 240 --width 640 --height 480
```

射线结果在 `camera_optical_frame`：x 右、y 下、z 前；`bearing_right_deg` 右正、`elevation_up_deg` 上正，distance_m 始终为 null。尺寸不匹配或未标定时拒绝计算；转到车体方向仍需真实外参。当前相机具体型号未知，不影响按实际画面标定，但更换传感器/镜头或图像变换后应重新验证。[OpenCV 标定方法](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

## 6. 制动与最终精度

`braking.csv`：`trial_id,speed_mps,stop_distance_m,valid,notes`。在受控低速场地测不同速度的实际停车距离、重复试验，标记停止请求位置和停稳位置；记录载荷、地面、电池及网络状态。工具只分组显示已测最大值，**不会把少量样本最大值当安全上界**。摄像头曝光/传输/识别/决策及底盘执行延迟需要计入整个停车预算。

`navigation.csv`：`trial_id,target_x_m,target_y_m,target_yaw_deg,actual_x_m,actual_y_m,actual_yaw_deg,valid,notes`。实际位置/角度用独立地面基准量，不用 AMCL 自报位姿当真值。报告位置欧氏误差与 yaw 环绕误差，可统计不同方向/距离的成功率；不能先把 goal_tolerance 改小就声称更准确。

识别精度另外评估：收集本机相机的真实图片并完整标注，按场景/拍摄批次划分训练与验证，避免相邻视频帧跨集合；统计交通灯/STOP 的 TP/FP/FN、召回率、误检、不同距离和光照表现。confidence=0.25 不是“准确率25%”。已有模型对照入口：

```bash
bash scripts/compare_yolo.sh --models /ABS/PATH/yolo26n.pt /ABS/PATH/yolo26s.pt \
  --images /ABS/PATH/labeled-images --output /ABS/PATH/comparison
```

每张 JPG 必须有同名完整 YOLO 标签 TXT；无目标图也需空 TXT，标签使用模型原 COCO 类别索引。本工具只给固定 confidence=0.25/IoU=0.5 下交通灯与 STOP 的 P/R，以及实际 IPC 推理延迟；不声称 mAP。交通灯颜色尚未分类，识别结果仍不控制运动。

## 生成候选报告与回滚

```bash
bash scripts/calibrate.sh analyze /ABS/PATH/my-calibration-session \
  --output /ABS/PATH/report-01.json
```

仅输出报告，不自动应用。所有原始参数、来源哈希、空白记录和候选输出可追溯。标定前后必须用不同的独立试验做对照，实车恢复后再由测量证据决定哪一项可正式采用。

本批源码备份、逐文件回滚和测试记录位于 `/home/<user>/ros-car-migration-audit/calibration-startup-20261108/`。未修改运动参数文件、URDF 或 Nav2 YAML；本阶段没有实车参数需要回滚。新增启动/标定工具的逐文件回滚见本批 `changes.json` / `rollback.py`，不要用 `git reset --hard`。

回滚前先停止本批服务。相机发布器与新增 lease.py、Qt 启动脚本与适配器应按组恢复；不要只移走仍被导入的模块。脚本会保留回滚前内容，并拒绝覆盖交付后又修改的文件。
