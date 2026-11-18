# ldc_camera

单路 HTTP/MJPEG 采集发布，保留原始 JPEG，默认 10 FPS，Best Effort、depth=1。图像 Header 使用主机收到完整帧时的时间，不是曝光时间；frame 默认为 `camera_optical_frame`。自动发现沿用现有 UDP 协议，支持手动 `url`。

```bash
source /home/<user>/ros-car-migration/scripts/env.bash
ros2 launch ldc_camera http_video_publisher.launch.py
# 原始 RGB 有订阅者时才解码转换发布；相同 Header 的 CameraInfo 始终发布
ros2 launch ldc_camera http_video_publisher.launch.py \
  'url:=http://<CAMERA_IP>:81/' publish_raw:=true
```

未标定时 CameraInfo.K[0]=0，尺寸失配时报告未标定。UDP 发现监听固定端口 33333；该端口被占用时会明确报错，不会静默改用临时端口。配置位于 `config/http_video_publisher.yaml`；`calibration_file` 指向真实标定 YAML。旧 `use_ffmpeg` / `jpeg_quality` 参数只为调用兼容保留，不再改变原 JPEG。

完整视觉启动、网页、RViz、ROS 接口、测试和回滚说明：
[视觉接入与使用](../../../docs/LDC视觉接入与使用.md)。

相机具体传感器型号待实机确认；不据此假定焦距或外参。

## 网页 5173、YOLO 与启动脚本

```bash
cd /home/<user>/ros-car-migration
bash scripts/start_all.sh --real
# 离线演示同样使用 localhost:5173
bash scripts/start_all.sh --sim
```

打开 **http://localhost:5173/**，在“摄像头与 YOLO”中观察画面。默认选择 YOLO 识别叠框图，可切换原图；一键入口会启动视觉服务。仅启动旧网页入口时，在卡片点击“启动视觉”。浏览器通过本机网关预览，不再直连 ESP32。默认 YOLO26n；交通灯颜色仍未判定。

单独启动相机：`bash scripts/start_camera.sh --real 'url:=http://<CAMERA_IP>:81/'`，随后 `bash scripts/start_yolo.sh --real`；或者直接 `bash scripts/start_vision.sh --real` 启动整组。不要叠加两种方式，相机采集锁会拒绝同一 Domain/图像话题的第二个采集者。

全部模块、Qt/RViz 和日志入口见 [一键启动与模块入口](../../../docs/LDC一键启动与模块入口.md)。相机内参、距离与角度标定见 [标定与精度验收](../../../docs/LDC标定与精度验收.md)。
