# ldc_vision

ROS Python 3.14 负责调度与标准检测消息；独立 Python 3.12 worker 加载本地 YOLO26n / PyTorch。最多一帧推理中、一帧最新待处理，超过 1 秒不显示，推理超时 5 秒。单帧坏图/解码错误只跳过该帧；worker 进程级故障按 1/2/4/8/16 秒退避自动重启，连续失败最多 5 次，达到上限后保留原图并等待手动重启。GPU 故障不静默改用 CPU。

跟踪/时序平滑默认开启：IoU 0.3 贪心匹配、连续 2 帧确认、丢失 3 帧删除、显示最低分 0.35、框/分数 EMA 0.45、类别冻结；参数 `tracking_enabled`、`track_confirm_frames`、`track_max_missed`、`track_min_score`、`track_iou_threshold` 可在 `config/vision.yaml` 调整。

使用统一入口 `bash scripts/start_vision.sh --real`；离线、网页和 RViz 说明见 [LDC 视觉接入与使用](../../../docs/LDC视觉接入与使用.md)。节点不发布运动指令或导航目标。
