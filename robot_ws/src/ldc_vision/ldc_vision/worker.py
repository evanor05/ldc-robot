"""Standalone worker: imports Torch here only, never imports ROS."""
import argparse
import hashlib
from contextlib import redirect_stdout
import os
from pathlib import Path
import sys
from ipc import read_packet, write_packet

# 判色模块与 worker 同级（flat sys.path 下可直接 import）。缺 cv2 时判色降级为
# unknown/不可用，YOLO 检测本身照常工作——判色故障不得让检测节点停摆。
try:
    import traffic_color
except Exception:  # noqa: BLE001
    traffic_color = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--device', default='0')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--confidence', type=float, default=.25)
    parser.add_argument('--color-mode', default='opencv',
                        choices=('opencv', 'yolo', 'consensus'))
    parser.add_argument('--color-label-map', default='',
                        help='JSON {标签名: red|yellow|green}，覆盖不规则的现场权重标签')
    args = parser.parse_args()
    label_map = {}
    if args.color_label_map:
        import json
        try:
            loaded = json.loads(args.color_label_map)
            if isinstance(loaded, dict):
                label_map = loaded
        except ValueError:
            label_map = {}
    os.environ['YOLO_OFFLINE'] = 'true'
    try:
        if not Path(args.model).is_file():
            raise ValueError('模型文件不存在；禁止自动下载')
        with redirect_stdout(sys.stderr):
            import cv2
            import numpy as np
            import torch
            from ultralytics import YOLO
            if args.device != 'cpu' and not torch.cuda.is_available():
                raise RuntimeError('指定 GPU 不可用；请显式选择 CPU')
            model = YOLO(args.model)
            model.predict(np.zeros((640,640,3),np.uint8), device=args.device,
                          imgsz=args.imgsz, verbose=False, max_det=100)
        write_packet(1, {'ready': True, 'names': model.names,
                         'model_sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest()})
        while True:
            metadata, image = read_packet(0, 3600)
            # A single bad frame must not kill the worker.  Return an error
            # result carrying the frame identity; the ROS node treats it as a
            # frame-level failure and keeps consuming later frames.
            try:
                frame = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('输入 JPEG 无法解码')
                with redirect_stdout(sys.stderr):
                    result = model.predict(frame, device=args.device, imgsz=args.imgsz,
                                           conf=args.confidence, verbose=False, max_det=100)[0]
                # Ultralytics boxes.xyxy already refer to result.orig_shape: do not
                # undo letterbox a second time or resize coordinates for the browser.
                detections = [{'class_id':int(cls), 'name':model.names[int(cls)], 'score':float(score),
                               'xyxy':[float(v) for v in box]}
                              for box,score,cls in zip(result.boxes.xyxy.cpu().tolist(),
                                                     result.boxes.conf.cpu().tolist(),result.boxes.cls.cpu().tolist())]
                # 判色在组装结果之后、回传之前：node 的绘制标注发生在本包回传之后，
                # 因此满足「绘制标注框前、用同一帧原图判色」，且每帧只跑一次 YOLO。
                # 只对交通灯框做（通用名或三色权重标签），其他类别不加字段。
                if traffic_color is not None:
                    color_labels = traffic_color.has_color_labels(model.names)
                    for detection in detections:
                        if not traffic_color.is_traffic_light_name(detection['name'], color_labels):
                            continue
                        try:
                            detection.update(traffic_color.light_color_fields(
                                frame, detection['xyxy'], detection['name'],
                                args.color_mode, label_map))
                        except Exception as error:  # noqa: BLE001
                            # 单个灯框判色失败只让该框 unknown，不影响整帧结果。
                            detection.update({
                                'opencv_color': 'unknown', 'opencv_confidence': 0.0,
                                'yolo_color': None, 'yolo_label': None,
                                'color': 'unknown', 'color_conflict': False,
                                'color_pass_eligible': False,
                                'color_reason': f'color_error: {error}'})
                write_packet(1, {'id':metadata['id'], 'header':metadata['header'],
                                 'width':frame.shape[1], 'height':frame.shape[0],
                                 'coordinates':'original_pixels', 'detections':detections,
                                 'color_mode':args.color_mode})
            except Exception as error:
                write_packet(1, {'id':metadata.get('id'), 'header':metadata.get('header'),
                                 'error':str(error)})
    except EOFError:
        return
    except Exception as error:
        try: write_packet(1, {'error':str(error)})
        except (OSError, TimeoutError): pass
        raise SystemExit(1)

if __name__ == '__main__':
    main()
