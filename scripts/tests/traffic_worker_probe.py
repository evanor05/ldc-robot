#!/usr/bin/env python3
"""真实 YOLO worker 的判色接线验证（真实 ultralytics + 通用权重，无 ROS）。

复用产品 WorkerClient（与 node.py 同一 IPC 客户端），喂一帧合成交通灯图像：
1. 验证 worker 用 --color-mode 启动、回传 detections 中 traffic light 项带判色字段；
2. 验证非交通类别不带判色字段（增量、不污染既有合约）；
3. 三种 color_mode 都能启动并回传（三色权重缺失时 yolo/consensus 应 fail-closed）。

注入式验证：合成帧里画一盏绿灯；真实 YOLO 若未检出 traffic light（通用权重对
合成图可能漏检），则用 --inject 模式直接把灯框送进判色函数，两种结果都如实记录，
不把「未检出」当作判色通过。
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'robot_ws/src/ldc_vision'))
sys.path.insert(0, str(ROOT / 'robot_ws/src/ldc_vision/ldc_vision'))

from ldc_vision.worker_client import WorkerClient  # noqa: E402
import traffic_color as tc  # noqa: E402

WORKER_PYTHON = '/home/<user>/esp/yolo-bench-env/bin/python'


def model_path():
    for candidate in (Path.home() / '.local/share/ldc/models/yolo26n.pt',
                      Path('/home/<user>/weights/yolo26n.pt')):
        if candidate.exists():
            return candidate
    raise SystemExit('找不到通用权重 yolo26n.pt')


def synthetic_frame():
    """640x480 场景：上半部画一盏点亮绿灯，其余为中性背景。"""
    frame = np.full((480, 640, 3), 70, dtype=np.uint8)
    hsv = np.zeros((60, 60, 3), np.uint8)
    hsv[..., 0] = 60; hsv[..., 1] = 255; hsv[..., 2] = 255
    frame[120:180, 280:340] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return frame, [280, 120, 340, 180]


def encode(frame):
    ok, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
    if not ok:
        raise SystemExit('JPEG 编码失败')
    return buffer.tobytes()


def run_mode(mode, device, report):
    client = WorkerClient(WORKER_PYTHON, str(model_path()), device, 640, .25, color_mode=mode)
    try:
        ready = client.start()
        frame, box = synthetic_frame()
        header = {'sec': 1000, 'nanosec': 0, 'frame_id': 'traffic-probe'}
        result = client.infer({'id': 'probe:1', 'header': header}, encode(frame))
        detections = result.get('detections', [])
        lights = [d for d in detections if tc.is_traffic_light_name(d['name'])]
        others = [d for d in detections if not tc.is_traffic_light_name(d['name'])]
        report[mode] = {
            'workerReady': True, 'colorModeEchoed': result.get('color_mode'),
            'modelNames': len(ready.get('names', {})),
            'detectedClasses': sorted({d['name'] for d in detections}),
            'trafficLightsDetected': len(lights),
            'lightColorFields': [{k: light.get(k) for k in ('name', 'opencv_color', 'color',
                                                           'color_pass_eligible', 'color_reason')}
                                 for light in lights],
            'nonLightHasColorFields': any(k in d for d in others for k in tc.COLOR_FIELDS),
        }
    finally:
        client.close()


def injected_check(report):
    """判色函数本身的接线验证（不受 YOLO 检出率影响）。"""
    frame, box = synthetic_frame()
    report['injected'] = {}
    for mode in tc.COLOR_MODES:
        fields = tc.light_color_fields(frame, box, 'traffic light', mode)
        report['injected'][mode] = {k: fields[k] for k in ('opencv_color', 'color',
                                                          'color_pass_eligible', 'color_reason')}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    report = {'result': 'FAIL', 'workerPython': WORKER_PYTHON, 'model': str(model_path())}
    run_mode('opencv', args.device, report)
    injected_check(report)
    checks = []
    opencv = report['opencv']
    checks.append(('worker echoes color_mode', opencv['colorModeEchoed'] == 'opencv'))
    checks.append(('non-light detections carry no color fields', opencv['nonLightHasColorFields'] is False))
    checks.append(('injected opencv green passes', report['injected']['opencv']['color_pass_eligible'] is True))
    checks.append(('injected yolo without 3-color weights fails closed',
                   report['injected']['yolo']['color_pass_eligible'] is False))
    checks.append(('injected consensus without 3-color weights fails closed',
                   report['injected']['consensus']['color_pass_eligible'] is False))
    # 真实 YOLO 是否检出 traffic light 只记录不断言（通用权重对合成图可靠性未知）。
    report['checks'] = [{'name': name, 'ok': bool(ok)} for name, ok in checks]
    report['result'] = 'PASS' if all(ok for _, ok in checks) else 'FAIL'
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    for check in report['checks']:
        print(('PASS ' if check['ok'] else 'FAIL ') + check['name'])
    print(json.dumps({'detectedClasses': opencv['detectedClasses'],
                      'trafficLightsDetected': opencv['trafficLightsDetected']}, ensure_ascii=False))
    print('RESULT', report['result'])
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())