"""双路交通灯判色（纯 OpenCV/NumPy，无 ROS/torch 依赖）。

同一模块被两侧导入：
- 隔离 YOLO worker（worker.py，flat sys.path，Python 3.12）：每帧只跑一次 YOLO，
  predict 之后、结果回传之前对同一帧原图的灯框做 OpenCV 判色；node 侧绘制标注框
  发生在回传之后，因此满足「绘制标注框前判色」。
- ROS node（node.py，包相对导入，Python 3.14）：观测消息构造辅助。

三种配置（color_mode）：
- opencv   ：只用 OpenCV 判色（当前仅通用权重时的默认）。
- yolo     ：只读模型实际标签对应的灯色（需三色权重；通用权重下为 unknown）。
- consensus：双路一致才采信；**仅同一灯框两路都为绿才 pass_eligible**（放行依据）。
  任一路缺失/冲突 → unknown（fail-closed）。

真实三色权重本轮未发现（lead 侦察 + F 复核）：yolo/consensus 用离线样例与 mock
标签验证接口，真实权重效果标「待验证」。
"""
import cv2
import numpy as np

COLORS = ('red', 'yellow', 'green', 'unknown')
COLOR_MODES = ('opencv', 'yolo', 'consensus')
# worker→node→观测话题增量携带的判色字段（既有检测字段不受影响）。
COLOR_FIELDS = ('opencv_color', 'opencv_confidence', 'yolo_color', 'yolo_label',
                'color', 'color_conflict', 'color_pass_eligible', 'color_reason')
GENERIC_LIGHT_NAMES = ('traffic light',)

# OpenCV HSV 色相桶（H 取值 0..179）。红跨 0 度需两段。
_HUE_RANGES = {
    'red': ((0, 10), (160, 179)),
    'yellow': ((14, 38),),
    'green': ((40, 95),),
}
_MIN_SAT = 90   # 低于此饱和度视为白/灰/过曝，不算点亮
_MIN_VAL = 90   # 低于此亮度视为熄灭/暗背景
_MIN_PURITY = 0.55  # 主色占点亮像素比例下限，低于则 ambiguous


def _result(color, confidence, reason, lit_ratio, lit_pixels):
    return {'color': color, 'confidence': float(confidence), 'reason': reason,
            'lit_ratio': float(lit_ratio), 'lit_pixels': int(lit_pixels)}


def classify_crop(frame, xyxy):
    """对同一帧原图（BGR）的灯框判红/黄/绿/unknown。

    只统计「点亮」像素（高饱和+高亮度）的色相分布；熄灭灯、过曝、遮挡、
    框太小都返回 unknown 并附原因，绝不猜测。
    """
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (int(round(float(v))) for v in xyxy)
    x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
    y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
    if x2 - x1 < 6 or y2 - y1 < 6:
        return _result('unknown', 0.0, 'box_too_small', 0.0, 0)
    crop = frame[y1:y2, x1:x2]
    if min(crop.shape[:2]) >= 5:
        crop = cv2.medianBlur(crop, 3)  # 抑制 JPEG 噪点，不改变灯色布局
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    lit = (sat >= _MIN_SAT) & (val >= _MIN_VAL)
    total = int(lit.sum())
    area = int(crop.shape[0]) * int(crop.shape[1])
    min_lit = max(4, area // 100)
    if total < min_lit:
        return _result('unknown', 0.0, 'no_lit_region', total / area, total)
    counts = {}
    for color, ranges in _HUE_RANGES.items():
        mask = np.zeros(hue.shape, dtype=bool)
        for lo, hi in ranges:
            mask |= (hue >= lo) & (hue <= hi)
        counts[color] = int((mask & lit).sum())
    color = max(counts, key=counts.get)
    dominant = counts[color]
    purity = dominant / total
    if dominant == 0 or purity < _MIN_PURITY:
        return _result('unknown', round(purity, 3), 'ambiguous', total / area, total)
    return _result(color, round(purity, 3), '', total / area, total)


def color_from_label(name, label_map=None):
    """模型实际标签 → 灯色。返回 (color|None, 命中标签|None)。

    通用权重的 'traffic light' 不含灯色信息 → (None, None)。
    三色权重标签（如 'red traffic light' / 'traffic_light_green'）按关键词映射；
    label_map 提供精确覆盖（现场权重标签不规则时用配置指定）。
    """
    if not isinstance(name, str):
        return None, None
    if label_map and name in label_map:
        color = label_map[name]
        return (color, name) if color in COLORS and color != 'unknown' else (None, None)
    text = name.lower().replace('_', ' ')
    if 'light' not in text:
        return None, None
    if 'red' in text:
        return 'red', name
    if 'yellow' in text or 'amber' in text:
        return 'yellow', name
    if 'green' in text:
        return 'green', name
    return None, None


def has_color_labels(names):
    """扫描 model.names，返回 {标签名: 灯色}；空字典 = 该权重无三色标签。"""
    mapping = {}
    for name in (names or {}).values():
        color, _ = color_from_label(name)
        if color is not None:
            mapping[name] = color
    return mapping


def is_traffic_light_name(name, color_labels=None):
    """通用交通灯名或三色权重灯色名都算交通灯（filter_mode traffic 用）。"""
    if name in GENERIC_LIGHT_NAMES:
        return True
    if color_labels and name in color_labels:
        return True
    color, _ = color_from_label(name)
    return color is not None


def resolve_color(mode, opencv_result, yolo_color):
    """按配置融合两路结果。返回 {color, pass_eligible, conflict, reason}。

    pass_eligible 是「可作为放行依据」的唯一标志：
    - opencv：opencv 判绿；
    - yolo：标签判绿（标签缺失 → unknown，不放行）；
    - consensus：**同一灯框两路都为绿**才放行；一致非绿采信该色；
      冲突/缺路 → unknown（fail-closed）。
    """
    if mode not in COLOR_MODES:
        raise ValueError(f'判色模式无效：{mode}')
    opencv_color = (opencv_result or {}).get('color', 'unknown')
    opencv_reason = (opencv_result or {}).get('reason', '')
    if mode == 'opencv':
        return {'color': opencv_color, 'pass_eligible': opencv_color == 'green',
                'conflict': False, 'reason': opencv_reason if opencv_color == 'unknown' else ''}
    if mode == 'yolo':
        if yolo_color is None:
            return {'color': 'unknown', 'pass_eligible': False, 'conflict': False,
                    'reason': 'yolo_label_unavailable'}
        return {'color': yolo_color, 'pass_eligible': yolo_color == 'green',
                'conflict': False, 'reason': ''}
    # consensus
    if yolo_color is None:
        return {'color': 'unknown', 'pass_eligible': False, 'conflict': False,
                'reason': 'yolo_path_unavailable'}
    if opencv_color == yolo_color:
        return {'color': opencv_color, 'pass_eligible': opencv_color == 'green',
                'conflict': False, 'reason': ''}
    return {'color': 'unknown', 'pass_eligible': False, 'conflict': True,
            'reason': 'dual_path_conflict'}


def light_color_fields(frame, xyxy, name, mode, label_map=None):
    """worker 侧便捷入口：一个灯框的两路判色 + 融合结果（增量字段）。"""
    opencv_result = classify_crop(frame, xyxy)
    yolo_color, yolo_label = color_from_label(name, label_map)
    resolved = resolve_color(mode, opencv_result, yolo_color)
    return {'opencv_color': opencv_result['color'],
            'opencv_confidence': opencv_result['confidence'],
            'yolo_color': yolo_color, 'yolo_label': yolo_label,
            'color': resolved['color'], 'color_conflict': resolved['conflict'],
            'color_pass_eligible': resolved['pass_eligible'],
            'color_reason': resolved['reason']}


def light_record(item):
    """跟踪后的检测 → 观测话题灯记录（只取合约字段，增量判色字段透传）。"""
    record = {'track_id': item.get('track_id'), 'name': item['name'],
              'score': float(item['score']), 'xyxy': [float(v) for v in item['xyxy']],
              'missed': int(item.get('missed', 0))}
    for key in COLOR_FIELDS:
        if key in item:
            record[key] = item[key]
    return record


def frame_observation(*, session, frame_id, sequence, header, width, height,
                      color_mode, lights, captured_at, published_at, wall_at):
    """新画面观测：原图 header.stamp+frame_id（旧帧判定）+采集单调时钟（断流判定）。

    lights 为空列表 = 新画面没检测到灯（与 unknown 灯色、stale_frame、stream_lost
    显式区分）。captured_at 为采集时刻的单调时钟估计（node 侧 monotonic-age）。
    """
    stamp_ns = int(header['sec']) * 1_000_000_000 + int(header['nanosec'])
    return {'kind': 'frame', 'session': session, 'id': frame_id, 'sequence': int(sequence),
            'header': {'sec': int(header['sec']), 'nanosec': int(header['nanosec']),
                       'frame_id': str(header['frame_id'])},
            'stamp_ns': stamp_ns, 'width': int(width), 'height': int(height),
            'color_mode': color_mode, 'light_count': len(lights), 'lights': list(lights),
            'at': float(captured_at), 'published_at': float(published_at), 'wall_at': float(wall_at)}


def status_observation(kind, **fields):
    """非画面事件观测：stale_frame / stream_lost / stream_recovered / detector_down。"""
    if kind not in ('stale_frame', 'stream_lost', 'stream_recovered', 'detector_down'):
        raise ValueError(f'观测事件类型无效：{kind}')
    return {'kind': kind, **fields}