"""Validate frame identity and original-pixel coordinates at the IPC boundary."""
import math

# 三色交通灯权重（competition_v2，nc=5，类别顺序固定）：
#   0=red 1=green 2=yellow 3=off 4=stop_sign
# 旧的 COCO 通用权重类名（'traffic light'/'stop sign'）仍保留，便于回滚与兼容双权重。
TRAFFIC_CLASSES = {
    # 新：三色权重类名
    'red', 'green', 'yellow', 'off', 'stop_sign',
    # 旧：COCO 通用权重类名（回滚用）
    'traffic light', 'stop sign',
}

# 亮灯三色（off 为熄灭态，不属于任一色）
LIGHT_COLORS = {'red', 'green', 'yellow'}


def checked_detections(result, frame_id, header, width, height):
    if result.get('id') != frame_id or result.get('header') != header:
        raise ValueError('worker 返回了错帧结果')
    if result.get('coordinates') != 'original_pixels' or (result.get('width'),result.get('height')) != (width,height):
        raise ValueError('worker 返回的坐标或尺寸不匹配')
    detections = result['detections']
    if not isinstance(detections, list) or len(detections)>100:
        raise ValueError('worker 检测列表超限')
    checked=[]
    for d in detections:
        box=d['xyxy'];score=d['score']
        if len(box)!=4 or not all(math.isfinite(v) for v in [*box,score]) or not 0<=score<=1:
            raise ValueError('worker 检测框或置信度无效')
        if not isinstance(d['name'],str) or not 0<len(d['name'])<=80:
            raise ValueError('worker 类别名无效')
        x1,y1,x2,y2=box
        if x2<=x1 or y2<=y1:
            raise ValueError('worker 返回退化检测框')
        x1,x2=max(0.,min(width,x1)),max(0.,min(width,x2))
        y1,y2=max(0.,min(height,y1)),max(0.,min(height,y2))
        if x2>x1 and y2>y1:
            checked.append({**d,'xyxy':[x1,y1,x2,y2]})
    return checked
