"""Fit candidate corrections from independent references; never edit live parameters."""
import csv
import json
import math
from pathlib import Path
import statistics
import yaml

SCHEMAS = {
    'distance': ['trial_id', 'direction', 'reference_m', 'odom_m', 'end_yaw_deg', 'valid', 'notes'],
    'turn': ['trial_id', 'direction', 'reference_deg', 'odom_deg', 'odom_source', 'valid', 'notes'],
    'laser': ['trial_id', 'reference_m', 'measured_m', 'valid', 'notes'],
    'bearing': ['trial_id', 'sensor', 'reference_deg', 'measured_deg', 'valid', 'notes'],
    'braking': ['trial_id', 'speed_mps', 'stop_distance_m', 'valid', 'notes'],
    'navigation': ['trial_id', 'target_x_m', 'target_y_m', 'target_yaw_deg', 'actual_x_m', 'actual_y_m', 'actual_yaw_deg', 'valid', 'notes'],
}


def number(row, key):
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f'{key} 不是有限数值')
    return value


def rows_from(path, schema):
    accepted, rejected, seen = [], [], set()
    with Path(path).open(newline='') as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != SCHEMAS[schema]:
            raise ValueError(f'{path.name} 表头与模板不一致')
        for line, row in enumerate(reader, 2):
            if not any(row.values()):
                continue
            identifier = row.get('trial_id', '').strip()
            if not identifier or identifier in seen:
                raise ValueError(f'{path.name}:{line} 试验 ID 缺失或重复')
            seen.add(identifier)
            if row['valid'].lower() not in ('true', 'false'):
                raise ValueError(f'{path.name}:{line} valid 必须显式填写 true/false')
            if row['valid'].lower() == 'false':
                rejected.append(identifier)
                continue
            accepted.append(row)
    return accepted, rejected


def stats(values):
    return dict(n=len(values), mean=statistics.mean(values), mean_absolute=statistics.mean(map(abs, values)),
                rmse=math.sqrt(statistics.mean(v*v for v in values)), maximum_absolute=max(map(abs, values)),
                sample_std=statistics.stdev(values) if len(values)>1 else None)


def scale_fit(reference, measured):
    if not reference or any(abs(x) < 1e-9 for x in measured):
        raise ValueError('尺度拟合需要非零测量')
    if any(a*b <= 0 for a, b in zip(reference, measured)):
        raise ValueError('参考与里程计方向不一致，应先校正轴向，不能用负轮径补偿')
    return sum(a*b for a, b in zip(reference, measured))/sum(b*b for b in measured)


def analyze(session):
    session = Path(session)
    baseline = json.loads((session/'baseline.json').read_text())
    geometry = yaml.safe_load((session/'dimensions.yaml').read_text())
    result = dict(status='awaiting_measurements', candidates={}, measurements={}, warnings=[], excluded={})
    datasets = {}
    for kind in SCHEMAS:
        datasets[kind], result['excluded'][kind] = rows_from(session/(kind+'.csv'), kind)
    common_scale = None
    for kind, ref_key, measured_key, directions in [
        ('distance', 'reference_m', 'odom_m', ('forward', 'reverse')),
        ('turn', 'reference_deg', 'odom_deg', ('ccw', 'cw'))]:
        rows = datasets[kind]
        if not rows:
            continue
        refs, measured = [], []
        for row in rows:
            ref, value = number(row, ref_key), number(row, measured_key)
            direction = row['direction']
            if direction not in directions or (ref > 0) != (direction == directions[0]):
                raise ValueError(f'{kind}:{row["trial_id"]} 方向和带符号参考值不一致')
            refs.append(ref); measured.append(value)
        fit = scale_fit(refs, measured)
        counts = {d: sum(r['direction']==d for r in rows) for d in directions}
        entry = dict(scale_reference_over_measured=fit, raw_error=stats([b-a for a, b in zip(refs, measured)]),
                     fitted_error=stats([fit*b-a for a, b in zip(refs, measured)]), counts=counts,
                     quality='provisional' if min(counts.values())<5 else 'candidate_requires_independent_validation')
        result['measurements'][kind] = entry
        if kind == 'distance':
            common_scale = fit
            result['candidates']['common_wheel_diameter_mm'] = baseline['firmware']['wheel_diameter_mm']*fit
            result['warnings'].append('距离尺度也可能受编码器计数/轮滑影响；共同轮径候选不是实测物理轮径，不能据此单独辨识左右轮差异。')
            entry['end_yaw_deg'] = stats([number(r, 'end_yaw_deg') for r in rows])
        elif common_scale is not None and all(r['odom_source']=='wheel_only' for r in rows):
            result['candidates']['effective_track_width_mm'] = baseline['firmware']['track_width_mm']*common_scale/fit
        else:
            result['warnings'].append('有效轮距暂不计算：需要先取得直行尺度，并确认转角来源是同一固件配置下的 wheel_only，而非 IMU/EKF/AMCL。')
    for kind in ('laser', 'bearing', 'braking', 'navigation'):
        rows = datasets[kind]
        if not rows:
            continue
        if kind == 'laser':
            reference = [number(r, 'reference_m') for r in rows]
            measured = [number(r, 'measured_m') for r in rows]
            if min(reference+measured) <= 0:
                raise ValueError('雷达参考与读数必须为正距离，单位 m')
            result['measurements'][kind] = dict(error_m=stats([b-a for a, b in zip(reference, measured)]))
        elif kind == 'bearing':
            by_sensor = {}
            for row in rows:
                error = (number(row, 'measured_deg')-number(row, 'reference_deg')+180)%360-180
                by_sensor.setdefault(row['sensor'], []).append(error)
            result['measurements'][kind] = {sensor: dict(error_deg=stats(values)) for sensor, values in by_sensor.items()}
        elif kind == 'braking':
            grouped = {}
            for row in rows:
                speed, distance = number(row, 'speed_mps'), number(row, 'stop_distance_m')
                if speed<=0 or distance<0:
                    raise ValueError('制动速度必须为正、停止距离不得为负')
                grouped.setdefault(str(speed), []).append(distance)
            result['measurements'][kind] = {speed: dict(n=len(values), measured_max_m=max(values)) for speed, values in grouped.items()}
            result['warnings'].append('已测最大停车距离不是安全上界；曝光/通信/决策延迟、载荷、地面和额外余量仍需独立测量。')
        else:
            position, yaw = [], []
            for row in rows:
                position.append(math.hypot(number(row,'actual_x_m')-number(row,'target_x_m'), number(row,'actual_y_m')-number(row,'target_y_m')))
                yaw.append((number(row,'actual_yaw_deg')-number(row,'target_yaw_deg')+180)%360-180)
            result['measurements'][kind] = dict(position_error_m=stats(position), yaw_error_deg=stats(yaw))
    extents = geometry['body_extents_from_base_link_m']
    if all(extents[k] is not None for k in ('front','back','left','right','height')):
        if not all(isinstance(v, (int,float)) and math.isfinite(v) and v>0 for v in extents.values()):
            raise ValueError('车体包络必须为有限正数，单位 m')
        front, back, left, right = [extents[k] for k in ('front','back','left','right')]
        result['candidates']['measured_footprint_m'] = [[front,left],[-back,left],[-back,-right],[front,-right]]
    result['status'] = 'candidate_only' if result['measurements'] or result['candidates'] else 'awaiting_measurements'
    result['warnings'].append('所有候选值仅写入本次报告，不修改固件、URDF、Nav2 或相机配置；拟合残差不等于独立验收精度。')
    return result
