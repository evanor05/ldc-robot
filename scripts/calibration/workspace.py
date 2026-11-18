"""Private calibration sessions for the web wizard. Writes reports, never live parameters."""
import argparse
import copy
import csv
import datetime
import fcntl
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shutil
import sys
import tempfile
import uuid
import yaml
from cli import initialize
from measurements import SCHEMAS, analyze
from camera import calibrate_folder

IDENTIFIER = re.compile(r'^[a-zA-Z0-9_-]{1,80}$')
TEXT_FIELDS = {'trial_id', 'direction', 'odom_source', 'sensor', 'valid', 'notes'}


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise ValueError('无效记录标识')
    return value


def child(root, name):
    result = root/identifier(name)
    if result.is_symlink() or result.resolve().parent != root.resolve():
        raise ValueError('不允许符号链接或越界目录')
    return result


def read_json(path):
    if path.is_symlink() or path.stat().st_size > 2*1024*1024:
        raise ValueError('记录文件无效或过大')
    return json.loads(path.read_text())


def write_json(path, data):
    encoded = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)
    with path.open('x') as handle:
        handle.write(encoded)


def new_id():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:8]


def safe_replace(session, filename, content):
    dest = session/filename
    if dest.is_symlink():
        raise ValueError('记录文件不能是符号链接')
    revision = session/'revisions'/new_id()
    revision.mkdir(parents=True)
    shutil.copy2(dest, revision/filename)
    temporary = dest.with_suffix(dest.suffix+'.tmp')
    temporary.write_text(content)
    temporary.replace(dest)


def session_view(session):
    data = read_json(session/'session.json')
    data['dimensions'] = yaml.safe_load((session/'dimensions.yaml').read_text())
    data['schemas'] = SCHEMAS
    data['measurements'] = {}
    data['measurementCounts'] = {}
    for kind in SCHEMAS:
        with (session/(kind+'.csv')).open(newline='') as handle:
            rows = list(csv.DictReader(handle))
            data['measurements'][kind] = rows[-100:]
            data['measurementCounts'][kind] = len(rows)
    data['reports'] = [dict(id=p.stem, **read_json(p)) for p in sorted((session/'reports').glob('*.json'))[-30:]]
    data['observations'] = [read_json(p) for p in sorted((session/'observations').glob('*.json'))[-100:]]
    data['captures'] = []
    for folder in sorted((session/'captures').iterdir())[-60:]:
        if folder.is_dir() and not folder.is_symlink():
            status = read_json(folder/'capture.json') if (folder/'capture.json').exists() else {'state': 'incomplete'}
            if (folder/'summary.json').exists():
                status['summary'] = read_json(folder/'summary.json')
            data['captures'].append(dict(id=folder.name, **status))
    return data


def geometry_fields(geometry, prefix=''):
    result = {}
    for key, value in geometry.items():
        path = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            result.update(geometry_fields(value, path))
        elif value is None or (isinstance(value, (int, float)) and not isinstance(value, bool)):
            result[path] = value
    return result


def mutate_geometry(session, values):
    data = yaml.safe_load((session/'dimensions.yaml').read_text())
    allowed = geometry_fields(data)
    if not isinstance(values, dict) or set(values)-allowed.keys():
        raise ValueError('未知尺寸/外参字段')
    for field, value in values.items():
        if value is not None:
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or abs(value)>(360 if field.endswith('_deg') else 100):
                raise ValueError(f'{field} 必须为有限实测数值（尺寸为 m，角度为 deg）')
            if ('body_extents' in field or 'diameter' in field or field=='mechanical_track_m') and value<=0:
                raise ValueError(f'{field} 必须大于零')
        target = data
        parts = field.split('.')
        for part in parts[:-1]:
            target = target[part]
        target[parts[-1]] = value
    data['status'] = 'partially_measured'
    safe_replace(session, 'dimensions.yaml', yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def append_measurement(session, kind, row):
    if kind not in SCHEMAS or not isinstance(row, dict) or set(row) != set(SCHEMAS[kind]):
        raise ValueError('测量字段与模板不一致')
    clean = {}
    for key, value in row.items():
        if key in TEXT_FIELDS:
            text = str(value).strip()
            if len(text)>500 or '\x00' in text:
                raise ValueError('记录文本过长或包含空字符')
            clean[key] = text
        else:
            number = float(value)
            if not math.isfinite(number) or abs(number)>1e7:
                raise ValueError(f'{key} 数值无效')
            clean[key] = number
    filename = kind+'.csv'
    original = (session/filename).read_text()
    if len(original.splitlines())>2000:
        raise ValueError('单项最多 2000 条记录，请新建会话')
    stream = io.StringIO()
    csv.DictWriter(stream, fieldnames=SCHEMAS[kind]).writerow(clean)
    content = original+stream.getvalue()
    # Validate the full candidate dataset before committing a revision.
    with tempfile.TemporaryDirectory(prefix='ldc-calibration-check-') as temporary:
        temporary = Path(temporary)
        for source in [session/'baseline.json', session/'dimensions.yaml', *session.glob('*.csv')]:
            shutil.copy2(source, temporary/source.name)
        (temporary/filename).write_text(content)
        analyze(temporary)
    safe_replace(session, filename, content)


def candidate(session):
    result = analyze(session)
    report_id = new_id()
    sources = [session/'baseline.json', session/'dimensions.yaml', *sorted(session.glob('*.csv'))]
    snapshot = session/'reports'/report_id
    snapshot.mkdir()
    hashes = {}
    for source in sources:
        shutil.copy2(source, snapshot/source.name)
        hashes[source.name] = hashlib.sha256(source.read_bytes()).hexdigest()
    result.update(createdAt=datetime.datetime.now(datetime.timezone.utc).isoformat(), sourceHashes=hashes,
                  sessionId=session.name, reportId=report_id, applied=False,
                  validationOf=read_json(session/'session.json').get('validationOf'))
    if result['validationOf']:
        # Evaluate raw held-out errors, never the fitted residuals of this new session.
        evaluation = {}
        for kind in ('distance', 'turn'):
            from measurements import rows_from
            rows, _ = rows_from(session/(kind+'.csv'), kind)
            if not rows:
                continue
            reference = 'reference_m' if kind == 'distance' else 'reference_deg'
            measured = 'odom_m' if kind == 'distance' else 'odom_deg'
            errors = [abs(float(row[measured])-float(row[reference]))/abs(float(row[reference])) for row in rows]
            counts = result['measurements'][kind]['counts']
            metric = max(errors) if kind == 'distance' else sum(errors)/len(errors)*360
            evaluation[kind] = dict(samples=len(rows), sufficientTrials=min(counts.values())>=5,
                                    metric='max_relative_error' if kind=='distance' else 'mean_deg_per_360',
                                    value=metric, target=.01 if kind=='distance' else 2.,
                                    meetsNumericalTarget=metric<=(.01 if kind=='distance' else 2.)+1e-12)
        result['independentValidation'] = dict(evaluation=evaluation, appliedVersionVerified=False,
            status='requires_measurement_and_version_review',
            note='使用新会话原始误差；试验独立性、真值及实际应用版本仍需人工核对，不自动放行车辆')
    write_json(session/'reports'/(report_id+'.json'), result)
    return result


def run(root, action, payload):
    if action=='list':
        return [read_json(p/'session.json') for p in sorted(root.iterdir(), reverse=True)
                if p.is_dir() and not p.is_symlink() and (p/'session.json').is_file()][:200]
    if action=='create':
        if sum(p.is_dir() for p in root.iterdir())>=200:
            raise ValueError('已有 200 个会话，请人工归档后再创建')
        name = str(payload.get('name', '标定与诊断')).strip()
        if not name or len(name)>80:
            raise ValueError('会话名称为 1–80 个字符')
        validation = payload.get('validationOf')
        if validation:
            parent = child(root, validation['sessionId'])
            previous = read_json(parent/'reports'/(identifier(validation['reportId'])+'.json'))
            validation = dict(sessionId=parent.name, reportId=previous['reportId'], sourceHashes=previous['sourceHashes'],
                              note='独立复测会话；须人工确认已应用的参数版本，不自动判定候选已生效')
        session = child(root, new_id())
        initialize(session)
        (session/'reports').mkdir()
        (session/'captures').mkdir()
        write_json(session/'session.json', dict(id=session.name, name=name, validationOf=validation,
                    createdAt=datetime.datetime.now(datetime.timezone.utc).isoformat(), state='awaiting_measurements',
                    simulation=bool(payload.get('simulation')), applied=False))
        return session_view(session)
    session = child(root, payload.get('sessionId'))
    if not session.is_dir():
        raise ValueError('会话不存在')
    if action=='get':
        return session_view(session)
    if action=='dimensions':
        mutate_geometry(session, payload.get('values'))
        return session_view(session)
    if action=='measurement':
        append_measurement(session, payload.get('kind'), payload.get('row'))
        return session_view(session)
    if action=='analyze':
        return candidate(session)
    if action=='camera':
        capture = child(session/'captures', payload.get('captureId'))
        output = session/'reports'/('camera_'+new_id())
        square = float(payload.get('squareMm', 20))
        if not math.isfinite(square) or not 1<=square<=100:
            raise ValueError('实测棋盘格长必须在 1–100mm 之间')
        result = calibrate_folder(capture/'images', output, 9, 6, square)
        report = dict(result=result, candidateDirectory=str(output), applied=False, status='camera_candidate',
                      reportId=output.name, sessionId=session.name, captureId=capture.name,
                      sourceHashes={'images.json': hashlib.sha256((output/'images.json').read_bytes()).hexdigest()},
                      createdAt=datetime.datetime.now(datetime.timezone.utc).isoformat())
        write_json(session/'reports'/(output.name+'.json'), report)
        return report
    if action=='observations':
        note = str(payload.get('note', '')).strip()
        if not 1<=len(note)<=2000:
            raise ValueError('人工观测记录为 1–2000 字符')
        folder = session/'observations'
        folder.mkdir(exist_ok=True)
        write_json(folder/(new_id()+'.json'), dict(note=note, project=payload.get('project'), createdAt=datetime.datetime.now(datetime.timezone.utc).isoformat()))
        return {'saved': True}
    raise ValueError('未知标定操作')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True)
    parser.add_argument('--action', required=True)
    args = parser.parse_args()
    try:
        root = Path(args.root).expanduser()
        if root.is_symlink():
            raise ValueError('数据根目录不能是符号链接')
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        root = root.resolve()
        payload = json.loads(sys.stdin.read(128*1024+1) or '{}')
        if not isinstance(payload, dict):
            raise ValueError('请求必须为 JSON 对象')
        with (root/'.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            result = run(root, args.action, payload)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(1, f'标定操作失败：{error}\n')


if __name__ == '__main__':
    main()
