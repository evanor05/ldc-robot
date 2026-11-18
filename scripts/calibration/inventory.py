"""Read-only geometry audit. CAD and configuration are never labelled measurements."""
import hashlib
import math
from pathlib import Path
import re
import struct
import xml.etree.ElementTree as ET
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def transform(origin):
    result = np.eye(4)
    if origin is None:
        return result
    x, y, z = map(float, origin.get('xyz', '0 0 0').split())
    r, p, yaw = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yaw), math.sin(yaw)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def mesh_vertices(path):
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from('<I', raw, 80)[0]
        if 84 + 50*count == len(raw):
            dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            return np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    vertices = re.findall(r'vertex\s+([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)', raw.decode('ascii'))
    if not vertices:
        raise ValueError(f'无法解析 STL: {path.name}')
    return np.array(vertices, dtype=float)


def read_inventory(root=ROOT):
    root = Path(root)
    paths = ['firmware/chassis/main/config/motion_config.h',
             'robot_ws/src/ldc_description/urdf/ldc.urdf',
             'robot_ws/src/ldc_navigation/param/ldc_nav2.yaml',
             'robot_ws/src/camsense_lidar/params/camsense_lidar.yaml']
    source = (root/paths[0]).read_text()
    def constant(name):
        match = re.search(r'\b'+name+r'\s*=\s*([\d.]+)f?\s*;', source)
        if not match:
            raise ValueError(f'未找到数值常量 {name}，请人工核对源码')
        return float(match[1])
    robot = ET.parse(root/paths[1]).getroot()
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    def to_base(link):
        if link == 'base_link':
            return np.eye(4)
        joint = joints[link]
        return to_base(joint.find('parent').get('link')) @ transform(joint.find('origin'))
    xyz = {name: to_base(name)[:3, 3].tolist() for name in
           ('left_wheel_link', 'right_wheel_link', 'laser_frame', 'camera_link', 'camera_optical_frame', 'imu_link')}
    all_bounds, radius = [], 0.0
    mesh_sources = {}
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                continue
            prefix = 'package://ldc_description/'
            filename = mesh.get('filename')
            if not filename.startswith(prefix):
                raise ValueError('只读取本包内的模型文件')
            path = root/'robot_ws/src/ldc_description'/filename[len(prefix):]
            points = mesh_vertices(path).astype(float) * np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
            matrix = to_base(link.get('name')) @ transform(visual.find('origin'))
            points = points @ matrix[:3, :3].T + matrix[:3, 3]
            all_bounds.append((points.min(axis=0), points.max(axis=0)))
            radius = max(radius, float(np.linalg.norm(points[:, :2], axis=1).max()))
            mesh_sources[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    bounds_min = np.min([b[0] for b in all_bounds], axis=0)
    bounds_max = np.max([b[1] for b in all_bounds], axis=0)
    nav = yaml.safe_load((root/paths[2]).read_text())
    costmaps = {key: nav[key][key]['ros__parameters'] for key in ('local_costmap', 'global_costmap')}
    nav_summary = {key: {k: value[k] for k in ('robot_base_frame', 'resolution', 'robot_radius') if k in value}
                   for key, value in costmaps.items()}
    wheel_track = abs(xyz['left_wheel_link'][1]-xyz['right_wheel_link'][1])
    firmware_track = constant('kTrackWidthMm')/1000
    warnings = []
    if abs(wheel_track-firmware_track) > .001:
        warnings.append(f'轮距来源不一致：固件 {firmware_track*1000:.1f} mm，URDF 轮心间距 {wheel_track*1000:.1f} mm；有效轮距与机械轮距均需实测，不能直接互相覆盖。')
    if abs(xyz['left_wheel_link'][0]) > .001:
        warnings.append(f'URDF 轮轴中点位于 base_link.x={xyz["left_wheel_link"][0]:.5f} m；需核对差速里程计实际参考点，不能把旋转中心自动视作 base_link 原点。')
    for key, params in costmaps.items():
        if params.get('robot_radius', float('inf')) < radius and not params.get('footprint'):
            warnings.append(f'{key} 的圆半径 {params["robot_radius"]:.3f} m 小于 CAD 模型水平最远点 {radius:.3f} m；需测量实车包络后配置 footprint。')
    collision = robot.find("link[@name='base_link']/collision/geometry/box")
    summary = dict(status='unmeasured', firmware=dict(wheel_diameter_mm=constant('kWheelDiameterMm'),
                   track_width_mm=constant('kTrackWidthMm'), pulses_per_wheel_rev=constant('kPulsesPerWheelRev')),
                   urdf=dict(wheel_center_separation_m=wheel_track, sensor_positions_in_base_link_m=xyz,
                             collision_box_m=list(map(float, collision.get('size').split())),
                             cad_visual_bounds_m=dict(min=bounds_min.tolist(), max=bounds_max.tolist(),
                                                     size=(bounds_max-bounds_min).tolist(), circumscribed_radius=radius)),
                   navigation=nav_summary,
                   goal_tolerances=nav['controller_server']['ros__parameters']['general_goal_checker'],
                   standalone_lidar=yaml.safe_load((root/paths[3]).read_text()), warnings=warnings)
    summary['sources_sha256'] = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in paths}
    summary['sources_sha256'].update(mesh_sources)
    return summary
