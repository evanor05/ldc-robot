#!/usr/bin/env python3
"""离线校验路径规划：批量起终点，逐点验合法性。

校验项：
  1. 正例：N 组"已知自由区"随机起终点，统计成功率；
  2. 路径合法性：路径上每个点对应的地图栅格必须是 FREE，不能是 obstacle 或 unknown；
  3. 负例：目标落在障碍上 / 落在未知区上，必须**失败**（不允许输出穿墙路径）；
  4. 记录规划耗时与路径长度。

用法（planner_only.launch.py 已启动时）：
  python3 test/planning_offline_check.py \
      --map src/ldc_navigation/maps/test_arena.yaml --pairs 100
"""
import argparse
import json
import math
import os
import random
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

FREE, OCC, UNKNOWN = 254, 0, 205
HERE = os.path.dirname(os.path.abspath(__file__))
# Real maps used by the default map quality gate. These are read-only checks;
# no ROS node or planner is started in --quality-only mode.
DEFAULT_QUALITY_MAPS = [
    os.path.join(HERE, '..', 'maps', 'room.yaml'),
    os.path.join(HERE, '..', '..', '..', '..', 'maps', 'map_20261111145304_250.yaml'),
    os.path.join(HERE, '..', '..', '..', '..', 'maps', 'map_20261111145429_359.yaml'),
]


def map_quality_ok(composition):
    """Return True when the map is not the known broken 'all free / no unknown' case."""
    return not (composition['free_pct'] > 95.0 and composition['unknown_pct'] < 0.5)


def read_pgm(path):
    data = open(path, 'rb').read()
    idx, fields = 0, []
    while len(fields) < 4:
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b'#':
            while data[idx:idx + 1] not in (b'\n', b''):
                idx += 1
            continue
        start = idx
        while not data[idx:idx + 1].isspace():
            idx += 1
        fields.append(data[start:idx])
    idx += 1
    w, h = int(fields[1]), int(fields[2])
    return w, h, data[idx:idx + w * h]


def read_map_yaml(path):
    cfg = {}
    with open(path) as f:
        for line in f:
            line = line.split('#')[0].strip()
            if not line or ':' not in line:
                continue
            k, v = line.split(':', 1)
            cfg[k.strip()] = v.strip()
    origin = [float(x) for x in cfg['origin'].strip('[]').split(',')]
    return {
        'image': cfg['image'].strip('"\''),
        'resolution': float(cfg['resolution']),
        'origin': origin,
        'free_thresh': float(cfg.get('free_thresh', 0.196)),
        'occupied_thresh': float(cfg.get('occupied_thresh', 0.65)),
    }


class MapGroundTruth:

    def __init__(self, yaml_path):
        cfg = read_map_yaml(yaml_path)
        img_path = cfg['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        self.w, self.h, self.px = read_pgm(img_path)
        self.res = cfg['resolution']
        self.ox, self.oy = cfg['origin'][0], cfg['origin'][1]
        # map_server trinary: occ = (255 - pixel)/255
        #   occ > occupied_thresh            -> 100 (occupied)
        #   occ < free_thresh                -> 0   (free)
        #   其余                             -> -1  (unknown)
        # 反解出像素边界，避免把 205（unknown 惯例值）当成 free。
        self.free_min = int(round(255.0 * (1.0 - cfg['free_thresh']))) + 1
        self.occ_max = int(round(255.0 * (1.0 - cfg['occupied_thresh']))) - 1

    def world_to_cell(self, wx, wy):
        mx = int((wx - self.ox) / self.res)
        my = int((wy - self.oy) / self.res)
        return mx, my

    def in_bounds(self, mx, my):
        return 0 <= mx < self.w and 0 <= my < self.h

    def pixel(self, mx, my):
        # PGM 行序自上而下，世界 y 自下而上
        return self.px[(self.h - 1 - my) * self.w + mx]

    def cell_to_world(self, mx, my):
        return self.ox + (mx + 0.5) * self.res, self.oy + (my + 0.5) * self.res

    def composition(self):
        """地图三态占比，用于暴露全白碎片图。"""
        total = self.w * self.h
        free = occ = unk = 0
        for v in self.px:
            if v >= self.free_min:
                free += 1
            elif v <= self.occ_max:
                occ += 1
            else:
                unk += 1
        return {
            'free_pct': round(free * 100.0 / total, 2),
            'occupied_pct': round(occ * 100.0 / total, 2),
            'unknown_pct': round(unk * 100.0 / total, 2),
        }

    def free_cells_with_clearance(self, clearance_cells=5):
        """返回远离障碍/未知区至少 clearance_cells 格的自由栅格。"""
        out = []
        r = clearance_cells
        for my in range(r, self.h - r, 2):
            for mx in range(r, self.w - r, 2):
                if self.pixel(mx, my) < self.free_min:
                    continue
                ok = True
                for dy in (-r, 0, r):
                    for dx in (-r, 0, r):
                        if self.pixel(mx + dx, my + dy) < self.free_min:
                            ok = False
                            break
                    if not ok:
                        break
                if ok:
                    out.append((mx, my))
        return out

    def find_cell(self, pred):
        for my in range(self.h):
            for mx in range(self.w):
                if pred(self.pixel(mx, my)):
                    return mx, my
        return None

    def find_deep_cell(self, pred, depths=(8, 5, 3, 2)):
        """找一块深处区域：邻域全部满足 pred。

        必须这样选，否则目标可能落在障碍/未知区边缘 1~2 格内，
        NavFn 的 tolerance 吸附会把它合法地挪到旁边的自由格，
        负例就变成假失败。障碍较薄时自动降低要求的深度。
        """
        for depth_cells in depths:
            r = depth_cells
            for my in range(r, self.h - r):
                for mx in range(r, self.w - r):
                    if not pred(self.pixel(mx, my)):
                        continue
                    ok = True
                    for dy in (-r, 0, r):
                        for dx in (-r, 0, r):
                            if not pred(self.pixel(mx + dx, my + dy)):
                                ok = False
                                break
                        if not ok:
                            break
                    if ok:
                        return mx, my, depth_cells
        return None


class PlannerCheck(Node):

    def __init__(self, gt):
        super().__init__('planning_offline_check')
        self.gt = gt
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    def wait_server(self, timeout=30.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def plan(self, start_world, goal_world, timeout=20.0):
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = 'GridBased'
        for field, (wx, wy) in (('start', start_world), ('goal', goal_world)):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            setattr(goal, field, ps)

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None, 'goal_rejected'
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout)
        wrapped = res_future.result()
        if wrapped is None:
            return None, 'timeout'
        return wrapped.result, 'ok'

    def path_violations(self, path):
        """返回违规点列表与路径长度，违规指落在障碍或未知栅格。"""
        bad = []
        length = 0.0
        prev = None
        for pose in path.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y
            mx, my = self.gt.world_to_cell(wx, wy)
            if not self.gt.in_bounds(mx, my):
                bad.append((wx, wy, 'out_of_bounds'))
            else:
                v = self.gt.pixel(mx, my)
                if v <= self.gt.occ_max:
                    bad.append((wx, wy, 'obstacle'))
                elif v < self.gt.free_min:
                    bad.append((wx, wy, 'unknown'))
            if prev is not None:
                length += math.hypot(wx - prev[0], wy - prev[1])
            prev = (wx, wy)
        return bad, length


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', help='地图 YAML 路径；--quality-only 时可省略')
    ap.add_argument('--pairs', type=int, default=100, help='正例组数')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--json-out', default='')
    ap.add_argument('--quality-only', action='store_true',
                    help='只做地图阈值/三态质量检查，不启动 ROS planner')
    ap.add_argument('--quality-map', action='append', default=[],
                    help='质量检查地图 YAML，可重复；默认检查 3 张真实地图')
    args = ap.parse_args()

    quality_maps = args.quality_map or (DEFAULT_QUALITY_MAPS if args.quality_only else [])
    if args.quality_only:
        bad = 0
        for path in quality_maps:
            try:
                gt = MapGroundTruth(path)
                comp = gt.composition()
            except Exception as error:
                print(f'FAIL {path}: {error}')
                bad += 1
                continue
            ok = map_quality_ok(comp)
            print(f"{'PASS' if ok else 'FAIL'} {path}  "
                  f"free={comp['free_pct']}% occupied={comp['occupied_pct']}% "
                  f"unknown={comp['unknown_pct']}%")
            if not ok:
                bad += 1
                print('  reason: free>95% and unknown<0.5%; '
                      'likely free_thresh/unknown semantics bug')
        print(f'QUALITY VERDICT: {"PASS" if bad == 0 else "FAIL"} ({len(quality_maps) - bad}/'
              f'{len(quality_maps)} maps)')
        return 0 if bad == 0 else 2

    if not args.map:
        ap.error('--map is required unless --quality-only is used')
    gt = MapGroundTruth(args.map)
    comp = gt.composition()
    print(f'map: {args.map}  {gt.w}x{gt.h}  res={gt.res}')
    print(f"地图构成: free={comp['free_pct']}%  "
          f"occupied={comp['occupied_pct']}%  unknown={comp['unknown_pct']}%")
    if not map_quality_ok(comp):
        print('FAIL: 该地图 free>95% 且 unknown<0.5%，很可能是 free_thresh/unknown 语义错误；'
              '拒绝在碎片图上做规划验收')
        return 2

    rclpy.init()
    node = PlannerCheck(gt)
    if not node.wait_server(30.0):
        print('FAIL: /compute_path_to_pose 不可用')
        rclpy.shutdown()
        return 2

    candidates = gt.free_cells_with_clearance()
    print(f'可用自由栅格（含 5 格安全距离）: {len(candidates)}')
    if len(candidates) < 10:
        print('FAIL: 可测自由栅格太少')
        rclpy.shutdown()
        return 2

    rnd = random.Random(args.seed)
    ok = 0
    total_viol = 0
    lengths, times = [], []
    failures = []

    for i in range(args.pairs):
        s_cell = rnd.choice(candidates)
        g_cell = rnd.choice(candidates)
        if math.hypot(s_cell[0] - g_cell[0], s_cell[1] - g_cell[1]) < 40:
            continue
        s_world = gt.cell_to_world(*s_cell)
        g_world = gt.cell_to_world(*g_cell)
        t0 = time.monotonic()
        result, status = node.plan(s_world, g_world)
        dt = time.monotonic() - t0
        if result is None:
            failures.append({'status': status})
            continue
        if result.error_code != 0 or len(result.path.poses) < 2:
            failures.append({'error_code': int(result.error_code),
                             'error_msg': result.error_msg})
            continue
        bad, length = node.path_violations(result.path)
        total_viol += len(bad)
        if bad:
            failures.append({'violations': bad[:5], 'count': len(bad)})
        else:
            ok += 1
            lengths.append(length)
            times.append(dt)

    # ---- 负例 ----
    # 目标必须选在障碍/未知区的"深处"（邻域 8 格都是同类），
    # 否则 NavFn 的 tolerance 吸附会合法地把终点挪到旁边自由格，负例失去意义。
    negatives = []
    occ_cell = gt.find_deep_cell(lambda p: p == OCC)
    unk_cell = gt.find_deep_cell(lambda p: p == UNKNOWN)
    for label, found in (('goal_on_obstacle', occ_cell), ('goal_in_unknown', unk_cell)):
        if found is None:
            negatives.append({'case': label, 'skipped': 'no deep cell of this type',
                              'pass': False})
            continue
        mx, my, depth = found
        cell = (mx, my)
        s_cell = candidates[0]
        result, status = node.plan(gt.cell_to_world(*s_cell), gt.cell_to_world(*cell))
        if result is None:
            negatives.append({'case': label, 'rejected_by': status,
                              'depth_cells': depth, 'pass': True})
            continue
        # 即使规划器返回了路径，也必须逐点校验它没有真的开进障碍/未知区
        bad = []
        if len(result.path.poses) >= 2:
            bad, _ = node.path_violations(result.path)
        passed = result.error_code != 0 and len(result.path.poses) == 0
        negatives.append({
            'case': label,
            'depth_cells': depth,
            'error_code': int(result.error_code),
            'error_msg': result.error_msg[:200],
            'poses': len(result.path.poses),
            'path_violations': len(bad),
            'pass': bool(passed),
        })

    summary = {
        'map': args.map,
        'map_composition': comp,
        'pairs_requested': args.pairs,
        'pairs_ok': ok,
        'pairs_failed': len(failures),
        'success_rate': round(ok / max(1, ok + len(failures)), 4),
        'path_violation_points': total_viol,
        'avg_path_len_m': round(sum(lengths) / len(lengths), 3) if lengths else None,
        'avg_plan_sec': round(sum(times) / len(times), 4) if times else None,
        'max_plan_sec': round(max(times), 4) if times else None,
        'negative_cases': negatives,
        'failures_sample': failures[:8],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.json_out:
        with open(args.json_out, 'w') as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)

    all_neg_pass = all(n.get('pass') for n in negatives)
    verdict = (summary['path_violation_points'] == 0
               and summary['success_rate'] >= 0.99
               and all_neg_pass)
    print('VERDICT:', 'PASS' if verdict else 'FAIL')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if verdict else 1


if __name__ == '__main__':
    sys.exit(main())
