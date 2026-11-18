#!/usr/bin/env python3
"""生成离线验证用的合成试验场地（P5 PGM + YAML）。

注意：这是**合成图**，不是真实场地地图，只用来验证
  - 地图三态语义（free / occupied / unknown）是否被规划器正确解读
  - A* 是否绕障、是否拒绝穿越未知区域
真实比赛地图必须由 GMapping/Cartographer 实车建图产生。

输出：maps/test_arena.pgm / maps/test_arena.yaml
"""
import os

W = H = 400          # 400 * 0.05 m = 20 m x 20 m
RES = 0.05
ORIGIN = (-10.0, -10.0)

FREE = 254
OCC = 0
UNKNOWN = 205

img = bytearray([FREE]) * (W * H)


def put(x0, y0, x1, y1, val):
    for y in range(max(0, y0), min(H, y1 + 1)):
        row = y * W
        for x in range(max(0, x0), min(W, x1 + 1)):
            img[row + x] = val


# 1) 外圈墙：3 格厚（0.15 m）
put(0, 0, W - 1, 2, OCC)
put(0, H - 3, W - 1, H - 1, OCC)
put(0, 0, 2, H - 1, OCC)
put(W - 3, 0, W - 1, H - 1, OCC)

# 2) 内部障碍（若干矩形），构成绕行走廊
put(80, 80, 84, 300, OCC)      # 竖墙，留上下通道
put(200, 40, 204, 260, OCC)    # 竖墙，底部留口
put(120, 150, 190, 154, OCC)   # 横墙
put(250, 200, 340, 204, OCC)   # 横墙
put(140, 320, 260, 324, OCC)   # 横向隔断

# 3) 未知区域：右上角整块未探索，规划器必须拒绝进入
put(300, 40, 396, 160, UNKNOWN)

# 4) 实心障碍块（40x40 格 = 2 m x 2 m），用于验证"目标落在障碍上必须失败"
#    需要足够厚，否则选不出"深处"的障碍栅格来测。
put(150, 200, 190, 240, OCC)

# 5) 起点/终点都放在已知自由区
START = (30, 350)   # (x, y) 栅格
GOAL = (350, 330)

for name, (cx, cy) in (('start', START), ('goal', GOAL)):
    assert img[cy * W + cx] == FREE, f'{name} 不在自由栅格上'

here = os.path.dirname(os.path.abspath(__file__))
maps_dir = os.path.join(os.path.dirname(here), 'maps')
os.makedirs(maps_dir, exist_ok=True)

pgm_path = os.path.join(maps_dir, 'test_arena.pgm')
with open(pgm_path, 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (W, H))
    f.write(bytes(img))

yaml_path = os.path.join(maps_dir, 'test_arena.yaml')
with open(yaml_path, 'w') as f:
    f.write(
        'image: test_arena.pgm\n'
        'mode: trinary\n'
        f'resolution: {RES}\n'
        f'origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        # 0.196 是 ROS 惯例：像素 205 恰好判为 unknown，不会被当成 free
        'free_thresh: 0.196\n'
    )


def grid_to_world(cx, cy):
    """PGM 行序自上而下，世界 y 自下而上，换算时必须翻转 y。"""
    return (ORIGIN[0] + (cx + 0.5) * RES,
            ORIGIN[1] + (H - 1 - cy + 0.5) * RES)


print(f'wrote {pgm_path}')
print(f'wrote {yaml_path}')
print(f'START(grid)={START} -> world=({grid_to_world(*START)[0]:.3f}, '
      f'{grid_to_world(*START)[1]:.3f})')
print(f'GOAL(grid)={GOAL} -> world=({grid_to_world(*GOAL)[0]:.3f}, '
      f'{grid_to_world(*GOAL)[1]:.3f})')
print('未知块(grid x 300-396, y 40-160) -> 世界 x 5.00-9.85, y 1.95-7.95，'
      '规划器应拒绝进入该区域')
