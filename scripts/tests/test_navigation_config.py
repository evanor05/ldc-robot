"""Static production-config regression for the navigation launch/params.

These assertions are intentionally offline: they catch the two configuration
regressions that caused real-vehicle costmap overload and the direct-launch
bypass, without starting ROS or a vehicle.
"""
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
PARAM = ROOT / 'robot_ws/src/ldc_navigation/param/ldc_nav2.yaml'
LAUNCH = ROOT / 'robot_ws/src/ldc_navigation/launch/navigation2.launch.py'


class NavigationConfigTests(unittest.TestCase):
    def setUp(self):
        self.params = yaml.safe_load(PARAM.read_text())

    def test_local_costmap_has_exactly_one_laser_layer(self):
        local = self.params['local_costmap']['local_costmap']['ros__parameters']
        plugins = local['plugins']
        self.assertIn('obstacle_layer', plugins)
        self.assertNotIn('voxel_layer', plugins,
                         'voxel + obstacle 同时订阅 /scan 会把 costmap 更新拖到 100–200ms')
        self.assertEqual(local['obstacle_layer']['observation_sources'], 'scan')
        self.assertEqual(local['obstacle_layer']['scan']['topic'], '/scan')
        self.assertGreaterEqual(local['update_frequency'], 10.0,
                                '单层 2D 雷达配置应至少 10Hz 更新，避免 controller 等待 costmap')

    def test_global_costmap_still_uses_single_scan_layer(self):
        global_map = self.params['global_costmap']['global_costmap']['ros__parameters']
        self.assertNotIn('voxel_layer', global_map['plugins'])
        self.assertEqual(global_map['track_unknown_space'], True)

    def test_production_navfn_policy(self):
        planner = self.params['planner_server']['ros__parameters']['GridBased']
        self.assertIs(planner['allow_unknown'], False,
                      '生产导航不得穿越未知区；目标预检负责拒绝 unknown')
        self.assertLessEqual(planner['tolerance'], 0.25,
                             'NavFn 容差必须不超过 DWB 的 xy_goal_tolerance=0.25')
        self.assertAlmostEqual(planner['tolerance'], 0.1)

    def test_direct_launch_default_does_not_bypass_bridge(self):
        text = LAUNCH.read_text()
        self.assertIn("default_value='cmd_vel_auto'", text,
                      '默认输出必须是桥接输入 /cmd_vel_auto，不能是 /cmd_vel 或会自环的 /cmd_vel_nav')
        self.assertNotIn("default_value='cmd_vel_nav'", text)
        self.assertNotIn("default_value='cmd_vel'", text)


if __name__ == '__main__':
    unittest.main()
