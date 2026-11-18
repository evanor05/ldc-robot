"""Pure unit tests for frontier exploration and velocity gating.

No ROS graph, action server or vehicle is started.  Run with:
    cd robot_ws/src/ldc_exploration
    python3 -m unittest discover -s test -p 'test_*.py'
"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ldc_exploration.frontier_core import (  # noqa: E402
    BlacklistEntry, FrontierBlacklist, MapInfo, find_frontiers, goal_yaw,
    map_is_explorable, map_stats, quaternion_from_yaw, select_frontier,
)
from ldc_exploration.gate_core import decide_velocity  # noqa: E402


def blank_grid(width=20, height=20):
    return [-1] * (width * height)


def fill_free(data, width, x0, y0, x1, y1):
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            data[y * width + x] = 0


def fill_occupied(data, width, x, y):
    data[y * width + x] = 100


class FrontierCoreTests(unittest.TestCase):
    INFO = MapInfo(20, 20, 0.05, 0.0, 0.0)

    def test_map_stats_and_empty_map_wait(self):
        data = blank_grid()
        stats = map_stats(data, self.INFO)
        self.assertEqual(stats.free, 0)
        self.assertEqual(stats.unknown, 400)
        self.assertFalse(map_is_explorable(stats), '全未知图必须保持 idle')

        fill_free(data, 20, 2, 2, 8, 8)  # 49 free
        stats = map_stats(data, self.INFO)
        self.assertEqual(stats.free, 49)
        self.assertFalse(map_is_explorable(stats), 'free<50 必须保持 idle')

        fill_free(data, 20, 9, 9, 15, 15)
        stats = map_stats(data, self.INFO)
        self.assertEqual(stats.free, 98)
        self.assertTrue(map_is_explorable(stats))
        self.assertFalse(map_is_explorable(stats, max_unknown_ratio=0.5))

    def test_frontier_detection_and_8_neighbour_cluster(self):
        data = blank_grid()
        fill_free(data, 20, 4, 4, 12, 12)
        clusters = find_frontiers(data, self.INFO, (0.1, 0.1),
                                  min_frontier_cells=12, robot_radius=0.0, goal_clearance=0.0)
        self.assertEqual(len(clusters), 1)
        # Border of a 9x9 free rectangle: 9*4 - 4 = 32 frontier cells.
        self.assertEqual(clusters[0].size, 32)
        self.assertAlmostEqual(clusters[0].target[0], 0.425, places=6)
        self.assertAlmostEqual(clusters[0].target[1], 0.425, places=6)

    def test_min_cells_and_distance_filters(self):
        data = blank_grid()
        fill_free(data, 20, 4, 4, 12, 12)
        self.assertEqual(find_frontiers(data, self.INFO, (0.1, 0.1),
                                        min_frontier_cells=40, robot_radius=0.0), [])
        self.assertEqual(find_frontiers(data, self.INFO, (0.1, 0.1),
                                        min_frontier_cells=12, min_goal_distance=10.0,
                                        robot_radius=0.0), [])
        self.assertEqual(find_frontiers(data, self.INFO, (0.1, 0.1),
                                        min_frontier_cells=12, max_goal_distance=0.1,
                                        robot_radius=0.0), [])
        self.assertEqual(find_frontiers(data, self.INFO, (0.1, 0.1),
                                        min_frontier_cells=12, min_robot_distance=100.0,
                                        robot_radius=0.0), [])

    def test_blacklist_radius_and_expiry(self):
        data = blank_grid()
        fill_free(data, 20, 4, 4, 12, 12)
        clusters = find_frontiers(data, self.INFO, (0.1, 0.1), robot_radius=0.0, goal_clearance=0.0)
        target = clusters[0].target
        blacklist = FrontierBlacklist(radius=0.2)
        self.assertFalse(blacklist.contains(target[0] + 1.0, target[1], 10.0))
        blacklist.add(target[0], target[1], 10.0, seconds=5.0)
        self.assertTrue(blacklist.contains(target[0] + 0.1, target[1], 10.0))
        self.assertFalse(blacklist.contains(target[0] + 1.0, target[1], 10.0))
        self.assertEqual(blacklist.count(10.0), 1)
        # A nearby but outside-radius goal is not blocked.
        near = type(clusters[0])(clusters[0].cells, (target[0] + 1.0, target[1]),
                                 clusters[0].centroid, clusters[0].distance)
        self.assertIs(select_frontier([near], blacklist, 10.0), near)
        self.assertIsNone(select_frontier([clusters[0]], blacklist, 10.0))
        self.assertEqual(blacklist.count(16.0), 0)
        self.assertIs(select_frontier(clusters, blacklist, 16.0), clusters[0])

    def test_goal_yaw_and_quaternion(self):
        self.assertAlmostEqual(goal_yaw((0.0, 0.0), (1.0, 0.0)), 0.0)
        self.assertAlmostEqual(goal_yaw((0.0, 0.0), (0.0, 1.0)), 1.5707963267948966)
        z, w = quaternion_from_yaw(0.0)
        self.assertAlmostEqual(z, 0.0)
        self.assertAlmostEqual(w, 1.0)

    def test_clearance_rejects_frontier_behind_wall(self):
        data = blank_grid()
        fill_free(data, 20, 4, 4, 12, 12)
        # Put an occupied cell in the middle of the open area; frontier targets
        # remain around the border, so use a small robot_radius to keep them.
        fill_occupied(data, 20, 8, 8)
        with_clearance = find_frontiers(data, self.INFO, (0.1, 0.1),
                                        min_frontier_cells=12, robot_radius=1.0)
        self.assertEqual(with_clearance, [], '目标周围有障碍时应被 clearance 过滤')


class SafeGoalTests(unittest.TestCase):
    def test_goal_is_inside_free_space_not_on_frontier_boundary(self):
        info = MapInfo(60, 60, 0.05, 0.0, 0.0)
        data = blank_grid(60, 60)
        fill_free(data, 60, 10, 10, 50, 50)  # 41x41 free, 82 frontier cells
        clusters = find_frontiers(data, info, (0.1, 0.1), min_frontier_cells=12,
                                  robot_radius=0.15, goal_clearance=0.35)
        self.assertTrue(clusters)
        # Cluster border is x=0.5..2.55; a safe goal must be pulled inward by >=0.35 m.
        for cluster in clusters:
            x, y = cluster.target
            self.assertGreater(x, 0.5 + 0.30)
            self.assertGreater(y, 0.5 + 0.30)
            self.assertLess(x, 2.55 - 0.30)
            self.assertLess(y, 2.55 - 0.30)


class GateCoreTests(unittest.TestCase):
    def test_safe_command_is_clipped(self):
        decision = decide_velocity(0.5, -2.0, 0.1, False, 0.1)
        self.assertTrue(decision.ok)
        self.assertAlmostEqual(decision.linear, 0.15)
        self.assertAlmostEqual(decision.angular, -0.8)
        self.assertEqual(decision.reason, 'ok')

    def test_zero_min_linear_blocks_reverse(self):
        decision = decide_velocity(-0.2, 0.0, 0.1, False, 0.1, min_linear=0.0)
        self.assertTrue(decision.ok)
        self.assertEqual(decision.linear, 0.0)
        reverse = decide_velocity(-0.2, 0.0, 0.1, False, 0.1, min_linear=-0.1)
        self.assertAlmostEqual(reverse.linear, -0.1)

    def test_estop_and_scan_timeout_and_command_timeout_zero(self):
        for kwargs in (
            dict(scan_age=0.1, estop=True, command_age=0.1),
            dict(scan_age=None, estop=False, command_age=0.1),
            dict(scan_age=1.1, estop=False, command_age=0.1),
            dict(scan_age=0.1, estop=False, command_age=0.6),
        ):
            decision = decide_velocity(0.2, 0.2, **kwargs)
            self.assertFalse(decision.ok)
            self.assertEqual((decision.linear, decision.angular), (0.0, 0.0))

    def test_nonfinite_is_zero(self):
        decision = decide_velocity(float('nan'), 0.1, 0.1, False, 0.1)
        self.assertFalse(decision.ok)
        self.assertEqual((decision.linear, decision.angular), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
