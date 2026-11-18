"""navigation_profile.py prefix 参数等价性测试（裁决 Q2(a) / B 契约）。

覆盖 B（nav-runtime）提出的交接硬条件：
- prefix='' 与缺省：服务全名逐字节一致，snapshot 结构/语义一致；
- prefix 规范化：去尾 '/'、非空保证前导 '/'，嵌套 costmap 键原样拼接；
- 结果键不带前缀（values/observed 按 PARAMETERS 键）；
- close()：取消 pending + 销毁 client + generation 递增，
  **迟到回调不得写入已 close 的快照**（真实 GetParameters 服务注入）；
- close() 后 poll() no-op；reset() 保持现语义（不换 client）。

需要 ROS 环境（source /opt/ros/lyrical/setup.bash），私有域由 pytest 进程环境决定；
本测试不发布任何运动指令，不触碰 /cmd_vel。
"""
import importlib.util
from pathlib import Path
import threading
import time
import unittest

import rclpy
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters

SCRIPTS = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_profile_prefix', SCRIPTS / 'navigation_profile.py')
profile_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile_module)
RuntimeProfile = profile_module.RuntimeProfile


class PrefixEquivalenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('profile_prefix_test')
        self.profiles = []

    def tearDown(self):
        for profile in self.profiles:
            try:
                profile.close()
            except Exception:
                pass
        self.node.destroy_node()

    def make(self, prefix=None):
        profile = RuntimeProfile(self.node) if prefix is None else RuntimeProfile(self.node, prefix=prefix)
        self.profiles.append(profile)
        return profile

    def test_default_and_empty_prefix_names_byte_identical(self):
        default = self.make()
        empty = self.make(prefix='')
        for name in RuntimeProfile.PARAMETERS:
            expected = f'/{name}/get_parameters'
            self.assertEqual(default.clients[name].srv_name, expected)
            self.assertEqual(empty.clients[name].srv_name, expected)
            self.assertEqual(default.clients[name].srv_name, empty.clients[name].srv_name)

    def test_prefix_normalization(self):
        cases = [('ldc/nav', '/ldc/nav'), ('/ldc/nav/', '/ldc/nav'),
                 ('ldc/nav/', '/ldc/nav'), ('/ldc/nav', '/ldc/nav'),
                 ('/ldc/nav/b0/r1', '/ldc/nav/b0/r1')]
        for raw, normalized in cases:
            profile = self.make(prefix=raw)
            self.assertEqual(profile.prefix, normalized)
            for name in RuntimeProfile.PARAMETERS:
                self.assertEqual(profile.clients[name].srv_name,
                                 f'{normalized}/{name}/get_parameters')
            self.assertEqual(
                profile.clients['local_costmap/local_costmap'].srv_name,
                f'{normalized}/local_costmap/local_costmap/get_parameters')

    def test_snapshot_structure_unchanged(self):
        default = self.make().snapshot()
        empty = self.make(prefix='').snapshot()
        self.assertEqual(set(default), set(empty))
        for key in ('source', 'parameters', 'sha256', 'fresh', 'observedNodes',
                    'expectedNodes', 'complete', 'firmwareVerified'):
            self.assertEqual(default[key], empty[key], key)
        self.assertEqual(default['expectedNodes'], list(RuntimeProfile.PARAMETERS))

    def test_result_keys_have_no_prefix(self):
        # 用真实服务喂一个带前缀的 profile，断言 values 键仍为 PARAMETERS 原键。
        gate = threading.Event()

        def handler(request, response):
            gate.wait(2.0)
            response.values = [ParameterValue() for _ in request.names]
            return response

        self.node.create_service(GetParameters, '/snap/planner_server/get_parameters', handler)
        profile = self.make(prefix='/snap')
        deadline = time.monotonic() + 3
        while not profile.clients['planner_server'].service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        profile.poll()
        self.assertIn('planner_server', profile.pending)
        gate.set()
        deadline = time.monotonic() + 3
        while 'planner_server' not in profile.values and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertIn('planner_server', profile.values)
        self.assertNotIn('/snap/planner_server', profile.values)
        self.assertIn('planner_server', profile.observed)

    def test_close_rejects_late_callback(self):
        gate = threading.Event()

        def handler(request, response):
            gate.wait(2.0)
            response.values = [ParameterValue() for _ in request.names]
            return response

        self.node.create_service(GetParameters, '/late/planner_server/get_parameters', handler)
        profile = self.make(prefix='/late')
        deadline = time.monotonic() + 3
        while not profile.clients['planner_server'].service_is_ready() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        profile.poll()
        self.assertIn('planner_server', profile.pending)
        old_generation = profile.generation
        profile.close()
        self.assertGreater(profile.generation, old_generation)
        self.assertEqual(profile.pending, {})
        self.assertEqual(profile.clients, {})
        gate.set()  # 迟到响应此刻才放行
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(profile.values, {})
        self.assertEqual(profile.observed, {})

    def test_poll_after_close_is_noop(self):
        profile = self.make()
        profile.close()
        profile.poll()
        self.assertEqual(profile.pending, {})
        self.assertEqual(profile.clients, {})
        self.assertEqual(profile.values, {})

    def test_reset_keeps_clients(self):
        profile = self.make()
        clients = profile.clients
        generation = profile.generation
        profile.reset()
        self.assertIs(profile.clients, clients)
        self.assertGreater(profile.generation, generation)
        self.assertEqual(profile.values, {})


if __name__ == '__main__':
    unittest.main()