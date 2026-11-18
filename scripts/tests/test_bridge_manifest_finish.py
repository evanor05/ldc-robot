"""回归：manifest 收尾事件不得被 record() 关键字冲突吞掉（D 发现，lead 核实）。

bridge.manifest_finish 曾以 record('run_manifest_finished', <位置 reason>, reason=...)
触发 TypeError（record 签名 (event_type, reason='', **extra)），被 except 吞掉后
每次 stop/shutdown 丢失 run_manifest_finished、误记 run_manifest_failed。
"""
import unittest
from unittest.mock import Mock, patch

import rclpy
from scripts.navigation_bridge import Bridge


class ManifestFinishEventTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Bridge(journal_path=':memory:')
        self.node.respond = lambda request, **reply: None
        self.finish_calls = []
        stub = Mock()
        stub.finish_run = lambda run_id, status, reason, snapshots=None: self.finish_calls.append(
            (run_id, status, reason))
        stub.update_manifest = Mock()
        self.patcher = patch('scripts.navigation_bridge.run_manifest', stub)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.addCleanup(self.node.destroy_node)

    def events(self, kind):
        return [e for e in self.node.events if e['type'] == kind]

    def test_stop_records_manifest_finished_with_reason(self):
        self.node.manifest_run_id = 'run-stop-test'
        self.node.process({'op': 'stop'})
        finished = self.events('run_manifest_finished')
        self.assertEqual(len(finished), 1, 'stop 必须留下 run_manifest_finished 事件')
        self.assertEqual(finished[0]['reason'], '运行清单已结束: aborted')
        self.assertEqual(finished[0]['finishReason'], '用户停止导航')
        self.assertEqual(finished[0]['runId'], 'run-stop-test')
        self.assertFalse(self.events('run_manifest_failed'),
                         '收尾成功不得误记 run_manifest_failed')
        self.assertEqual(self.finish_calls, [('run-stop-test', 'aborted', '用户停止导航')])

    def test_shutdown_path_records_manifest_finished_with_reason(self):
        # main() finally 的退出路径同款调用
        self.node.manifest_run_id = 'run-exit-test'
        self.node.manifest_finish('aborted', '导航桥接进程退出')
        finished = self.events('run_manifest_finished')
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0]['finishReason'], '导航桥接进程退出')
        self.assertFalse(self.events('run_manifest_failed'))

    def test_estop_records_manifest_finished(self):
        self.node.manifest_run_id = 'run-estop-test'
        self.node.process({'op': 'estop'})
        finished = self.events('run_manifest_finished')
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0]['finishReason'], '已急停')
        self.assertFalse(self.events('run_manifest_failed'))


if __name__ == '__main__':
    unittest.main()