"""Bridge construction is passive; stdin EOF/failure must still request shutdown."""
import io
import os
from pathlib import Path
import tempfile
import uuid
import queue
import unittest
from unittest.mock import patch
import rclpy
from scripts.navigation_bridge import Bridge


class BrokenStream:
    def __iter__(self):
        raise OSError('input pipe failed')


class BridgeInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()

    def test_default_construction_does_not_start_stdin_thread(self):
        with patch('scripts.navigation_bridge.threading.Thread') as thread:
            node=Bridge(journal_path=':memory:')
            try:self.assertFalse(thread.called)
            finally:node.destroy_node()

    def test_explicit_stream_reads_objects_and_eof_once(self):
        node=Bridge(journal_path=':memory:', command_stream=io.StringIO('{"op":"status"}\ninvalid\n[]\nnull\n'))
        try:
            node.command_reader.join(timeout=2)
            self.assertFalse(node.command_reader.is_alive())
            items=[]
            while not node.commands.empty():items.append(node.commands.get_nowait())
            self.assertEqual(items,[{'op':'status'},{'op':'shutdown'}])
        finally:node.destroy_node()

    def test_input_error_is_handled_and_requests_shutdown(self):
        node=Bridge(journal_path=':memory:')
        try:
            node.read_commands(BrokenStream())
            self.assertEqual(node.commands.get_nowait(),{'op':'shutdown'})
            with self.assertRaises(queue.Empty):node.commands.get_nowait()
        finally:node.destroy_node()

    def test_eof_goes_through_tick_and_publishes_only_zero(self):
        from unittest.mock import Mock
        node=Bridge(journal_path=':memory:', command_stream=io.StringIO(''))
        try:
            node.command_reader.join(timeout=2)
            node.velocity=Mock()
            node.mode='manual'
            node.tick()
            self.assertTrue(node.shutdown_requested)
            self.assertEqual(node.mode,'locked')
            self.assertTrue(node.velocity.publish.called)
            for call in node.velocity.publish.call_args_list:
                message=call.args[0]
                self.assertEqual(message.linear.x,0.)
                self.assertEqual(message.angular.z,0.)
        finally:node.destroy_node()

    def test_explicit_memory_store_ignores_external_journal_environment(self):
        # A test runner or shell may inherit the production journal override.
        with tempfile.TemporaryDirectory() as folder:
            external = Path(folder) / 'production.sqlite3'
            original = b'production evidence: must remain unchanged'
            external.write_bytes(original)
            with patch.dict(os.environ, {'LDC_ACTION_JOURNAL_PATH': str(external)}):
                node = Bridge(journal_path=':memory:')
                try:
                    identifier = uuid.uuid4().hex
                    node.action_journal.prepare(identifier, {'source': 'unit-test'})
                    self.assertEqual(node.action_journal.pending()[0]['goalUUID'], identifier)
                    self.assertEqual(node.action_journal.filename, ':memory:')
                finally:
                    node.destroy_node()
            self.assertEqual(external.read_bytes(), original)
            self.assertEqual([p.name for p in Path(folder).iterdir()], ['production.sqlite3'])

    def test_main_explicitly_passes_stdin_and_locks_on_exit(self):
        from scripts import navigation_bridge as module
        with patch.object(module,'Bridge') as factory, patch.object(module.rclpy,'init'), patch.object(module.rclpy,'ok',return_value=False):
            module.main()
            factory.assert_called_once_with(command_stream=module.sys.stdin)
            factory.return_value.lock.assert_called_once()
