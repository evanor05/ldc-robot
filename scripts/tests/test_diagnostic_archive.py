import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import queue
import urllib.parse
from scripts.diagnostics.storage import Archive, TYPES, COMMAND_TOPICS, report, atomic_json, open_archive
from scripts.diagnostics.storage import verify_archive
from scripts.diagnostics.capture import backend_snapshots, capture


class ArchiveTests(unittest.TestCase):
    def test_replay_requires_finalized_unchanged_database(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)/'capture'; archive = Archive(folder, 1024*1024)
            digest = archive.close()
            atomic_json(folder/'capture.json', dict(state='recording', sha256=digest))
            with self.assertRaisesRegex(ValueError, '完整结束'):
                verify_archive(folder)
            atomic_json(folder/'capture.json', dict(state='empty', sha256=digest))
            self.assertEqual(verify_archive(folder)['sha256'], digest)
            with (folder/'recording.sqlite3').open('ab') as handle:
                handle.write(b'changed')
            with self.assertRaisesRegex(ValueError, '校验失败'):
                verify_archive(folder)

    def test_event_cursor_does_not_skip_when_snapshot_queue_is_full(self):
        from contextlib import nullcontext
        from io import BytesIO
        cursors = []
        class Stop:
            count = 0
            def is_set(self): return self.count >= 3
            def wait(self, _): self.count += 1
        class Output:
            count = 0
            def put_nowait(self, value):
                self.count += 1
                if self.count == 1: raise queue.Full()
        class Opener:
            def open(self, url, **_):
                data = {}
                if '/events' in url:
                    cursor = int(urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)['since'][0])
                    cursors.append(cursor)
                    data = dict(events=[dict(seq=cursor+1)], latest=cursor+1)
                return nullcontext(BytesIO(json.dumps(data).encode()))
        with patch('scripts.diagnostics.capture.urllib.request.build_opener', return_value=Opener()):
            backend_snapshots(3000, Stop(), Output())
        self.assertEqual(cursors, [0, 0, 1])

    def test_storage_error_is_failed_not_successful_capture(self):
        def snapshots(_port, _stop, output): output.put({})
        with tempfile.TemporaryDirectory() as temporary, \
                patch('scripts.diagnostics.capture.backend_snapshots', side_effect=snapshots), \
                patch.object(Archive, 'event', side_effect=OSError('injected disk full')):
            result = capture(Path(temporary)/'capture', .5, 1024*1024, 0, 3000)
            self.assertEqual(result['state'], 'failed')
            self.assertIn('disk full', result['error'])

    def test_capacity_stops_without_overwriting_history(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)/'capture'
            archive = Archive(folder, 1024*1024)
            archive.append('/scan', 100, 0., b'first-frame')
            with self.assertRaises(OverflowError):
                archive.append('/scan', 101, .1, bytes(1024*1024))
            digest = archive.close()
            self.assertEqual(len(digest), 64)
            self.assertLess((folder/'recording.sqlite3').stat().st_size, 1024*1024)
            with open_archive(folder) as db:
                self.assertEqual(db.execute('SELECT cdr FROM messages').fetchone()[0], b'first-frame')
            with self.assertRaises(FileExistsError):
                Archive(folder, 1024*1024)

    def test_full_archive_and_bounded_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)/'capture'; archive = Archive(folder, 4*1024*1024)
            archive.append('/cmd_vel', 100, 0., b'observed-only')
            for i in range(150):
                archive.event(i, dict(events={'events': [dict(seq=i, reason='example')]}))
            archive.close()
            atomic_json(folder/'capture.json', dict(state='complete'))
            result = report(folder)
            self.assertEqual(result['snapshotCount'], 150)
            self.assertEqual(len(result['snapshots']), 120)
            self.assertEqual(result['snapshots'][0]['elapsed'], 30)
            self.assertIn('/cmd_vel', COMMAND_TOPICS)
            self.assertIn('/initialpose', COMMAND_TOPICS)

    def test_ros_nonfinite_scan_survives_exact_cdr_storage(self):
        from sensor_msgs.msg import LaserScan
        from rclpy.serialization import serialize_message, deserialize_message
        import math
        msg = LaserScan(); msg.ranges = [float('inf'), float('nan'), 1.]
        with tempfile.TemporaryDirectory() as temporary:
            archive = Archive(Path(temporary)/'capture', 1024*1024)
            archive.append('/scan', 100, 0., serialize_message(msg))
            archive.close()
            with open_archive(Path(temporary)/'capture') as db:
                decoded = deserialize_message(db.execute('SELECT cdr FROM messages').fetchone()[0], LaserScan)
            self.assertTrue(math.isinf(decoded.ranges[0]))
            self.assertTrue(math.isnan(decoded.ranges[1]))


if __name__ == '__main__':
    unittest.main()
