import importlib.util
import json
from pathlib import Path
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

spec = importlib.util.spec_from_file_location('qt_console_control', Path(__file__).resolve().parents[1] / 'qt_console_control.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class QtConsoleTest(unittest.TestCase):
    def setUp(self):
        self.records = []
        self.owner = None
        case = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                case.records.append((self.path, body))
                status, result = 200, {'success': True}
                if self.path.endswith('/claim'):
                    if case.owner and case.owner != body['sessionId']:
                        status, result = 409, {'success': False, 'error': '控制权由其他页面持有'}
                    else:
                        case.owner = body['sessionId']
                elif self.path.endswith('/manual_enable'):
                    result['controlId'] = 'manual-test'
                elif self.path.endswith('/release'):
                    if case.owner == body['sessionId']:
                        case.owner = None
                    else:
                        status, result = 409, {'success': False, 'error': '已失去控制权'}
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(result).encode())

            def log_message(self, *_):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.client = module.ConsoleControl(f'http://127.0.0.1:{self.server.server_port}/api')

    def tearDown(self):
        self.client.close()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_only_explicit_enable_can_send_and_lease_is_released(self):
        with self.assertRaises(RuntimeError):
            self.client.velocity(.1, 0)
        self.assertEqual(self.records, [])
        self.client.enable()
        self.client.velocity(.1, .5)
        self.client.velocity(0, 0)
        self.client.request('navigation/heartbeat')
        commands = [body for path, body in self.records if path.endswith('/velocity')]
        self.assertEqual([body['sequence'] for body in commands], [1, 2])
        self.assertEqual(commands[0]['linearX'], .1)
        self.assertEqual(commands[1]['linearX'], 0)
        self.assertTrue(all(body['controlId'] == 'manual-test' and body['sessionId'] == self.client.session for body in commands))
        self.client.close()
        self.assertIsNone(self.owner)

    def test_foreign_owner_is_never_taken_over_or_stopped(self):
        self.owner = 'browser-owner'
        with self.assertRaisesRegex(RuntimeError, '其他页面'):
            self.client.enable()
        self.client.close()
        self.assertEqual(self.owner, 'browser-owner')
        self.assertEqual([path for path, _ in self.records], ['/api/control/claim'])

    def test_invalid_speed_and_old_owner_release(self):
        self.client.enable()
        with self.assertRaises(ValueError):
            self.client.velocity(float('nan'), 0)
        self.client.velocity(100, -100)
        command = self.records[-1][1]
        self.assertEqual(command['linearX'], .4)
        self.assertEqual(command['angularZ'], -1.2)
        self.owner = 'new-browser-owner'
        self.client.close()
        self.assertEqual(self.owner, 'new-browser-owner')
        self.assertFalse(any(path.endswith('/estop') for path, _ in self.records))


if __name__ == '__main__':
    unittest.main()
