import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer


class TaskStartupTests(unittest.TestCase):
    def test_slow_start_keeps_lease_alive_and_releases_without_enabling_motion(self):
        records=[];case=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                body=json.dumps(dict(isRunning=False,checks={key:dict(ok=True) for key in ('lidar','odometry','tf')})).encode()
                self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def do_POST(self):
                data=json.loads(self.rfile.read(int(self.headers['Content-Length'])));records.append((self.path,data))
                if self.path.endswith('/mapping'):time.sleep(2.2)
                body=json.dumps({'success':True}).encode();self.send_response(200);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            def log_message(self,*args):pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            script=Path(__file__).resolve().parents[1]/'runtime/task.py'
            result=subprocess.run([sys.executable,str(script),'mapping'],env=dict(os.environ,ROBOT_API_PORT=str(server.server_port)),capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            paths=[p for p,_ in records];self.assertGreaterEqual(paths.count('/api/navigation/heartbeat'),3)
            self.assertEqual(paths[0],'/api/control/claim');self.assertEqual(paths[-1],'/api/control/release')
            self.assertFalse(any('enable' in p or 'velocity' in p or 'goals' in p for p in paths))
            self.assertEqual(len({d['sessionId'] for _,d in records}),1)
        finally:server.shutdown();server.server_close();thread.join()
