"""Loopback-only latest-frame fanout. Slow viewers cannot block ROS or inference."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import socket
import threading
import time


class PreviewGateway:
    def __init__(self, owner, port):
        self.owner = owner
        streams = threading.BoundedSemaphore(4)
        requests = threading.BoundedSemaphore(8)

        class Server(ThreadingHTTPServer):
            daemon_threads = True
            allow_reuse_address = True
            def process_request(self, request, address):
                request.settimeout(1)
                request.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                if not requests.acquire(blocking=False):
                    request.close(); return
                super().process_request(request, address)
            def process_request_thread(self, request, address):
                try: super().process_request_thread(request, address)
                finally: requests.release()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args): pass
            def reply(self, value, code=200):
                data = json.dumps(value, allow_nan=False).encode()
                self.send_response(code); self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(data))); self.send_header('Cache-Control','no-store')
                self.end_headers(); self.wfile.write(data)
            def do_POST(self):
                if self.path == '/shutdown':
                    self.reply({'success':True})
                    owner.request_shutdown()
                    return
                if self.path != '/filter': return self.reply({'error':'not found'},404)
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=256: raise ValueError('invalid body')
                    mode=json.loads(self.rfile.read(length))['mode']
                    owner.set_filter(mode)
                    self.reply({'success':True})
                except (ValueError, KeyError, TypeError) as error: self.reply({'error':str(error)},400)
            def do_GET(self):
                if self.path == '/status': return self.reply(owner.status())
                if self.path not in ('/raw.mjpg','/annotated.mjpg'): return self.reply({'error':'not found'},404)
                if not streams.acquire(blocking=False): return self.reply({'error':'preview client limit'},429)
                kind='raw' if self.path=='/raw.mjpg' else 'annotated'
                try:
                    self.send_response(200)
                    self.send_header('Content-Type','multipart/x-mixed-replace; boundary=ldcframe')
                    self.send_header('Cache-Control','no-store'); self.end_headers()
                    sequence=None
                    while not owner.stop.is_set():
                        frame=owner.preview(kind)
                        if frame is None:
                            # Close a stale stream: browsers must not silently
                            # keep an old image while the status reports failure.
                            if sequence is not None: break
                            time.sleep(.05);continue
                        if sequence==frame[0]: time.sleep(.01);continue
                        sequence,jpeg=frame
                        self.wfile.write(b'--ldcframe\r\nContent-Type: image/jpeg\r\nContent-Length: '+
                                         str(len(jpeg)).encode()+b'\r\nX-Frame-Id: '+str(sequence).encode()+
                                         b'\r\n\r\n'+jpeg+b'\r\n')
                        self.wfile.flush()
                except (OSError, socket.timeout): pass
                finally: streams.release()
        self.server = Server(('127.0.0.1',port),Handler)
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=2)
