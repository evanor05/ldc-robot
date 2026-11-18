import os
from pathlib import Path
import subprocess
import threading
from .ipc import read_packet, write_packet

WORKER_RESTART_BASE_SECONDS = 1.0
WORKER_RESTART_MAX_SECONDS = 30.0


class WorkerFrameError(RuntimeError):
    """One frame failed inside worker; the worker process is still usable."""


def worker_restart_delay(failures):
    """Exponential backoff for consecutive worker-level failures, capped."""
    failures = max(1, int(failures))
    return min(WORKER_RESTART_MAX_SECONDS, WORKER_RESTART_BASE_SECONDS * 2 ** (failures - 1))


class WorkerClient:
    def __init__(self, python, model, device, imgsz, confidence, worker_script=None,
                 color_mode='opencv', color_label_map=''):
        self.process = None
        self.closed = False
        self.lock = threading.Lock()
        self.worker_script = Path(worker_script) if worker_script else Path(__file__).with_name('worker.py')
        self.arguments = [python, '-u', str(self.worker_script),
                          '--model', model, '--device', device, '--imgsz', str(imgsz),
                          '--confidence', str(confidence), '--color-mode', color_mode]
        if color_label_map:
            self.arguments += ['--color-label-map', color_label_map]

    def start(self):
        with self.lock:
            if self.closed: raise RuntimeError('worker 已关闭')
            self.process = subprocess.Popen(self.arguments, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            process = self.process
        os.set_blocking(process.stdin.fileno(), False)
        metadata, _ = read_packet(process.stdout.fileno(), 45)
        if not metadata.get('ready'):
            raise RuntimeError(metadata.get('error', '模型初始化失败'))
        return metadata

    def infer(self, metadata, image):
        process = self.process
        write_packet(process.stdin.fileno(), metadata, image)
        result, _ = read_packet(process.stdout.fileno(), 5)
        if 'error' in result:
            # A frame-level error (bad JPEG / per-frame predict failure) is
            # recoverable: the worker loop keeps running.  EOF/timeout while
            # reading is a worker-level failure and is raised as-is.
            raise WorkerFrameError(result['error'])
        return result

    def close(self):
        with self.lock:
            self.closed = True
            process, self.process = self.process, None
        if process:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=2)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=2)
            for stream in (process.stdin, process.stdout):
                try: stream.close()
                except OSError: pass
