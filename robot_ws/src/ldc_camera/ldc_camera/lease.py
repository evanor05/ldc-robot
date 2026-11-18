"""One camera acquisition owner per ROS domain and image topic."""
import fcntl
import hashlib
import os
from pathlib import Path


class CameraLease:
    def __init__(self, topic, directory=None):
        directory=Path(directory or Path(os.environ.get('XDG_RUNTIME_DIR','/tmp'))/f'ldc-camera-{os.getuid()}')
        directory.mkdir(mode=0o700,parents=True,exist_ok=True)
        key=hashlib.sha256((os.environ.get('ROS_DOMAIN_ID','0')+'|'+topic).encode()).hexdigest()[:20]
        self.handle=(directory/(key+'.lock')).open('a')
        try:fcntl.flock(self.handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            self.handle.close()
            raise RuntimeError('该 Domain/图像话题已有摄像头采集者；保留原采集，单独启动 YOLO 或先停止原视觉会话') from None

    def close(self):
        self.handle.close()
