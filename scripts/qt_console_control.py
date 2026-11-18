#!/usr/bin/env python3
"""Qt teleop adapter: HTTP lease/velocity authority, never publishes /cmd_vel."""
import argparse
import json
import math
import os
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid


class ConsoleControl:
    def __init__(self, api):
        self.api = api.rstrip('/')
        self.session = 'qt-' + uuid.uuid4().hex
        self.control_id = None
        self.sequence = 0
        self.claimed = False
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(self, endpoint, **data):
        body = {'sessionId': self.session, **data}
        if endpoint not in ('navigation/heartbeat', 'navigation/velocity'):
            body['requestId'] = uuid.uuid4().hex
        req = urllib.request.Request(self.api + '/' + endpoint,
            data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(req, timeout=6 if endpoint == 'control/manual_enable' else 1) as reply:
                result = json.load(reply)
        except urllib.error.HTTPError as error:
            try:
                detail = json.load(error).get('error', str(error))
            except (ValueError, OSError):
                detail = str(error)
            finally:
                error.close()
            raise RuntimeError(detail) from error
        if not result.get('success'):
            raise RuntimeError(result.get('error', '控制请求失败'))
        return result

    def enable(self):
        self.request('control/claim')  # Never takes over another owner's lease.
        self.claimed = True
        result = self.request('control/manual_enable')
        self.control_id = result['controlId']

    def velocity(self, linear, angular):
        if not self.control_id:
            raise RuntimeError('Qt 遥控未启用')
        if not all(math.isfinite(x) for x in (linear, angular)):
            raise ValueError('速度必须是有限数值')
        self.sequence += 1
        return self.request('navigation/velocity', controlId=self.control_id,
            sequence=self.sequence, linearX=max(-.4, min(.4, linear)),
            angularZ=max(-1.2, min(1.2, angular)))

    def close(self):
        # release only affects our own lease, even if a browser has since taken over.
        self.control_id = None
        if self.claimed:
            try:
                self.request('control/release')
            except (RuntimeError, OSError, ValueError) as error:
                print('Qt 释放控制权:', error, flush=True)
            self.claimed = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--api', default='http://127.0.0.1:3000/api')
    parser.add_argument('--enable', action='store_true', help='明确取得空闲控制权并启用 Qt 遥控；默认只观察')
    parser.add_argument('--camera-topic', default='/vision/annotated/compressed', help='默认观察 YOLO 叠框图；原图用 /camera/image_raw/compressed')
    args = parser.parse_args()
    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node('qt_console_adapter')
    topic = '/qt_console/input_' + uuid.uuid4().hex
    lock = threading.Lock()
    latest = [0., 0., 0.]

    def command(msg):
        with lock:
            latest[:] = [msg.linear.x, msg.angular.z, time.monotonic()]

    subscription = node.create_subscription(Twist, topic, command, 1)
    client = ConsoleControl(args.api)
    child = None
    stop = threading.Event()
    spin_thread = None

    def interrupt(*_):
        stop.set()

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        if args.enable:
            client.enable()
        env = dict(os.environ, LDC_CONSOLE_MANAGED='manual' if args.enable else 'observe')
        # Every Qt control publication is isolated. Only this adapter forwards teleop.
        child = subprocess.Popen(['ros2', 'run', 'ldc_qt_gui', 'ldc_qt_gui',
            '--ros-args', '-p', 'camera_topic:=' + args.camera_topic, '-p', 'cmd_vel_topic:=' + topic, '-p', 'goal_topic:=' + topic + '/unused_goal',
            '-r', '/initialpose:=' + topic + '/unused_initialpose'], env=env, start_new_session=True)

        def spin():
            while not stop.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.05)
        spin_thread = threading.Thread(target=spin, daemon=True)
        spin_thread.start()
        print('Qt 遥控已启用：W/S 前后，A/D 左右，松开或空格停车。' if args.enable else
              'Qt 只读观察。驾驶请先在网页释放控制权，再用 start_qt.sh --real --enable 启动。', flush=True)
        print('建图、保存、定位、导航、急停由网页统一管理；本窗口的任务启动按钮禁用。', flush=True)
        heartbeat_at = 0.
        first_frame_deadline = time.monotonic() + 5
        while not stop.is_set() and child.poll() is None:
            now = time.monotonic()
            if args.enable:
                if now - heartbeat_at >= .5:
                    client.request('navigation/heartbeat')
                    heartbeat_at = now
                with lock:
                    linear, angular, received_at = latest
                # A frozen/closed GUI cannot leave its last nonzero command running.
                if received_at and now - received_at > .25:
                    raise RuntimeError('Qt 窗口数据中断，已锁定；重新启动才能启用')
                if not received_at:
                    if now > first_frame_deadline:
                        raise RuntimeError('Qt 窗口未产生输入心跳，请检查启动日志')
                    linear = angular = 0.
                client.velocity(linear, angular)
            stop.wait(.05)
        return child.returncode or 0
    except (RuntimeError, OSError, ValueError) as error:
        print('Qt 控制已停止：', error, flush=True)
        print('检查传感器/控制权；恢复后需重新运行 --enable，不自动恢复运动。', flush=True)
        return 1
    finally:
        stop.set()
        client.close()
        if child and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        if spin_thread:
            spin_thread.join(timeout=1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
