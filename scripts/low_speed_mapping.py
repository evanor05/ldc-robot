#!/usr/bin/env python3
"""Local hold-to-drive mapping panel; defaults to a latched emergency stop."""
from datetime import datetime
from pathlib import Path
import subprocess
import threading
import time

import gi
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

gi.require_version('Gtk', '4.0')
gi.require_version('Gdk', '4.0')
from gi.repository import Gdk, GLib, Gtk

ROOT = Path(__file__).resolve().parents[1]
KEYS = {
    Gdk.KEY_w: (0.05, 0.0), Gdk.KEY_Up: (0.05, 0.0),
    Gdk.KEY_s: (-0.05, 0.0), Gdk.KEY_Down: (-0.05, 0.0),
    Gdk.KEY_a: (0.0, 0.3), Gdk.KEY_Left: (0.0, 0.3),
    Gdk.KEY_d: (0.0, -0.3), Gdk.KEY_Right: (0.0, -0.3),
}


class MappingPanel(Gtk.Application):
    def __init__(self):
        super().__init__(application_id='local.ldc.LowSpeedMapping')
        rclpy.init()
        self.node = rclpy.create_node('ldc_low_speed_mapping')
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_publisher = self.node.create_publisher(Bool, '/emergency_stop', 10)
        self.enabled = False
        self.command = (0.0, 0.0)
        self.scan_count = 0
        self.last_scan = 0.0
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.on_scan, rclpy.qos.qos_profile_sensor_data)
        self.connect('activate', self.activate_panel)

    def on_scan(self, _message):
        self.scan_count += 1
        self.last_scan = time.monotonic()

    def stop(self):
        self.command = (0.0, 0.0)
        self.publisher.publish(Twist())

    def set_estop(self, state):
        self.stop()
        self.enabled = False
        if not state and (time.monotonic() - self.last_scan > 0.8 or
                          self.estop_publisher.get_subscription_count() == 0):
            self.status.set_text('ROS 链路未就绪，等待雷达和急停订阅者后再启用')
            return
        self.estop_publisher.publish(Bool(data=state))
        self.enabled = not state
        self.status.set_text('已发送 ROS 急停请求' if state else
                             '已发送 ROS 解锁请求：按住方向键低速移动，松开即停')

    def start_direction(self, vx, wz):
        if self.enabled:
            self.command = (vx, wz)

    def key_pressed(self, _controller, key, _code, _modifiers):
        key = Gdk.keyval_to_lower(key)
        if key in KEYS:
            self.start_direction(*KEYS[key])
            return True
        if key == Gdk.KEY_space:
            self.stop()
            return True
        if key == Gdk.KEY_Escape:
            self.set_estop(True)
            return True
        return False

    def save_map(self, _button):
        prefix = ROOT / 'maps' / datetime.now().strftime('mapping_%Y%m%d_%H%M%S_%f')
        self.save_button.set_sensitive(False)
        self.status.set_text('正在保存地图……')

        def worker():
            try:
                result = subprocess.run([
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                    '--ros-args', '-p', 'save_map_timeout:=8.0',
                    '-p', 'map_subscribe_transient_local:=true'],
                    capture_output=True, text=True, timeout=15)
                message = '已保存：' + str(prefix.with_suffix('.yaml')) if result.returncode == 0 else (
                    '保存失败：' + (result.stderr or result.stdout)[-300:])
            except Exception as error:
                message = '保存失败：' + str(error)
            GLib.idle_add(self.save_finished, message)

        threading.Thread(target=worker, daemon=True).start()

    def save_finished(self, message):
        self.status.set_text(message)
        self.save_button.set_sensitive(True)
        return False

    def activate_panel(self, _app):
        window = Gtk.ApplicationWindow(application=self, title='LDC · 低速遥控建图')
        window.set_default_size(620, 380)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        for side in ['top', 'bottom', 'start', 'end']:
            getattr(box, 'set_margin_' + side)(18)
        window.set_child(box)
        title = Gtk.Label(label='WASD / 方向键：按住移动，松开停止\n线速度 0.05 m/s · 转速 0.3 rad/s · Space 停车 · Esc 急停')
        box.append(title)
        enable = Gtk.Button(label='启用低速遥控（解除急停）')
        enable.connect('clicked', lambda _b: self.set_estop(False))
        box.append(enable)
        row = Gtk.Box(spacing=8, homogeneous=True)
        for label, command in [('左转 A', (0, .3)), ('前进 W', (.05, 0)),
                               ('后退 S', (-.05, 0)), ('右转 D', (0, -.3))]:
            button = Gtk.Button(label=label)
            gesture = Gtk.GestureClick()
            gesture.connect('pressed', lambda _g, _n, _x, _y, c=command: self.start_direction(*c))
            gesture.connect('released', lambda *_args: self.stop())
            gesture.connect('cancel', lambda *_args: self.stop())
            button.add_controller(gesture)
            row.append(button)
        box.append(row)
        estop = Gtk.Button(label='立即急停（锁定）')
        estop.connect('clicked', lambda _b: self.set_estop(True))
        box.append(estop)
        self.save_button = Gtk.Button(label='保存当前地图')
        self.save_button.connect('clicked', self.save_map)
        box.append(self.save_button)
        self.status = Gtk.Label(label='默认急停，请确认环境后启用遥控', wrap=True, selectable=True)
        self.scan_label = Gtk.Label(label='等待雷达数据')
        box.append(self.status)
        box.append(self.scan_label)
        keys = Gtk.EventControllerKey()
        keys.connect('key-pressed', self.key_pressed)
        keys.connect('key-released', lambda *_args: self.stop())
        window.add_controller(keys)
        window.connect('notify::is-active', lambda win, _p: self.stop() if not win.is_active() else None)
        window.connect('close-request', self.close_panel)
        GLib.timeout_add(50, self.tick)
        self.set_estop(True)
        window.present()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.enabled and time.monotonic() - self.last_scan > 0.8:
            self.set_estop(True)
            self.status.set_text('雷达数据中断，已禁用遥控并请求急停，请检查连接')
        message = Twist()
        if self.enabled:
            message.linear.x, message.angular.z = self.command
        self.publisher.publish(message)
        self.scan_label.set_text(f'已接收雷达 {self.scan_count} 帧；建图正在后台更新')
        return True

    def close_panel(self, _window):
        self.set_estop(True)
        return False


if __name__ == '__main__':
    panel = MappingPanel()
    try:
        panel.run(None)
    finally:
        panel.set_estop(True)
        panel.node.destroy_node()
        rclpy.shutdown()
