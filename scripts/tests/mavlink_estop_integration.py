#!/usr/bin/env python3
"""Real ROS subscriber -> compiled UART bridge -> PTY, never a physical device."""
import os
import pty
import select
import struct
import subprocess
import time
from pathlib import Path
import rclpy
from std_msgs.msg import Bool


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '198', 'Use isolated domain 198'
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    binary = Path(__file__).resolve().parents[2] / 'robot_ws/install/ldc_mavlink_bridge/lib/ldc_mavlink_bridge/mavlink_bridge'
    rclpy.init(); node = rclpy.create_node('mcu_authority_uart_test')
    publisher = node.create_publisher(Bool, '/emergency_stop', 10)
    process = subprocess.Popen([str(binary), '--ros-args', '-p', f'port:={os.ttyname(slave)}'],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    buffer = bytearray()
    def frames(timeout):
        end = time.monotonic()+timeout
        commands=[]
        while time.monotonic()<end:
            if select.select([master],[],[],.02)[0]:
                buffer.extend(os.read(master,4096))
            while len(buffer)>=10:
                if buffer[0] not in (0xfd,0xfe):
                    del buffer[0];continue
                v2=buffer[0]==0xfd;header=10 if v2 else 6
                size=header+buffer[1]+2+(13 if v2 and buffer[2]&1 else 0)
                if len(buffer)<size:break
                frame=bytes(buffer[:size]);del buffer[:size]
                msgid=int.from_bytes(frame[7:10],'little') if v2 else frame[5]
                if msgid==76:
                    payload=frame[header:header+frame[1]].ljust(33,b'\0')
                    commands.append((struct.unpack_from('<H',payload,28)[0],
                                     struct.unpack_from('<f',payload)[0],payload[30],payload[31]))
        return commands
    try:
        end=time.monotonic()+10
        while publisher.get_subscription_count()<1:
            if time.monotonic()>end or process.poll() is not None:raise AssertionError('bridge did not subscribe')
            rclpy.spin_once(node,timeout_sec=.05)
        assert not frames(.3), 'startup must not auto-arm'
        for stop,expected in [(True,0.),(False,1.),(True,0.)]:
            publisher.publish(Bool(data=stop))
            received=frames(.6)
            assert received==[(400,expected,1,1)],received
        assert not frames(.4), 'enable/stop must not be replayed on a timer'
        print('PASS real ROS emergency_stop -> MAVLink ARM/DISARM on PTY; no startup/repeated arm')
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown();os.close(master);os.close(slave)

if __name__=='__main__':main()
