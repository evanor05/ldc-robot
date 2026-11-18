"""Passive ROS recorder for calibration; does not publish control or start drivers."""
import json
import math
from pathlib import Path
import time


def finite_tree(value):
    if isinstance(value,dict):return {key:finite_tree(v) for key,v in value.items()}
    if isinstance(value,(tuple,list)):return [finite_tree(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


def record(output,seconds=30,camera_every=0):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu,LaserScan,CompressedImage,CameraInfo
    from tf2_msgs.msg import TFMessage
    from rosidl_runtime_py.convert import message_to_ordereddict
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    images=output/'images';images.mkdir()
    rclpy.init();node=rclpy.create_node('ldc_calibration_observer')
    counts={};image_at=0.;last_stamp=None
    log=(output/'samples.jsonl').open('x')
    def receive(topic):
        def callback(msg):
            nonlocal image_at,last_stamp
            counts[topic]=counts.get(topic,0)+1
            if isinstance(msg,CompressedImage):
                stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
                if not camera_every or time.monotonic()-image_at<camera_every or stamp==last_stamp:return
                image_at=time.monotonic();last_stamp=stamp
                path=images/(f'{stamp[0]}_{stamp[1]:09}.jpg')
                with path.open('xb') as handle:handle.write(bytes(msg.data))
                data=dict(image=path.name,header=message_to_ordereddict(msg.header))
            else:data=message_to_ordereddict(msg)
            row=finite_tree(dict(topic=topic,received_monotonic=time.monotonic(),data=data))
            log.write(json.dumps(row,allow_nan=False)+'\n')
        return callback
    specs=[('/odom',Odometry),('/imu',Imu),('/scan',LaserScan),('/camera/camera_info',CameraInfo),
           ('/tf',TFMessage),('/camera/image_raw/compressed',CompressedImage)]
    subscriptions=[node.create_subscription(typ,topic,receive(topic),qos_profile_sensor_data) for topic,typ in specs]
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',receive('/tf_static'),QoSProfile(depth=20,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    start=time.monotonic()
    try:
        print('只读记录已开始；由操作者遥控或移动标定板，结束后另填独立参考值。',flush=True)
        while time.monotonic()-start<seconds:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:pass
    finally:
        log.close();node.destroy_node();rclpy.shutdown()
        (output/'capture.json').write_text(json.dumps(dict(seconds=time.monotonic()-start,counts=counts,passive=True),indent=2))
    if not counts:raise ValueError('未收到 ROS 数据；记录目录已保留，请检查 Domain 和驱动')
    return counts
