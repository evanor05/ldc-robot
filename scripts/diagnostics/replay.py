"""Replay sensor CDR in Domain 196; recorded commands go to observation topics."""
import json
import math
import os
from pathlib import Path
import time
from .storage import TYPES, COMMAND_TOPICS, open_archive, verify_archive


def replay(folder, rate=1.):
    if not math.isfinite(rate) or not .1 <= rate <= 4:
        raise ValueError('回放倍率必须为 0.1–4')
    verify_archive(folder)
    # Force isolation before importing/initializing ROS, ignoring caller domain.
    os.environ.update(ROS_DOMAIN_ID='196', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_LOCALHOST_ONLY='1', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    for key in ('ROS_STATIC_PEERS', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE',
                'CYCLONEDDS_URI', 'ROS_DISCOVERY_SERVER'):
        os.environ.pop(key, None)
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = rclpy.create_node('ldc_isolated_replay')
    clock = node.create_publisher(Clock, '/clock', 10)
    publishers = {}
    db = open_archive(folder)
    count = 0
    print('隔离回放：Domain 196，本机发现；运动与初始位置只发布到 /replay/recorded/*。', flush=True)
    try:
        for (topic,) in db.execute('SELECT DISTINCT topic FROM messages'):
            if topic not in TYPES:
                raise ValueError('归档包含未知话题')
            target = '/replay/recorded'+topic if topic in COMMAND_TOPICS else topic
            cls = get_message(TYPES[topic])
            qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL if topic in ('/tf_static', '/map') else DurabilityPolicy.VOLATILE)
            publishers[topic] = (node.create_publisher(cls, target, qos), cls)
        # Give discovery a bounded head start, including for one-shot static data.
        start = time.monotonic()+1.
        for topic, ros_ns, elapsed, cdr in db.execute('SELECT topic,ros_ns,elapsed,cdr FROM messages ORDER BY id'):
            if topic not in TYPES or len(cdr)>8*1024*1024:
                raise ValueError('归档包含未知话题或超大消息')
            if not math.isfinite(elapsed) or not 0 <= elapsed <= 601 or not isinstance(ros_ns, int) or ros_ns<0:
                raise ValueError('归档时间字段无效')
            deadline = start+elapsed/rate
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=max(0., min(.05, deadline-time.monotonic())))
            msg = Clock()
            msg.clock.sec, msg.clock.nanosec = divmod(ros_ns, 1_000_000_000)
            clock.publish(msg)
            publisher, cls = publishers[topic]
            publisher.publish(deserialize_message(cdr, cls))
            count += 1
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        db.close()
        node.destroy_node()
        rclpy.shutdown()
    return dict(messages=count, domain=196, commandsReplayedToRobot=False)


def summarize_capture(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from scripts.calibration.record import finite_tree
    from scripts.calibration.summarize import summarize_rows
    from .storage import atomic_json
    with open_archive(folder) as db:
        def rows():
            for topic, elapsed, cdr in db.execute("SELECT topic,elapsed,cdr FROM messages WHERE topic IN ('/odom','/scan','/imu') ORDER BY id"):
                data = message_to_ordereddict(deserialize_message(cdr, get_message(TYPES[topic])))
                yield finite_tree(dict(topic=topic, received_monotonic=elapsed, data=data))
        result = summarize_rows(rows())
    atomic_json(Path(folder)/'summary.json', result)
    return result
