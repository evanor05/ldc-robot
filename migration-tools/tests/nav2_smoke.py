"""Activate real Nav2 with synthetic sensors and plan a path; no vehicle transport."""
import os
os.environ['ROS_DOMAIN_ID'] = '197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import json
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from tf2_ros import TransformBroadcaster

rclpy.init()
node = rclpy.create_node('nav2_migration_test')
broadcaster = TransformBroadcaster(node)
odom_pub = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
scan_pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
position = [0.0, 0.0]
maps = []
map_sub = node.create_subscription(OccupancyGrid, '/map', maps.append,
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
def sensors():
    stamp = node.get_clock().now().to_msg()
    tf = TransformStamped()
    tf.header.stamp = stamp
    tf.header.frame_id = 'map'
    tf.child_frame_id = 'odom'
    tf.transform.rotation.w = 1.0
    # AMCL has no initial pose in this test, so it does not own this test transform.
    broadcaster.sendTransform(tf)
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x, odom.pose.pose.position.y = position
    odom.pose.pose.position.z = 0.06205
    odom.pose.pose.orientation.w = 1.0
    odom_pub.publish(odom)
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -3.1415927
    scan.angle_max = 3.1415927
    scan.angle_increment = 6.2831854 / 359
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.02
    scan.range_max = 12.0
    scan.ranges = [float('inf')] * 360
    scan_pub.publish(scan)
timer = node.create_timer(0.1, sensors)
logs = Path(tempfile.mkdtemp(prefix='ros-car-nav2-'))
processes = []
files = []
names = ['map_server','amcl','controller_server','planner_server','smoother_server',
         'behavior_server','bt_navigator','waypoint_follower','velocity_smoother']
clients = {name:node.create_client(GetState, f'/{name}/get_state') for name in names}
def wait_future(future, seconds=10):
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), 'Service/action timeout'
    return future.result()
try:
    for package, launch, filename in [('ldc_bringup','ldc_bringup.launch.py','model.log'),
                                     ('ldc_navigation','navigation2.launch.py','navigation.log')]:
        log = open(logs / filename, 'w'); files.append(log)
        processes.append(subprocess.Popen(['ros2','launch',package,launch],
                         stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    deadline = time.monotonic() + 50
    states = {}
    pending = {}
    pose_selected = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        assert all(p.poll() is None for p in processes), f'Launch exited; see {logs}'
        if maps and not pose_selected:
            # Find a free 1 m square in the real saved map, avoiding arbitrary robot placement.
            m = maps[-1]; w=m.info.width; h=m.info.height
            candidates=[]
            for y in range(10,h-10,3):
                for x in range(10,w-10,3):
                    if all(m.data[(y+dy)*w+x+dx] == 0 for dy in range(-10,11,2) for dx in range(-10,11,2)):
                        candidates.append((abs(x-w/2)+abs(y-h/2),x,y))
            assert candidates, 'No clear test pose in saved map'
            _,x,y=min(candidates)
            position[:]=[m.info.origin.position.x+(x+0.5)*m.info.resolution,
                         m.info.origin.position.y+(y+0.5)*m.info.resolution]
            pose_selected=True
        for name, client in clients.items():
            if name in pending and pending[name].done():
                states[name]=pending.pop(name).result().current_state.id
            if name not in pending and client.service_is_ready():
                pending[name]=client.call_async(GetState.Request())
        if len(states)==len(names) and all(state==3 for state in states.values()) and pose_selected:
            break
    assert len(states)==len(names) and all(s==3 for s in states.values()), f'Not all active: {states}; logs={logs}'
    action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    assert action.wait_for_server(timeout_sec=5)
    goal = ComputePathToPose.Goal()
    goal.use_start=True
    goal.start.header.frame_id=goal.goal.header.frame_id='map'
    goal.start.pose.position.x,goal.start.pose.position.y=position
    goal.goal.pose.position.x=position[0]+0.15
    goal.goal.pose.position.y=position[1]
    goal.start.pose.orientation.w=goal.goal.pose.orientation.w=1.0
    goal.planner_id='GridBased'
    handle=wait_future(action.send_goal_async(goal))
    assert handle.accepted, 'Planner rejected goal'
    result=wait_future(handle.get_result_async())
    assert result.status==4 and len(result.result.path.poses)>1, result
    publishers=node.get_publishers_info_by_topic('/cmd_vel')
    assert publishers and all(p.topic_type=='geometry_msgs/msg/Twist' for p in publishers), publishers
    odom_subscribers = node.get_subscriptions_info_by_topic('/odom')
    for name in ('controller_server', 'bt_navigator', 'velocity_smoother'):
        endpoints = [endpoint for endpoint in odom_subscribers if endpoint.node_name == name]
        assert endpoints and all(endpoint.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for endpoint in endpoints), name
    print(json.dumps({'result':'PASS','active_nodes':states,'planned_poses':len(result.result.path.poses),
        'cmd_vel_type':'geometry_msgs/msg/Twist','synthetic_sensors':True,'vehicle_connected':False,'logs':str(logs)}))
finally:
    for p in processes:
        if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
    for p in processes:
        try: p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL); p.wait()
    for log in files: log.close()
    node.destroy_node()
    rclpy.shutdown()
