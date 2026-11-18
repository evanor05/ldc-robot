"""Real Lyrical ROS <-> Agent <-> repository XRCE 2.4.2 client, on loopback."""
import os
os.environ['ROS_DOMAIN_ID']='197'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import json
from pathlib import Path
import subprocess
import tempfile
import time
import rclpy
from ament_index_python.packages import get_package_prefix
from std_msgs.msg import String
from geometry_msgs.msg import Twist

root=Path(__file__).resolve().parents[2]
logs=Path(tempfile.mkdtemp(prefix='ros-car-xrce-'))
prefix=root/'agent_ws/.deps/legacy-install'
client=logs/'legacy-client'
subprocess.run(['cc',str(Path(__file__).with_name('xrce_ros_client.c')),
    '-I'+str(prefix/'include'),'-L'+str(prefix/'lib'),
    '-I'+str(prefix/'microcdr-2.0.1/include'),'-L'+str(prefix/'microcdr-2.0.1/lib'),
    '-lmicroxrcedds_client','-lmicrocdr','-lpthread','-lm','-lrt','-o',str(client)],check=True)
rclpy.init()
node=rclpy.create_node('xrce_migration_test')
received=[]
sub=node.create_subscription(String,'/migration_check/xrce_rx',lambda m:received.append(m.data),10)
pub=node.create_publisher(Twist,'/migration_check/xrce_tx',10)
message=Twist();message.linear.x=0.123;message.angular.z=0.321
timer=node.create_timer(0.1,lambda:pub.publish(message))
agent_log=open(logs/'agent.log','w')
client_log=open(logs/'client.log','w')
agent=subprocess.Popen([str(Path(get_package_prefix('micro_ros_agent'))/'lib/micro_ros_agent/micro_ros_agent'),
    'udp4','-p','18888','-v','4'],stdout=agent_log,stderr=subprocess.STDOUT)
process=None
try:
    time.sleep(1)
    assert agent.poll() is None, f'Agent failed: {logs}'
    process=subprocess.Popen([str(client)],stdout=client_log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+18
    while process.poll() is None and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=0.1)
    assert process.poll()==0, f'Legacy client failed ({process.poll()}); logs={logs}'
    assert 'legacy-client-to-lyrical-ok' in received, f'No ROS message from client; logs={logs}'
    print(json.dumps({'result':'PASS','checks':['XRCE 2.4.2 session','XRCE String to real ROS subscriber','real ROS Twist to XRCE reader','session deletion'],
        'limitation':'amd64 loopback; ESP32, Wi-Fi and custom services remain unverified','logs':str(logs)}))
finally:
    for p in (process,agent):
        if p is None: continue
        if p.poll() is None: p.terminate()
        try:p.wait(timeout=4)
        except subprocess.TimeoutExpired:p.kill();p.wait()
    agent_log.close();client_log.close()
    node.destroy_node();rclpy.shutdown()
