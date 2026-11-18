"""ROS side of the console protocol check, isolated from vehicle topics."""
import rclpy
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
rclpy.init()
node=rclpy.create_node('console_protocol_test')
pub=node.create_publisher(BatteryState,'/migration_check/battery_state',10)
battery=BatteryState(voltage=7.8,percentage=0.5,present=True)
def callback(message):
    if abs(message.linear.x-0.123)<1e-9 and abs(message.angular.z-0.321)<1e-9:
        print('TWIST_OK',flush=True)
sub=node.create_subscription(Twist,'/migration_check/web_twist',callback,10)
timer=node.create_timer(0.1,lambda:pub.publish(battery))
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.shutdown()
