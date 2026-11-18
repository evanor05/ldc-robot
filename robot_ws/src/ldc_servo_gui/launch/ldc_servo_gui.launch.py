from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ldc_servo_gui",
                executable="ldc_servo_gui",
                name="ldc_servo_gui",
                output="screen",
                parameters=[{"servo_topic": "/servo_cmd"}],
            )
        ]
    )
