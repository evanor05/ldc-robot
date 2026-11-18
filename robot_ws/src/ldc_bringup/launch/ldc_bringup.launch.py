import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory('ldc_description')
    urdf_model_path = os.path.join(description_share, 'urdf', 'ldc.urdf')

    robot_description = {'robot_description': Command(['cat ', urdf_model_path])}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[robot_description],
        name='joint_state_publisher',
        output='screen',
    )

    ldc_bringup_node = Node(
        package='ldc_bringup',
        executable='ldc_bringup',
        name='ldc_bringup',
        output='screen',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        ldc_bringup_node,
    ])
