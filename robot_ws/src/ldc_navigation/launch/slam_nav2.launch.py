"""Live GMapping and Nav2, with a single map -> odom publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('ldc_navigation'))
    slam_share = Path(get_package_share_directory('slam_gmapping'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(share / 'param/ldc_nav2.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(slam_share / 'launch/slam_gmapping.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/navigation2.launch.py')),
            launch_arguments={
                'slam': 'true', 'use_sim_time': 'false',
                'params_file': LaunchConfiguration('params_file'),
            }.items()),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(share / 'rviz/ldc_navigation.rviz')],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
