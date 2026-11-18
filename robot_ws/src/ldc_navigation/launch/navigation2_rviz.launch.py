"""Saved-map navigation with the vehicle's Lyrical launch and RViz config."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('ldc_navigation'))
    sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=str(share / 'maps/room.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'param/ldc_nav2.yaml')),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/navigation2.launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': sim_time,
                'params_file': LaunchConfiguration('params_file'),
            }.items()),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', str(share / 'rviz/ldc_navigation.rviz')],
            parameters=[{'use_sim_time': ParameterValue(sim_time, value_type=bool)}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
