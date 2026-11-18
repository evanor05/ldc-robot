"""Prepare GMapping + Nav2 + passive frontier diagnostics, without final output.

Why not Include slam_nav2.launch.py directly?
  slam_nav2.launch.py does not expose cmd_vel_output (it was written before the
  bridge safety gate).  Reusing it would make the output topic implicit.  This
  launch includes the same two building blocks exactly once:
    - slam_gmapping.launch.py: the only /map and map->odom source
    - navigation2.launch.py with slam:=true: filters out map_server and amcl
  No map_server/AMCL/lifecycle duplication is introduced.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    exploration_share = Path(get_package_share_directory('ldc_exploration'))
    navigation_share = Path(get_package_share_directory('ldc_navigation'))
    gmapping_share = Path(get_package_share_directory('slam_gmapping'))
    config = str(exploration_share / 'config/exploration.yaml')
    params = str(navigation_share / 'param/ldc_nav2.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 只为兼容 start_mapping_task.sh 的 rviz:=false 传参；本模块不启动 RViz。
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('exploration_config', default_value=config),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gmapping_share / 'launch/slam_gmapping.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation_share / 'launch/navigation2.launch.py')),
            launch_arguments={
                'slam': 'true',
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('params_file'),
                'cmd_vel_output': 'cmd_vel_auto',
            }.items()),
        Node(
            package='ldc_exploration',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[LaunchConfiguration('exploration_config'),
                        {'use_sim_time': use_sim_time}]),
    ])
