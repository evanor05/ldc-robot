"""只启动路径规划链：map_server + planner_server（内含 global_costmap）。

用途：不需要实车、不需要 AMCL、不需要控制器，就能验证
  地图是否被正确解读、A* 是否按预期绕障、是否拒绝穿越未知区域。

用法：
  ros2 launch ldc_navigation planning_only.launch.py \
      map:=/abs/path/arena.yaml

离线自测时用一个恒等静态 TF（map -> base_link）让代价地图能初始化；
上车时不要用这个 launch，改用完整导航 launch。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    share = get_package_share_directory('ldc_navigation')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    publish_fake_tf = LaunchConfiguration('publish_fake_tf')

    nodes = [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
    ]

    actions = [
        DeclareLaunchArgument(
            'map',
            description='必填：经过审计的地图 YAML 绝对路径',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'param', 'nav2_planning.yaml'),
            description='规划参数文件',
        ),
        DeclareLaunchArgument(
            'publish_fake_tf',
            default_value='true',
            description='离线自测用恒等 map->base_link 静态 TF；上车请设 false',
        ),
    ]

    for package, executable, name in nodes:
        extra = {'yaml_filename': ParameterValue(map_file, value_type=str)} \
            if name == 'map_server' else {}
        actions.append(Node(
            package=package,
            executable=executable,
            name=name,
            output='screen',
            parameters=[ParameterFile(params_file, allow_substs=True), extra],
        ))

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planning',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [name for _, _, name in nodes],
        }],
    ))

    # 仅离线自测：让 global_costmap 能拿到 robot pose。
    actions.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='planning_offline_tf',
        # Lyrical 的 static_transform_publisher 已不接受旧的 8 位置参数形式，
        # 必须用 --frame-id / --child-frame-id 显式指定，否则报 "Frame id must not be empty"。
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'map', '--child-frame-id', 'base_link',
        ],
        condition=IfCondition(publish_fake_tf),
    ))

    return LaunchDescription(actions)
