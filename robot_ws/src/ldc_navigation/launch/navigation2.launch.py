# flake8: noqa  # legacy：待整体重构，暂免 style 检查
"""Vehicle Nav2 components without the nav2_bringup Gazebo dependency."""
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _launch_navigation(context):
    share = Path(get_package_share_directory('ldc_navigation'))
    sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    # 每轮运行命名空间（B 隔离）：空 = 现状根命名空间（standalone/legacy 不变）。
    # 非空时由控制台后端传入 /ldc/nav/b{bootId}/r{epoch}（与 velocityTopic 同源身份），
    # 全部节点、内部 Action/服务/话题、参数 YAML 根路径与私有 TF 总线接入同一轮。
    run_namespace = LaunchConfiguration('run_namespace').perform(context).rstrip('/')
    rewrites = {'use_sim_time': sim_time}
    if LaunchConfiguration('global_inflation').perform(context) == 'false':
        rewrites['global_costmap.global_costmap.ros__parameters.inflation_layer.enabled'] = 'false'
    configured = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=run_namespace or None,
        param_rewrites=rewrites, convert_types=True), allow_substs=True)
    nodes = [
        ('nav2_map_server', 'map_server', 'map_server'),
        ('nav2_amcl', 'amcl', 'amcl'),
        ('nav2_controller', 'controller_server', 'controller_server'),
        ('nav2_planner', 'planner_server', 'planner_server'),
        ('nav2_smoother', 'smoother_server', 'smoother_server'),
        ('nav2_behaviors', 'behavior_server', 'behavior_server'),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother'),
    ]
    # Live SLAM already supplies /map and map -> odom. Do not also run AMCL.
    if LaunchConfiguration('slam').perform(context).lower() == 'true':
        nodes = [entry for entry in nodes if entry[2] not in ('map_server', 'amcl')]
    console_managed = LaunchConfiguration('managed_by_console').perform(context).lower() == 'true'
    actions = []
    # 雷达数据规范化：固件把「无回波」下发成 0.0（实车实测 360 点里 83-91 个），
    # AMCL 会当成贴脸障碍、costmap 会在车身周围标假障碍。本节点把 0.0/NaN 转成 inf
    # 后发到 scan_clean，AMCL 与 costmap 消费 scan_clean（见 ldc_nav2.yaml）。
    # 注意：此节点退出**不**触发整轮 Shutdown——它故障时应降级排查而不是让导航整体停摆。
    if LaunchConfiguration('sanitize_scan').perform(context).lower() == 'true':
        sanitizer = ExecuteProcess(
            cmd=[sys.executable, str(share / 'scripts' / 'lidar_sanitizer.py'),
                 '--ros-args', '-p', 'input_topic:=/scan', '-p', 'output_topic:=/scan_clean'],
            output='screen')
        actions.append(sanitizer)
    if run_namespace:
        # 私有 TF 总线的公共→私有转发器（裁决 Q3：launch 拥有的独立进程，
        # 生命周期跟随本轮；Bridge 崩溃时随 Nav2 保留，正常 stop 由后端杀树清理）。
        # 先于 Nav2 节点启动；退出触发整轮 Shutdown（撤权由 Bridge TF 断供检查兜底）。
        forwarder = ExecuteProcess(
            cmd=[sys.executable, str(share / 'scripts' / 'nav_tf_forwarder.py'),
                 '--namespace', run_namespace],
            output='screen')
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=forwarder,
            on_exit=[EmitEvent(event=Shutdown(reason='nav_tf_forwarder exited'))])))
        actions.append(forwarder)
    for package, executable, name in nodes:
        extra = {'yaml_filename': LaunchConfiguration('map')} if name == 'map_server' else {}
        if console_managed:
            extra['bond_heartbeat_period'] = 0.0
        remaps = []
        if run_namespace:
            # 私有 TF 总线：本轮 AMCL 的 map→odom 只进 NS/tf；公共→私有由 TF 转发器
            # 按允许变换白名单转发（拒绝 map 父帧，旧 AMCL/GMapping 无法混入）。
            # 必须用绝对名 remap：lyrical tf2_ros 收发均硬编码 "/tf"/"/tf_static"
            # （transform_listener.hpp:237/245、transform_broadcaster.hpp:87、
            # rclpy DEFAULT_TF_TOPIC='/tf'；相对规则 'tf' 不匹配，ns-probe 运行图实测）。
            remaps += [('/tf', run_namespace + '/tf'), ('/tf_static', run_namespace + '/tf_static')]
            # 共享环境输入：相对名参数 remap 回公共话题（decision.md:7 允许共享）。
            if name == 'amcl':
                remaps += [('scan', '/scan')]
            if name in ('controller_server', 'velocity_smoother'):
                remaps += [('odom', '/odom')]
        if name in ('controller_server', 'behavior_server'):
            remaps += [('cmd_vel', 'cmd_vel_nav')]
        elif name == 'velocity_smoother':
            remaps += [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', LaunchConfiguration('cmd_vel_output'))]
        node = Node(
        package=package, executable=executable, name=name,
        namespace=run_namespace or '/', output='screen',
        parameters=[configured, extra], remappings=remaps)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[EmitEvent(event=Shutdown(reason=f'{name} exited'))])))
        actions.append(node)
    if console_managed:
        return actions
    managed = [name for _, _, name in nodes]
    actions.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=run_namespace or '/',
        output='screen', parameters=[{
            'use_sim_time': ParameterValue(sim_time, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'bond_timeout': ParameterValue(LaunchConfiguration('bond_timeout'), value_type=float),
            'node_names': managed,
        }]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('ldc_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        # 默认输出桥接输入 /cmd_vel_auto：没有桥接时无人转发，安全。
        # 不能用 cmd_vel_nav——velocity_smoother 自己订阅 cmd_vel_nav，会形成自环。
        # 控制台模式仍由后端显式覆盖为 cmd_vel_auto（navigationRoutes.js）。
        # 仅用于离线/台架直连测试时才显式传 cmd_vel_output:=cmd_vel。
        DeclareLaunchArgument('cmd_vel_output', default_value='cmd_vel_auto'),
        DeclareLaunchArgument('bond_timeout', default_value='4.0'),
        DeclareLaunchArgument('managed_by_console', default_value='false'),
        # 每轮运行命名空间：空 = 根命名空间（现状）。控制台启动时传
        # /ldc/nav/b{bootId}/r{epoch}，与 Bridge begin 回执的 runIdentity 同源。
        DeclareLaunchArgument('run_namespace', default_value=''),
        # 雷达清洗（0.0 无效束 → inf）。默认开启；排查时可 -p 传 false 直连固件话题，
        # 同时把 ldc_nav2.yaml 的 scan_topic / costmap topic 改回 /scan。
        DeclareLaunchArgument('sanitize_scan', default_value='true'),
        DeclareLaunchArgument('global_inflation', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('slam', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value=str(share / 'maps/room.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(share / 'param/ldc_nav2.yaml')),
        OpaqueFunction(function=_launch_navigation),
    ])
