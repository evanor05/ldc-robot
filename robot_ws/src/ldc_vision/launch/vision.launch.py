from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'url': '', 'gateway_port': '18320', 'device': '0', 'imgsz': '640', 'confidence': '0.25',
        # 三色交通灯权重（nc=5）；回滚通用权重：改为 yolo26n.pt 并把 color_mode 设 opencv
        'model_path': str(Path.home() / '.local/share/ldc/models/traffic26n.pt'),
        'color_mode': 'yolo',
        'worker_python': '/home/<user>/esp/yolo-bench-env/bin/python', 'class_filter': 'all',
        'calibration_file': '', 'read_timeout': '2.0',
        'params_file': str(Path(get_package_share_directory('ldc_vision')) / 'config/vision.yaml'),
    }
    camera = {'url': ParameterValue(LaunchConfiguration('url'), value_type=str),
              'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'), value_type=str),
              'read_timeout': ParameterValue(LaunchConfiguration('read_timeout'), value_type=float),
              'fps': 30.0, 'publish_raw': True, 'frame_id': 'camera_optical_frame'}
    detector = {name: ParameterValue(LaunchConfiguration(name), value_type=(int if name in ('gateway_port', 'imgsz') else float if name == 'confidence' else str))
                for name in defaults if name not in ('url', 'calibration_file', 'read_timeout', 'params_file')}
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='视觉节点退出，清理整组'))])),
        Node(package='ldc_camera', executable='http_video_publisher', name='http_video_publisher', output='screen', parameters=[camera]),
        Node(package='ldc_vision', executable='detector', name='ldc_detector', output='screen',
             parameters=[LaunchConfiguration('params_file'), detector]),
    ])
