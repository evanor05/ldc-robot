import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _optional_float(context, launch_config):
    value = context.perform_substitution(launch_config).strip()
    return float(value) if value else None


def _optional_bool(context, launch_config):
    value = context.perform_substitution(launch_config).strip().lower()
    if not value:
        return None
    return value in ("1", "true", "yes", "on")


def _optional_int(context, launch_config):
    value = context.perform_substitution(launch_config).strip()
    return int(value) if value else None


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    string_params = {
        "url": LaunchConfiguration("url"),
        "calibration_file": LaunchConfiguration("calibration_file"),
        "topic": LaunchConfiguration("topic"),
        "raw_topic": LaunchConfiguration("raw_topic"),
        "frame_id": LaunchConfiguration("frame_id"),
        "discovery_broadcast": LaunchConfiguration("discovery_broadcast"),
    }
    float_params = {
        "fps": LaunchConfiguration("fps"),
        "read_timeout": LaunchConfiguration("read_timeout"),
        "reconnect_delay": LaunchConfiguration("reconnect_delay"),
        "discovery_timeout": LaunchConfiguration("discovery_timeout"),
    }

    overrides = {}
    for name, launch_config in string_params.items():
        value = context.perform_substitution(launch_config).strip()
        if value:
            overrides[name] = value

    for name, launch_config in float_params.items():
        value = _optional_float(context, launch_config)
        if value is not None:
            overrides[name] = value

    jpeg_quality = _optional_int(context, LaunchConfiguration("jpeg_quality"))
    if jpeg_quality is not None:
        overrides["jpeg_quality"] = jpeg_quality

    use_ffmpeg = _optional_bool(context, LaunchConfiguration("use_ffmpeg"))
    if use_ffmpeg is not None:
        overrides["use_ffmpeg"] = use_ffmpeg

    publish_raw = _optional_bool(context, LaunchConfiguration("publish_raw"))
    if publish_raw is not None:
        overrides["publish_raw"] = publish_raw

    discovery = _optional_bool(context, LaunchConfiguration("discovery"))
    if discovery is not None:
        overrides["discovery"] = discovery

    discovery_port = _optional_int(context, LaunchConfiguration("discovery_port"))
    if discovery_port is not None:
        overrides["discovery_port"] = discovery_port

    parameters = [context.perform_substitution(params_file)]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="ldc_camera",
            executable="http_video_publisher",
            name="http_video_publisher",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("ldc_camera")
    default_params_file = os.path.join(
        package_dir,
        "config",
        "http_video_publisher.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to YAML file with camera parameters.",
        ),
        DeclareLaunchArgument(
            "url",
            default_value="",
            description="HTTP/MJPEG video stream URL.",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="",
            description="Compressed image topic to publish.",
        ),
        DeclareLaunchArgument(
            "raw_topic",
            default_value="",
            description="Raw image topic to publish for image_transport base topic.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="",
            description="Frame id for published images.",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="",
            description="Maximum publish rate, positive and at most 60.",
        ),
        DeclareLaunchArgument(
            "reconnect_delay",
            default_value="",
            description="Seconds to wait before reconnect attempts.",
        ),
        DeclareLaunchArgument(
            "use_ffmpeg",
            default_value="",
            description="Legacy compatibility argument; bounded MJPEG reader is always used.",
        ),
        DeclareLaunchArgument(
            "jpeg_quality",
            default_value="",
            description="Legacy compatibility argument; original JPEG bytes are preserved.",
        ),
        DeclareLaunchArgument(
            "publish_raw",
            default_value="",
            description="Also publish raw RGB images on raw_topic when true.",
        ),
        DeclareLaunchArgument(
            "discovery",
            default_value="",
            description="Discover the camera over UDP when url is empty.",
        ),
        DeclareLaunchArgument(
            "discovery_port",
            default_value="",
            description="UDP discovery port (default 33333).",
        ),
        DeclareLaunchArgument(
            "discovery_timeout",
            default_value="",
            description="UDP discovery timeout in seconds (default 2.0).",
        ),
        DeclareLaunchArgument(
            "discovery_broadcast",
            default_value="",
            description="UDP discovery broadcast address (default 255.255.255.255).",
        ),
        DeclareLaunchArgument("calibration_file", default_value="", description="Actual calibration YAML; empty means uncalibrated."),
        DeclareLaunchArgument("read_timeout", default_value="", description="MJPEG socket read timeout in seconds."),
        OpaqueFunction(function=_launch_setup),
    ])
