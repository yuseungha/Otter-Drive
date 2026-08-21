"""Replay track video with YOLO geometry, dry-run control, and one HUD."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Paths come from the environment (see .env.example).
# Laptop-only launch file: not used on the Jetson.
DEFAULT_VIDEO = EnvironmentVariable('KMU_VIDEO_PATH', default_value='')
DEFAULT_MODEL = EnvironmentVariable('KMU_MODEL_PATH', default_value='')


def generate_launch_description() -> LaunchDescription:
    """Create the safe video lane-drive launch description."""
    default_perception_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    default_control_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_video_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    perception_config = LaunchConfiguration('perception_config')
    control_config = LaunchConfiguration('control_config')
    video_config = LaunchConfiguration('video_config')
    video_path = LaunchConfiguration('video_path')
    model_path = LaunchConfiguration('model_path')
    loop = LaunchConfiguration('loop')
    playback_rate = LaunchConfiguration('playback_rate')
    display = LaunchConfiguration('display')
    auto_capture = LaunchConfiguration('auto_capture')
    enabled = LaunchConfiguration('enabled')
    dry_run = LaunchConfiguration('dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    steering_only = LaunchConfiguration('steering_only')
    serial_bridge = LaunchConfiguration('serial_bridge')
    serial_port = LaunchConfiguration('serial_port')
    system_python = {'PYTHONNOUSERSITE': '1'}

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            default_value=DEFAULT_VIDEO,
            description='Absolute WSL path to the MP4 input.'),
        DeclareLaunchArgument(
            'model_path',
            default_value=DEFAULT_MODEL,
            description='Absolute WSL path to road_best.pt.'),
        DeclareLaunchArgument(
            'loop', default_value='true',
            description='Restart from frame zero at end of video.'),
        DeclareLaunchArgument(
            'playback_rate', default_value='1.0',
            description='Video playback speed multiplier.'),
        DeclareLaunchArgument(
            'display', default_value='true',
            description='Open the four-panel verification window.'),
        DeclareLaunchArgument(
            'auto_capture', default_value='false',
            description='Save configured evidence frames under runs/.'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('steering_only', default_value='true'),
        DeclareLaunchArgument('serial_bridge', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument(
            'perception_config', default_value=default_perception_config),
        DeclareLaunchArgument(
            'control_config', default_value=default_control_config),
        DeclareLaunchArgument(
            'video_config', default_value=default_video_config),
        Node(
            package='kmu_track',
            executable='video_source',
            name='video_source',
            output='screen',
            parameters=[
                video_config,
                {
                    'video_path': video_path,
                    'loop': ParameterValue(loop, value_type=bool),
                    'playback_rate': ParameterValue(
                        playback_rate, value_type=float),
                    'wait_for_lane_ready': True,
                },
            ],
            additional_env=system_python,
        ),
        Node(
            package='kmu_track',
            executable='yolo_lane_detector',
            name='yolo_lane_detector',
            output='screen',
            parameters=[perception_config, {'model_path': model_path}],
        ),
        Node(
            package='kmu_track',
            executable='lane_control',
            name='lane_control',
            output='screen',
            parameters=[
                control_config,
                {
                    'enabled': ParameterValue(enabled, value_type=bool),
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'hardware_confirmed': ParameterValue(
                        hardware_confirmed, value_type=bool),
                    'steering_only': ParameterValue(
                        steering_only, value_type=bool),
                    'ignore_mission_state': True,
                },
            ],
        ),
        Node(
            package='rc_car_teleop',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'serial_port': serial_port}],
            condition=IfCondition(serial_bridge),
        ),
        Node(
            package='kmu_track',
            executable='actuation_monitor',
            name='actuation_monitor',
            output='screen',
            parameters=[{'dry_run': ParameterValue(dry_run, value_type=bool)}],
        ),
        Node(
            package='kmu_track',
            executable='track_visualizer',
            name='track_visualizer',
            output='screen',
            parameters=[
                video_config,
                {
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'auto_capture': ParameterValue(
                        auto_capture, value_type=bool),
                },
            ],
            additional_env=system_python,
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane verification window closed'),
        ),
    ])
