"""Safe-by-default live camera to steering actuation launch."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Model path comes from the environment (see .env.example).
DEFAULT_MODEL = EnvironmentVariable('KMU_MODEL_PATH', default_value='')


def generate_launch_description():
    default_camera = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'camera.yaml'])
    default_perception = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    default_control = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_video = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    camera_config = LaunchConfiguration('camera_config')
    perception = LaunchConfiguration('perception_config')
    control = LaunchConfiguration('control_config')
    video = LaunchConfiguration('video_config')
    enabled = LaunchConfiguration('enabled')
    dry_run = LaunchConfiguration('dry_run')
    confirmed = LaunchConfiguration('hardware_confirmed')
    steering_only = LaunchConfiguration('steering_only')
    serial_port = LaunchConfiguration('serial_port')
    camera_device = LaunchConfiguration('camera_device')
    display = LaunchConfiguration('display')
    model_path = LaunchConfiguration('model_path')
    serial_bridge = LaunchConfiguration('serial_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('steering_only', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL),
        DeclareLaunchArgument('serial_bridge', default_value='false'),
        DeclareLaunchArgument('camera_config', default_value=default_camera),
        DeclareLaunchArgument(
            'perception_config', default_value=default_perception),
        DeclareLaunchArgument('control_config', default_value=default_control),
        DeclareLaunchArgument('video_config', default_value=default_video),
        Node(
            package='kmu_track', executable='usb_camera_source',
            name='usb_camera_source', output='screen',
            parameters=[camera_config, {'device': camera_device}]),
        Node(
            package='kmu_track', executable='yolo_lane_detector',
            name='yolo_lane_detector', output='screen',
            parameters=[perception, {'model_path': model_path}]),
        Node(
            package='kmu_track', executable='lane_control',
            name='lane_control', output='screen',
            parameters=[control, {
                'enabled': ParameterValue(enabled, value_type=bool),
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'hardware_confirmed': ParameterValue(
                    confirmed, value_type=bool),
                'steering_only': ParameterValue(
                    steering_only, value_type=bool),
                'ignore_mission_state': True,
            }]),
        Node(
            package='rc_car_teleop', executable='serial_bridge',
            name='serial_bridge', output='screen',
            parameters=[{'serial_port': serial_port}],
            condition=IfCondition(serial_bridge)),
        Node(
            package='kmu_track', executable='actuation_monitor',
            name='actuation_monitor', output='screen',
            parameters=[{'dry_run': ParameterValue(dry_run, value_type=bool)}]),
        Node(
            package='kmu_track', executable='track_visualizer',
            name='track_visualizer', output='screen',
            parameters=[video, {
                'dry_run': ParameterValue(dry_run, value_type=bool)}],
            additional_env={'PYTHONNOUSERSITE': '1'},
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane verification window closed')),
    ])
