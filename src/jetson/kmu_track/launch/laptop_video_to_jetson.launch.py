"""Run video perception on the laptop and send steering only to Jetson."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Create the laptop video-to-steering sender pipeline."""
    perception = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    control = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    video = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    display = LaunchConfiguration('display')
    system_python = {'PYTHONNOUSERSITE': '1'}
    return LaunchDescription([
        DeclareLaunchArgument('video_path'),
        DeclareLaunchArgument('model_path'),
        DeclareLaunchArgument('jetson_host', default_value='192.168.1.5'),
        DeclareLaunchArgument('port', default_value='9100'),
        DeclareLaunchArgument('display', default_value='true'),
        DeclareLaunchArgument('loop', default_value='true'),
        Node(
            package='kmu_track', executable='video_source', output='screen',
            parameters=[video, {
                'video_path': LaunchConfiguration('video_path'),
                'loop': ParameterValue(
                    LaunchConfiguration('loop'), value_type=bool),
                'wait_for_lane_ready': True,
            }], additional_env=system_python),
        Node(
            package='kmu_track', executable='yolo_lane_detector',
            output='screen', parameters=[perception, {
                'model_path': LaunchConfiguration('model_path'),
                'input_size': 320,
            }]),
        Node(
            package='kmu_track', executable='lane_control', output='screen',
            parameters=[control, {
                'enabled': True,
                'dry_run': True,
                'hardware_confirmed': False,
                'steering_only': True,
                'ignore_mission_state': True,
            }]),
        Node(
            package='kmu_track', executable='tcp_steering_sender',
            output='screen', parameters=[{
                'host': LaunchConfiguration('jetson_host'),
                'port': ParameterValue(
                    LaunchConfiguration('port'), value_type=int),
            }]),
        Node(
            package='kmu_track', executable='track_visualizer',
            output='screen', parameters=[video, {'dry_run': True}],
            additional_env=system_python, condition=IfCondition(display),
            on_exit=Shutdown(reason='Laptop lane window closed')),
    ])
