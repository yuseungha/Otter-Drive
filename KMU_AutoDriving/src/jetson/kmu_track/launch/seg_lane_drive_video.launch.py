"""Replay video through the segmentation lane planner, safely preview or live."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_MODEL = EnvironmentVariable(
    'KMU_SEG_MODEL_PATH',
    default_value='/home/sandi/KMU_AutoDriving/models/center_lane_best.pt',
)


def generate_launch_description() -> LaunchDescription:
    """Create a model -> path -> dry-run steering video pipeline."""
    default_segmentation = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'segmentation_lane.yaml'])
    default_control = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_video = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    display = LaunchConfiguration('display')
    enabled = LaunchConfiguration('enabled')
    dry_run = LaunchConfiguration('dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    steering_only = LaunchConfiguration('steering_only')
    manage_serial_gate = LaunchConfiguration('manage_serial_gate')
    system_python = {'PYTHONNOUSERSITE': '1'}

    return LaunchDescription([
        DeclareLaunchArgument('video_path'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL),
        DeclareLaunchArgument('loop', default_value='false'),
        DeclareLaunchArgument('playback_rate', default_value='1.0'),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('steering_only', default_value='true'),
        DeclareLaunchArgument('manage_serial_gate', default_value='false'),
        DeclareLaunchArgument(
            'segmentation_config', default_value=default_segmentation),
        DeclareLaunchArgument('control_config', default_value=default_control),
        DeclareLaunchArgument('video_config', default_value=default_video),
        Node(
            package='kmu_track',
            executable='video_source',
            name='video_source',
            output='screen',
            parameters=[
                LaunchConfiguration('video_config'),
                {
                    'video_path': LaunchConfiguration('video_path'),
                    'loop': ParameterValue(
                        LaunchConfiguration('loop'), value_type=bool),
                    'playback_rate': ParameterValue(
                        LaunchConfiguration('playback_rate'),
                        value_type=float),
                    'wait_for_lane_ready': True,
                },
            ],
            additional_env=system_python,
            on_exit=Shutdown(reason='Video source stopped'),
        ),
        Node(
            package='kmu_track',
            executable='yolo_seg_lane_detector',
            name='yolo_seg_lane_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('segmentation_config'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),
        Node(
            package='kmu_track',
            executable='lane_control',
            name='lane_control',
            output='screen',
            parameters=[
                LaunchConfiguration('control_config'),
                {
                    'enabled': ParameterValue(enabled, value_type=bool),
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'hardware_confirmed': ParameterValue(
                        hardware_confirmed, value_type=bool),
                    'steering_only': ParameterValue(
                        steering_only, value_type=bool),
                    'manage_serial_gate': ParameterValue(
                        manage_serial_gate, value_type=bool),
                    'ignore_mission_state': True,
                },
            ],
        ),
        Node(
            package='kmu_track',
            executable='actuation_monitor',
            name='actuation_monitor',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(dry_run, value_type=bool),
            }],
        ),
        Node(
            package='kmu_track',
            executable='track_visualizer',
            name='track_visualizer',
            output='screen',
            parameters=[
                LaunchConfiguration('video_config'),
                {
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                },
            ],
            additional_env=system_python,
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane preview window closed'),
        ),
    ])
