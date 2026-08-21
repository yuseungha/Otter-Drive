"""Replay the supplied track video and show an annotated desktop window."""

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
    mission_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'mission.yaml'])
    perception_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    video_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'video.yaml'])

    video_path = LaunchConfiguration('video_path')
    loop = LaunchConfiguration('loop')
    playback_rate = LaunchConfiguration('playback_rate')
    model_path = LaunchConfiguration('model_path')
    display = LaunchConfiguration('display')
    system_python = {'PYTHONNOUSERSITE': '1'}

    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            default_value=DEFAULT_VIDEO,
            description='Absolute WSL path to the MP4 input.'),
        DeclareLaunchArgument(
            'loop', default_value='true',
            description='Restart from frame zero at end of video.'),
        DeclareLaunchArgument(
            'playback_rate', default_value='1.0',
            description='Video playback speed multiplier.'),
        DeclareLaunchArgument(
            'model_path', default_value=DEFAULT_MODEL,
            description='Absolute WSL path to road_best.pt.'),
        DeclareLaunchArgument(
            'display', default_value='true',
            description='Open the pipeline and HSV control windows.'),
        Node(
            package='kmu_track',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[mission_config],
        ),
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
                },
            ],
            additional_env=system_python,
        ),
        Node(
            package='kmu_track',
            executable='traffic_light_detector',
            name='traffic_light_detector',
            output='screen',
            parameters=[perception_config],
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
            executable='track_visualizer',
            name='track_visualizer',
            output='screen',
            parameters=[video_config],
            additional_env=system_python,
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Track visualizer closed'),
        ),
    ])
