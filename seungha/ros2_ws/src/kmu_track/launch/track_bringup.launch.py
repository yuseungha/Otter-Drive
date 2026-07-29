"""Single-command bringup for track development."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    mission_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'mission.yaml'])
    perception_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    video_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    traffic_motor_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'traffic_motor.yaml'])
    default_model = PathJoinSubstitution([
        FindPackageShare('kmu_track'), '..', '..', '..', '..',
        'models', 'road_best.pt',
    ])
    demo = LaunchConfiguration('demo')
    display = LaunchConfiguration('display')
    model_path = LaunchConfiguration('model_path')
    motor_output = LaunchConfiguration('motor_output')
    motor_dry_run = LaunchConfiguration('motor_dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'demo', default_value='false',
            description='Play a no-hardware nominal three-lap scenario.'),
        DeclareLaunchArgument(
            'display', default_value='false',
            description='Open the pipeline and HSV control windows.'),
        DeclareLaunchArgument(
            'model_path', default_value=default_model,
            description='Absolute path to road_best.pt.'),
        DeclareLaunchArgument(
            'motor_output', default_value='false',
            description='Launch traffic-light motor command controller.'),
        DeclareLaunchArgument(
            'motor_dry_run', default_value='true',
            description='Publish preview instead of live drive commands.'),
        DeclareLaunchArgument(
            'hardware_confirmed', default_value='false',
            description='Required true before live motor output.'),
        Node(
            package='kmu_track',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[mission_config],
        ),
        Node(
            package='kmu_track',
            executable='traffic_light_detector',
            name='traffic_light_detector',
            output='screen',
            parameters=[perception_config],
            condition=UnlessCondition(demo),
            # Humble cv_bridge is built against Ubuntu's NumPy 1.x. Ignore a
            # user-site NumPy 2.x installation for this ROS image node.
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='yolo_lane_detector',
            name='yolo_lane_detector',
            output='screen',
            parameters=[perception_config, {'model_path': model_path}],
            condition=UnlessCondition(demo),
        ),
        Node(
            package='kmu_track',
            executable='track_visualizer',
            name='track_visualizer',
            output='screen',
            parameters=[video_config],
            condition=IfCondition(display),
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='scenario_player',
            name='scenario_player',
            output='screen',
            parameters=[mission_config],
            condition=IfCondition(demo),
        ),
        Node(
            package='kmu_track',
            executable='traffic_motor_controller',
            name='traffic_motor_controller',
            output='screen',
            parameters=[traffic_motor_config, {
                'enabled': True,
                'dry_run': motor_dry_run,
                'hardware_confirmed': hardware_confirmed,
            }],
            condition=IfCondition(motor_output),
        ),
    ])
