"""Manual driving guarded by camera traffic-light perception."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    perception_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'perception.yaml'])
    motor_config = PathJoinSubstitution(
        [FindPackageShare('kmu_track'), 'config', 'traffic_motor.yaml'])
    camera_device = LaunchConfiguration('camera_device')
    motor_dry_run = LaunchConfiguration('motor_dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    serial_port = LaunchConfiguration('serial_port')
    web_teleop_enabled = LaunchConfiguration('web_teleop')
    preview_server_enabled = LaunchConfiguration('preview_server')
    live_hardware = IfCondition(PythonExpression([
        "'", motor_dry_run, "' == 'false' and '",
        hardware_confirmed, "' == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('motor_dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('web_teleop', default_value='true'),
        DeclareLaunchArgument('preview_server', default_value='true'),
        Node(
            package='kmu_track',
            executable='usb_camera_source',
            output='screen',
            parameters=[{'device': camera_device}],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='traffic_light_detector',
            output='screen',
            parameters=[perception_config],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='laptop_teleop',
            executable='web_teleop',
            output='screen',
            parameters=[{
                'listen_host': '127.0.0.1',
                'listen_port': 8765,
                'dry_run': motor_dry_run,
            }],
            remappings=[
                ('/rc_car/drive_cmd', '/rc_car/manual_drive_cmd'),
            ],
            condition=IfCondition(web_teleop_enabled),
        ),
        Node(
            package='kmu_track',
            executable='traffic_motor_controller',
            output='screen',
            parameters=[motor_config, {
                'enabled': True,
                'dry_run': motor_dry_run,
                'hardware_confirmed': hardware_confirmed,
                'manual_mode': True,
            }],
        ),
        Node(
            package='kmu_track',
            executable='traffic_preview_server',
            output='screen',
            parameters=[{
                'bind_address': '127.0.0.1',
                'port': 8080,
            }],
            condition=IfCondition(preview_server_enabled),
        ),
        Node(
            package='rc_car_teleop',
            executable='serial_bridge',
            output='screen',
            parameters=[{'serial_port': serial_port}],
            condition=live_hardware,
        ),
    ])
