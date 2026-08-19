"""USB camera, traffic-light perception, and guarded motor output."""

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
    live_hardware = IfCondition(PythonExpression([
        "'", motor_dry_run, "' == 'false' and '",
        hardware_confirmed, "' == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value='/dev/video0',
            description='Logitech V4L2 video device.'),
        DeclareLaunchArgument(
            'motor_dry_run',
            default_value='true',
            description='Preview drive commands without Arduino output.'),
        DeclareLaunchArgument(
            'hardware_confirmed',
            default_value='false',
            description='Required true before live motor output.'),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino serial device used only in live mode.'),
        Node(
            package='kmu_track',
            executable='usb_camera_source',
            name='usb_camera_source',
            output='screen',
            parameters=[{'device': camera_device}],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='traffic_light_detector',
            name='traffic_light_detector',
            output='screen',
            parameters=[perception_config],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='traffic_motor_controller',
            name='traffic_motor_controller',
            output='screen',
            parameters=[motor_config, {
                'enabled': True,
                'dry_run': motor_dry_run,
                'hardware_confirmed': hardware_confirmed,
            }],
        ),
        Node(
            package='rc_car_teleop',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'serial_port': serial_port}],
            condition=live_hardware,
        ),
    ])
