"""Jetson-side steering-only TCP receiver and Arduino serial bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the guarded Jetson receiver launch."""
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9100'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        Node(
            package='rc_car_teleop',
            executable='tcp_steering_receiver',
            output='screen',
            parameters=[{
                'port': ParameterValue(
                    LaunchConfiguration('port'), value_type=int),
                'command_timeout_sec': 0.30,
                'max_abs_steering': 650,
            }],
            on_exit=Shutdown(
                reason='TCP steering receiver exited; stopping bridge'),
        ),
        Node(
            package='rc_car_teleop',
            executable='serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'command_timeout_sec': 0.30,
            }],
            on_exit=Shutdown(
                reason='Serial bridge exited; stopping TCP receiver'),
        ),
    ])
