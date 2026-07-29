from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('send_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('reset_guard_sec', default_value='3.2'),
        Node(
            package='rc_car_teleop',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': ParameterValue(
                    LaunchConfiguration('baud_rate'), value_type=int),
                'send_rate_hz': ParameterValue(
                    LaunchConfiguration('send_rate_hz'), value_type=float),
                'reset_guard_sec': ParameterValue(
                    LaunchConfiguration('reset_guard_sec'), value_type=float),
            }],
        ),
    ])
