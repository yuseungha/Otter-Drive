from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("listen_port", default_value="8765"),
        Node(
            package="laptop_teleop",
            executable="web_teleop",
            name="web_teleop",
            output="screen",
            parameters=[{
                "listen_host": "0.0.0.0",
                "listen_port": ParameterValue(
                    LaunchConfiguration("listen_port"), value_type=int),
                "publish_rate_hz": 20.0,
                "browser_timeout_sec": 0.25,
                "dry_run": True,
            }],
        ),
    ])

