from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_MEGA_PORT = (
    "/dev/serial/by-id/"
    "usb-Arduino__www.arduino.cc__0043_5583832383535181C1B0-if00"
)


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    listen_port = LaunchConfiguration("listen_port")

    web_node = Node(
        package="laptop_teleop",
        executable="web_teleop",
        name="web_teleop",
        output="screen",
        parameters=[{
            "listen_host": "0.0.0.0",
            "listen_port": ParameterValue(listen_port, value_type=int),
            "publish_rate_hz": 20.0,
            "browser_timeout_sec": 0.25,
            "dry_run": False,
        }],
    )

    serial_bridge = Node(
        package="rc_car_teleop",
        executable="serial_bridge",
        name="serial_bridge",
        output="screen",
        parameters=[{
            "serial_port": serial_port,
            "baud_rate": 115200,
            "send_rate_hz": 20.0,
            "command_timeout_sec": 0.30,
            "reset_guard_sec": 3.2,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value=DEFAULT_MEGA_PORT),
        DeclareLaunchArgument("listen_port", default_value="8765"),
        web_node,
        # Let the web node subscribe before the bridge announces connection.
        TimerAction(period=1.0, actions=[serial_bridge]),
    ])

