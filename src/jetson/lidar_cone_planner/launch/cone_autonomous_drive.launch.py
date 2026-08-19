"""LiDAR path planning, Pure Pursuit, and manual-protocol actuation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    planner_share = get_package_share_directory("lidar_cone_planner")
    teleop_share = get_package_share_directory("rc_car_teleop")
    planner_config = os.path.join(planner_share, "config", "cone_planner.yaml")
    system_config = os.path.join(planner_share, "config", "cone_lidar_cv.yaml")
    controller_config = os.path.join(
        planner_share, "config", "cone_controller.yaml"
    )
    adapter_config = os.path.join(
        teleop_share, "config", "autonomous_drive.yaml"
    )

    lidar_port = LaunchConfiguration("lidar_port")
    arduino_port = LaunchConfiguration("arduino_port")
    planning_frame = LaunchConfiguration("planning_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    dry_run = LaunchConfiguration("dry_run")
    hardware_confirmed = LaunchConfiguration("hardware_confirmed")
    serial_bridge = LaunchConfiguration("serial_bridge")
    use_sim_time = LaunchConfiguration("use_sim_time")
    live_serial = IfCondition(PythonExpression([
        "'", serial_bridge, "' == 'true' and '",
        dry_run, "' == 'false' and '",
        hardware_confirmed, "' == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "lidar_port",
            default_value=(
                "/dev/serial/by-id/"
                "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
                "Controller_0001-if00-port0"
            ),
        ),
        DeclareLaunchArgument(
            "arduino_port",
            default_value="/dev/serial/by-id/REPLACE_WITH_ARDUINO_DEVICE",
        ),
        DeclareLaunchArgument("planning_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
        DeclareLaunchArgument("scan_topic", default_value="scan"),
        DeclareLaunchArgument("planner_config", default_value=planner_config),
        DeclareLaunchArgument("system_config", default_value=system_config),
        DeclareLaunchArgument(
            "controller_config", default_value=controller_config
        ),
        DeclareLaunchArgument("adapter_config", default_value=adapter_config),
        DeclareLaunchArgument("viewer_enabled", default_value="true"),
        DeclareLaunchArgument("viewer_record_path", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("hardware_confirmed", default_value="false"),
        DeclareLaunchArgument("serial_bridge", default_value="false"),
        DeclareLaunchArgument("require_odometry", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("throttle_min", default_value="0"),
        DeclareLaunchArgument("throttle_max", default_value="300"),
        DeclareLaunchArgument("steering_min", default_value="-900"),
        DeclareLaunchArgument("steering_max", default_value="900"),

        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 115200,
                "frame_id": laser_frame,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Sensitivity",
            }],
            remappings=[("scan", scan_topic)],
        ),
        Node(
            package="lidar_cone_planner",
            executable="cone_lidar_static_tf",
            name="cone_lidar_static_tf",
            output="screen",
            parameters=[
                LaunchConfiguration("system_config"),
                {
                    "planning_frame": planning_frame,
                    "laser_frame": laser_frame,
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="lidar_cone_planner",
            executable="cone_line_planner",
            name="cone_line_planner",
            output="screen",
            parameters=[
                LaunchConfiguration("planner_config"),
                {
                    "planning_frame": planning_frame,
                    "scan_topic": scan_topic,
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="lidar_cone_planner",
            executable="cone_cv_viewer",
            name="cone_cv_viewer",
            output="screen",
            parameters=[
                LaunchConfiguration("system_config"),
                {
                    "planning_frame": planning_frame,
                    "scan_topic": scan_topic,
                    "viewer_enabled": ParameterValue(
                        LaunchConfiguration("viewer_enabled"), value_type=bool
                    ),
                    "viewer_record_path": LaunchConfiguration(
                        "viewer_record_path"
                    ),
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="lidar_cone_planner",
            executable="cone_pure_pursuit",
            name="cone_pure_pursuit",
            output="screen",
            parameters=[
                LaunchConfiguration("controller_config"),
                {
                    "planning_frame": planning_frame,
                    "allow_compat_command": True,
                    "require_odometry": ParameterValue(
                        LaunchConfiguration("require_odometry"), value_type=bool
                    ),
                    "use_sim_time": ParameterValue(
                        use_sim_time, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="rc_car_teleop",
            executable="ackermann_to_drive_cmd",
            name="ackermann_to_drive_cmd",
            output="screen",
            parameters=[
                LaunchConfiguration("adapter_config"),
                {
                    "dry_run": ParameterValue(dry_run, value_type=bool),
                    "hardware_confirmed": ParameterValue(
                        hardware_confirmed, value_type=bool
                    ),
                },
            ],
        ),
        Node(
            package="rc_car_teleop",
            executable="serial_bridge",
            name="serial_bridge",
            output="screen",
            parameters=[{
                "serial_port": arduino_port,
                "drive_enabled": True,
                "limits_confirmed": True,
                "throttle_min": ParameterValue(
                    LaunchConfiguration("throttle_min"), value_type=int
                ),
                "throttle_max": ParameterValue(
                    LaunchConfiguration("throttle_max"), value_type=int
                ),
                "steering_min": ParameterValue(
                    LaunchConfiguration("steering_min"), value_type=int
                ),
                "steering_max": ParameterValue(
                    LaunchConfiguration("steering_max"), value_type=int
                ),
            }],
            condition=live_serial,
        ),
    ])
