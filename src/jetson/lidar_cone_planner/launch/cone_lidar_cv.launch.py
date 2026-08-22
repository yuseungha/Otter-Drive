"""A1 driver, measured static TF, fail-closed planner, and OpenCV viewer."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("lidar_cone_planner")
    planner_config = os.path.join(package_share, "config", "cone_planner.yaml")
    system_config = os.path.join(package_share, "config", "cone_lidar_cv.yaml")

    serial_port = LaunchConfiguration("serial_port")
    planning_frame = LaunchConfiguration("planning_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    scan_topic = LaunchConfiguration("scan_topic")
    viewer_enabled = LaunchConfiguration("viewer_enabled")
    viewer_record_path = LaunchConfiguration("viewer_record_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_"
                    "Controller_0001-if00-port0"
                ),
            ),
            DeclareLaunchArgument("serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("planner_config", default_value=planner_config),
            DeclareLaunchArgument("system_config", default_value=system_config),
            DeclareLaunchArgument("viewer_enabled", default_value="true"),
            DeclareLaunchArgument("viewer_record_path", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": serial_port,
                        "serial_baudrate": ParameterValue(
                            LaunchConfiguration("serial_baudrate"), value_type=int
                        ),
                        "frame_id": laser_frame,
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Sensitivity",
                    }
                ],
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
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
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
                            viewer_enabled, value_type=bool
                        ),
                        "viewer_record_path": viewer_record_path,
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]
    )
