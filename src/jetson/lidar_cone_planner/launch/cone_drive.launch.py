"""Launch cone detection/planning and the disabled-by-default controller."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("lidar_cone_planner")
    planner_config = os.path.join(package_share, "config", "cone_planner.yaml")
    controller_config = os.path.join(
        package_share, "config", "cone_controller.yaml"
    )
    common = {
        "planning_frame": LaunchConfiguration("planning_frame"),
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }
    planner_overrides = {
        **common,
        "scan_topic": LaunchConfiguration("scan_topic"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_config", default_value=planner_config),
            DeclareLaunchArgument(
                "controller_config", default_value=controller_config
            ),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="lidar_cone_planner",
                executable="cone_line_planner",
                name="cone_line_planner",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("planner_config"),
                    planner_overrides,
                ],
            ),
            Node(
                package="lidar_cone_planner",
                executable="cone_pure_pursuit",
                name="cone_pure_pursuit",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[LaunchConfiguration("controller_config"), common],
            ),
        ]
    )
