"""Launch the fail-closed Pure Pursuit controller by itself."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("lidar_cone_planner")
    parameter_file = os.path.join(package_share, "config", "cone_controller.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=parameter_file),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("planning_frame", default_value="base_link"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="lidar_cone_planner",
                executable="cone_pure_pursuit",
                name="cone_pure_pursuit",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "planning_frame": LaunchConfiguration("planning_frame"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
            ),
        ]
    )
