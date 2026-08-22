"""Run the deterministic cone world through the real planner and controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create an isolated, real-time synthetic closed loop."""

    package_share = get_package_share_directory("lidar_cone_planner")
    planner_config = os.path.join(
        package_share, "config", "cone_planner.yaml"
    )
    controller_config = os.path.join(
        package_share, "config", "sim_controller.yaml"
    )
    world_config = os.path.join(
        package_share, "config", "synthetic_world.yaml"
    )
    namespace = LaunchConfiguration("namespace")
    planning_frame = "sim_base_link"

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="sim"),
            DeclareLaunchArgument("scenario", default_value="straight"),
            DeclareLaunchArgument("drop_scan_after_s", default_value="-1.0"),
            DeclareLaunchArgument("drop_scan_duration_s", default_value="0.0"),
            DeclareLaunchArgument("random_seed", default_value="7"),
            DeclareLaunchArgument("planner_config", default_value=planner_config),
            DeclareLaunchArgument(
                "controller_config", default_value=controller_config
            ),
            DeclareLaunchArgument("world_config", default_value=world_config),
            Node(
                package="lidar_cone_planner",
                executable="synthetic_cone_world",
                name="synthetic_cone_world",
                namespace=namespace,
                output="screen",
                parameters=[
                    LaunchConfiguration("world_config"),
                    {
                        "scenario": LaunchConfiguration("scenario"),
                        "drop_scan_after_s": ParameterValue(
                            LaunchConfiguration("drop_scan_after_s"),
                            value_type=float,
                        ),
                        "drop_scan_duration_s": ParameterValue(
                            LaunchConfiguration("drop_scan_duration_s"),
                            value_type=float,
                        ),
                        "random_seed": ParameterValue(
                            LaunchConfiguration("random_seed"),
                            value_type=int,
                        ),
                        "use_sim_time": False,
                    },
                ],
            ),
            Node(
                package="lidar_cone_planner",
                executable="cone_line_planner",
                name="cone_line_planner",
                namespace=namespace,
                output="screen",
                parameters=[
                    LaunchConfiguration("planner_config"),
                    {
                        "scan_topic": "scan",
                        "planning_frame": planning_frame,
                        "range_max_m": 1.65,
                        "front_angle_min_deg": -100.0,
                        "front_angle_max_deg": 100.0,
                        "planning_max_abs_lateral_m": 1.2,
                        "track_width_m": 0.64,
                        # Keep the synthetic corridor override internally
                        # consistent with a real-course planner YAML.
                        "track_width_min_m": 0.54,
                        "track_width_max_m": 0.74,
                        "cone_obstacle_radius_m": 0.015,
                        # The local controller needs only a short horizon.
                        # Bounding the visible candidate set keeps the Python
                        # multi-hypothesis search inside the 10 Hz deadline.
                        "planning_max_forward_m": 1.40,
                        "max_cone_candidates": 12,
                        "pair_beam_width": 4,
                        "max_pairs": 4,
                        "use_sim_time": False,
                    },
                ],
            ),
            Node(
                package="lidar_cone_planner",
                executable="cone_pure_pursuit",
                name="cone_pure_pursuit",
                namespace=namespace,
                output="screen",
                parameters=[
                    LaunchConfiguration("controller_config"),
                    {
                        "planning_frame": planning_frame,
                        "odom_child_frame": planning_frame,
                        "use_sim_time": False,
                    },
                ],
            ),
        ]
    )
