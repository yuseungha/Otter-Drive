"""Launch only the perception-driven mode decision node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'mode_decision.yaml'])
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('planning_frame', default_value='base_link'),
        Node(
            package='kmu_track',
            executable='mode_decision',
            name='mode_decision',
            output='screen',
            parameters=[LaunchConfiguration('config'), {
                'planning_frame': LaunchConfiguration('planning_frame'),
            }],
        ),
    ])
