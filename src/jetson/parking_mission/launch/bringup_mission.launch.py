import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'parking_mission'
    
    # Find installed package assets, with a source-tree fallback.
    try:
        pkg_dir = get_package_share_directory(pkg_name)
    except Exception:
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    default_params_file = os.path.join(pkg_dir, 'config', 'mission_params.yaml')

    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'parking_mission.rviz')

    default_map_file = os.path.join(pkg_dir, 'maps', 'parking_map.yaml')
    if not os.path.exists(default_map_file):
        default_map_file = os.path.join(pkg_dir, 'parking_map.yaml')

    # Launch Arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Launch simulated vehicle node for RViz2 testing'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to mission_params.yaml'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz 2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz 2 configuration file'
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_file,
        description='Path to map YAML file'
    )

    use_sim = LaunchConfiguration('use_sim')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    map_yaml = LaunchConfiguration('map_yaml')

    # 1. Map Server Node
    map_server_node = Node(
        package=pkg_name,
        executable='map_server_node',
        name='map_server_node',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'frame_id': 'map'},
            {'topic_name': '/map'},
        ]
    )

    # 2. Mission Waypoint Follower Node
    mission_follower_node = Node(
        package=pkg_name,
        executable='mission_waypoint_follower',
        name='mission_waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    # 3. Kinematic Vehicle Simulator Node (Conditional)
    sim_vehicle_node = Node(
        package=pkg_name,
        executable='sim_vehicle_node',
        name='sim_vehicle_node',
        output='screen',
        parameters=[
            params_file,
            {'init_x': 1.80},
            {'init_y': 0.90},
            {'init_yaw': 3.141592},
        ],
        condition=IfCondition(use_sim)
    )

    # 4. RViz 2 Node (Conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_sim_arg,
        params_file_arg,
        rviz_arg,
        rviz_config_arg,
        map_yaml_arg,
        map_server_node,
        mission_follower_node,
        sim_vehicle_node,
        rviz_node,
    ])
