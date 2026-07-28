from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('yolo_topic', default_value='/yolo/detections'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('publish_composite', default_value='true'),
        Node(
            package='line_detection',
            executable='debug_node',
            name='debug_visualization_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'yolo_topic': LaunchConfiguration('yolo_topic'),
                'show_window': LaunchConfiguration('show_window'),
                'publish_composite': LaunchConfiguration('publish_composite'),
            }],
        ),
    ])
