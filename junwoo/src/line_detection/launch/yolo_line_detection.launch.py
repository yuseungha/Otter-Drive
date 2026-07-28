from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path', default_value='/home/juwnoo/Downloads/road_best.pt'
        ),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('imgsz', default_value='640'),
        DeclareLaunchArgument('confidence', default_value='0.25'),
        Node(
            package='line_detection',
            executable='yolo_detect_node',
            name='yolo_detect_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'imgsz': LaunchConfiguration('imgsz'),
                'confidence': LaunchConfiguration('confidence'),
            }],
        ),
    ])
