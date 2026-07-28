from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_VIDEO_PATH = (
    '/home/juwnoo/Downloads/'
    '국민대학교 자율주행 스튜디오 트랙 영상 - 자이트론 (480p, h264).mp4'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value=DEFAULT_VIDEO_PATH),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('fps', default_value='0.0'),
        DeclareLaunchArgument('loop', default_value='true'),
        Node(
            package='camera_publisher',
            executable='video_publisher_node',
            name='video_publisher_node',
            output='screen',
            parameters=[{
                'video_path': LaunchConfiguration('video_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'fps': LaunchConfiguration('fps'),
                'loop': LaunchConfiguration('loop'),
            }],
        ),
    ])
