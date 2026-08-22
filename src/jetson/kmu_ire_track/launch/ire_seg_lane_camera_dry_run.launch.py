"""Run the IRE center-priority camera planner without actuation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


DEFAULT_MODEL = EnvironmentVariable(
    'KMU_SEG_MODEL_PATH',
    default_value='/home/sandi/KMU_AutoDriving/models/lane_seg_v3_e37.pt',
)


def generate_launch_description() -> LaunchDescription:
    """Create an IRE camera -> model -> path -> preview pipeline."""
    default_camera = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'), 'config', 'ire_camera.yaml'])
    default_segmentation = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'),
        'config',
        'ire_segmentation_lane.yaml',
    ])
    default_control = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_video = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'video.yaml'])
    default_view = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'), 'config', 'ire_follow_view.yaml'])
    display = LaunchConfiguration('display')

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('camera_config', default_value=default_camera),
        DeclareLaunchArgument(
            'segmentation_config', default_value=default_segmentation),
        DeclareLaunchArgument('control_config', default_value=default_control),
        DeclareLaunchArgument('video_config', default_value=default_video),
        DeclareLaunchArgument('view_config', default_value=default_view),
        Node(
            package='kmu_ire_track',
            executable='ire_yolo_seg_lane_detector',
            name='ire_yolo_seg_lane_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('camera_config'),
                LaunchConfiguration('segmentation_config'),
                {
                    'camera_device': LaunchConfiguration('camera_device'),
                    'model_path': LaunchConfiguration('model_path'),
                    'publish_lane_overlay': ParameterValue(
                        display, value_type=bool),
                },
            ],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_ire_track',
            executable='ire_follow_view',
            name='ire_follow_view',
            output='screen',
            parameters=[LaunchConfiguration('view_config')],
            additional_env={'PYTHONNOUSERSITE': '1'},
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane preview window closed'),
        ),
    ])
