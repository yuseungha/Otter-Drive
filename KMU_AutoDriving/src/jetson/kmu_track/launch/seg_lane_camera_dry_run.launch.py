"""Run the live camera segmentation planner without hardware actuation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_MODEL = EnvironmentVariable(
    'KMU_SEG_MODEL_PATH',
    default_value='/home/sandi/KMU_AutoDriving/models/center_lane_best.pt',
)


def generate_launch_description() -> LaunchDescription:
    """Create a camera -> model -> path -> preview steering pipeline."""
    default_camera = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'camera.yaml'])
    default_segmentation = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'segmentation_lane.yaml'])
    default_control = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_video = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'video.yaml'])
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
        Node(
            package='kmu_track',
            executable='usb_camera_source',
            name='usb_camera_source',
            output='screen',
            parameters=[
                LaunchConfiguration('camera_config'),
                {'device': LaunchConfiguration('camera_device')},
            ],
            additional_env={'PYTHONNOUSERSITE': '1'},
        ),
        Node(
            package='kmu_track',
            executable='yolo_seg_lane_detector',
            name='yolo_seg_lane_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('segmentation_config'),
                {'model_path': LaunchConfiguration('model_path')},
            ],
        ),
        Node(
            package='kmu_track',
            executable='lane_control',
            name='lane_control',
            output='screen',
            parameters=[
                LaunchConfiguration('control_config'),
                {
                    'enabled': True,
                    'dry_run': True,
                    'hardware_confirmed': False,
                    'steering_only': True,
                    'ignore_mission_state': True,
                },
            ],
        ),
        Node(
            package='kmu_track',
            executable='actuation_monitor',
            name='actuation_monitor',
            output='screen',
            parameters=[{'dry_run': True}],
        ),
        Node(
            package='kmu_track',
            executable='track_visualizer',
            name='track_visualizer',
            output='screen',
            parameters=[
                LaunchConfiguration('video_config'),
                {'dry_run': True},
            ],
            additional_env={'PYTHONNOUSERSITE': '1'},
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane preview window closed'),
        ),
    ])
