"""Run the integrated IRE camera planner with guarded steering-only output."""

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
    default_value='/home/sandi/KMU_AutoDriving/models/center_lane_best.pt',
)


def generate_launch_description() -> LaunchDescription:
    """Create an IRE direct-camera, control, monitor, and preview pipeline."""
    default_camera = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'), 'config', 'ire_camera.yaml'])
    default_segmentation = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'),
        'config',
        'ire_segmentation_lane.yaml',
    ])
    default_control = PathJoinSubstitution([
        FindPackageShare('kmu_track'), 'config', 'lane_control.yaml'])
    default_view = PathJoinSubstitution([
        FindPackageShare('kmu_ire_track'), 'config', 'ire_follow_view.yaml'])
    display = LaunchConfiguration('display')
    enabled = LaunchConfiguration('enabled')
    dry_run = LaunchConfiguration('dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    steering_only = LaunchConfiguration('steering_only')
    manage_serial_gate = LaunchConfiguration('manage_serial_gate')
    system_python = {'PYTHONNOUSERSITE': '1'}

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('steering_only', default_value='true'),
        DeclareLaunchArgument('manage_serial_gate', default_value='false'),
        DeclareLaunchArgument('camera_config', default_value=default_camera),
        DeclareLaunchArgument(
            'segmentation_config', default_value=default_segmentation),
        DeclareLaunchArgument('control_config', default_value=default_control),
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
            additional_env=system_python,
            on_exit=Shutdown(reason='IRE camera detector stopped'),
        ),
        Node(
            package='kmu_track',
            executable='lane_control',
            name='lane_control',
            output='screen',
            parameters=[
                LaunchConfiguration('control_config'),
                {
                    'enabled': ParameterValue(enabled, value_type=bool),
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'hardware_confirmed': ParameterValue(
                        hardware_confirmed, value_type=bool),
                    'steering_only': ParameterValue(
                        steering_only, value_type=bool),
                    'manage_serial_gate': ParameterValue(
                        manage_serial_gate, value_type=bool),
                    'ignore_mission_state': True,
                },
            ],
            on_exit=Shutdown(reason='Lane controller stopped'),
        ),
        Node(
            package='kmu_track',
            executable='actuation_monitor',
            name='actuation_monitor',
            output='screen',
            parameters=[{
                'dry_run': ParameterValue(dry_run, value_type=bool),
            }],
        ),
        Node(
            package='kmu_ire_track',
            executable='ire_follow_view',
            name='ire_follow_view',
            output='screen',
            parameters=[LaunchConfiguration('view_config')],
            additional_env=system_python,
            condition=IfCondition(display),
            on_exit=Shutdown(reason='Lane preview window closed'),
        ),
    ])
