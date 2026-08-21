"""IRE lane control plus LiDAR obstacle/cone planning and one serial output."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _config(package: str, filename: str):
    return PathJoinSubstitution(
        [FindPackageShare(package), 'config', filename])


def generate_launch_description() -> LaunchDescription:
    camera_device = LaunchConfiguration('camera_device')
    model_path = LaunchConfiguration('model_path')
    lidar_port = LaunchConfiguration('lidar_port')
    arduino_port = LaunchConfiguration('arduino_port')
    planning_frame = LaunchConfiguration('planning_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    output_topic = LaunchConfiguration('output_topic')
    serial_bridge = LaunchConfiguration('serial_bridge')
    dry_run = LaunchConfiguration('dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    competition_no_stop = LaunchConfiguration('competition_no_stop')
    viewer = LaunchConfiguration('viewer')
    system_python = {'PYTHONNOUSERSITE': '1'}

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_device',
            default_value=EnvironmentVariable(
                'KMU_CAMERA_DEVICE', default_value='/dev/video0')),
        DeclareLaunchArgument(
            'model_path',
            default_value=EnvironmentVariable(
                'KMU_SEG_MODEL_PATH', default_value='')),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=EnvironmentVariable(
                'KMU_LIDAR_DEVICE', default_value='/dev/ttyUSB0')),
        DeclareLaunchArgument(
            'arduino_port',
            default_value=EnvironmentVariable(
                'KMU_SERIAL_DEVICE', default_value='/dev/ttyACM0')),
        DeclareLaunchArgument('planning_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument(
            'output_topic', default_value='/rc_car/drive_cmd_preview'),
        DeclareLaunchArgument('serial_bridge', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('competition_no_stop', default_value='false'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('throttle_max', default_value='700'),
        DeclareLaunchArgument('steering_min', default_value='-650'),
        DeclareLaunchArgument('steering_max', default_value='650'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=_config('kmu_ire_track', 'ire_camera.yaml')),
        DeclareLaunchArgument(
            'segmentation_config',
            default_value=_config(
                'kmu_ire_track', 'ire_segmentation_lane.yaml')),
        DeclareLaunchArgument(
            'control_config',
            default_value=_config('kmu_track', 'lane_control.yaml')),
        DeclareLaunchArgument(
            'autonomy_config',
            default_value=_config('kmu_track', 'unified_autonomy.yaml')),
        DeclareLaunchArgument(
            'cone_planner_config',
            default_value=_config(
                'lidar_cone_planner', 'cone_planner.yaml')),
        DeclareLaunchArgument(
            'lidar_system_config',
            default_value=_config(
                'lidar_cone_planner', 'cone_lidar_cv.yaml')),

        Node(
            package='kmu_ire_track',
            executable='ire_yolo_seg_lane_detector',
            name='ire_yolo_seg_lane_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('camera_config'),
                LaunchConfiguration('segmentation_config'),
                {
                    'camera_device': camera_device,
                    'model_path': model_path,
                    'planning_frame': planning_frame,
                    'publish_lane_overlay': ParameterValue(
                        viewer, value_type=bool),
                },
            ],
            additional_env=system_python,
            on_exit=Shutdown(
                reason='IRE lane detector stopped',
                condition=UnlessCondition(competition_no_stop),
            ),
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
                    'dry_run': ParameterValue(dry_run, value_type=bool),
                    'hardware_confirmed': ParameterValue(
                        hardware_confirmed, value_type=bool),
                    'steering_only': False,
                    'command_topic': '/rc_car/ire_lane_cmd',
                    'manage_serial_gate': False,
                    'ignore_mission_state': False,
                    'active_states': ['LANE'],
                },
            ],
            on_exit=Shutdown(
                reason='IRE lane controller stopped',
                condition=UnlessCondition(competition_no_stop),
            ),
        ),

        Node(
            package='rplidar_ros', executable='rplidar_node',
            name='rplidar_node', output='screen', parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': laser_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }], remappings=[('scan', scan_topic)]),
        Node(
            package='lidar_cone_planner',
            executable='cone_lidar_static_tf',
            name='cone_lidar_static_tf', output='screen',
            parameters=[LaunchConfiguration('lidar_system_config'), {
                'planning_frame': planning_frame,
                'laser_frame': laser_frame}]),
        Node(
            package='lidar_cone_planner', executable='cone_line_planner',
            name='cone_line_planner', output='screen',
            parameters=[LaunchConfiguration('cone_planner_config'), {
                'scan_topic': scan_topic,
                'planning_frame': planning_frame,
                'managed_subscription': False}]),
        Node(
            package='lidar_cone_planner', executable='cone_cv_viewer',
            name='cone_cv_viewer', output='screen',
            condition=IfCondition(viewer),
            parameters=[LaunchConfiguration('lidar_system_config'), {
                'planning_frame': planning_frame,
                'scan_topic': scan_topic,
                'viewer_enabled': True}]),

        Node(
            package='kmu_track', executable='unified_autonomy',
            name='unified_autonomy', output='screen',
            parameters=[LaunchConfiguration('autonomy_config'), {
                'planning_frame': planning_frame,
                'output_topic': output_topic,
                'auto_arm_drive': ParameterValue(
                    serial_bridge, value_type=bool),
                'competition_no_stop_enabled': ParameterValue(
                    competition_no_stop, value_type=bool)}]),
        Node(
            package='kmu_track', executable='actuation_monitor',
            name='actuation_monitor', output='screen',
            parameters=[{
                'dry_run': ParameterValue(dry_run, value_type=bool),
            }]),

        Node(
            package='rc_car_teleop', executable='serial_bridge',
            name='serial_bridge', output='screen',
            condition=IfCondition(serial_bridge),
            parameters=[{
                'serial_port': arduino_port,
                'drive_enabled': True,
                'limits_confirmed': True,
                'throttle_min': 0,
                'throttle_max': ParameterValue(
                    LaunchConfiguration('throttle_max'), value_type=int),
                'steering_min': ParameterValue(
                    LaunchConfiguration('steering_min'), value_type=int),
                'steering_max': ParameterValue(
                    LaunchConfiguration('steering_max'), value_type=int),
                'competition_no_stop_enabled': ParameterValue(
                    competition_no_stop, value_type=bool),
                'competition_minimum_throttle_counts': 320,
            }]),
    ])
