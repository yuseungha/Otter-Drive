"""Camera lane + two-line LiDAR cone planning with one Pure Pursuit."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    viewer = LaunchConfiguration('viewer')

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
            'output_topic', default_value='/rc_car/drive_cmd'),
        DeclareLaunchArgument('serial_bridge', default_value='false'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('throttle_max', default_value='700'),
        DeclareLaunchArgument('steering_min', default_value='-650'),
        DeclareLaunchArgument('steering_max', default_value='650'),
        DeclareLaunchArgument(
            'camera_config',
            default_value=_config('kmu_track', 'camera.yaml')),
        DeclareLaunchArgument(
            'segmentation_config',
            default_value=_config('kmu_track', 'segmentation_lane.yaml')),
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
            package='kmu_track', executable='usb_camera_source',
            name='usb_camera_source', output='screen',
            parameters=[LaunchConfiguration('camera_config'), {
                'device': camera_device}]),
        Node(
            package='kmu_track', executable='yolo_seg_lane_detector',
            name='yolo_seg_lane_detector', output='screen',
            parameters=[LaunchConfiguration('segmentation_config'), {
                'model_path': model_path,
                'planning_frame': planning_frame,
                'managed_subscription': True}]),

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
                # Live mode starts driving as soon as the existing serial
                # reset/firmware handshake reports ready.
                'auto_arm_drive': ParameterValue(
                    serial_bridge, value_type=bool)}]),

        # Optional transport to the existing Arduino protocol.  Planner loss
        # does not publish neutral; the unified controller keeps moving.
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
            }]),
    ])
