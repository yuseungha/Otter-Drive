"""Always-on drivers with mutually exclusive perception subscriptions."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _share(package: str, directory: str, filename: str):
    return PathJoinSubstitution(
        [FindPackageShare(package), directory, filename])


def generate_launch_description() -> LaunchDescription:
    dry_run = LaunchConfiguration('dry_run')
    hardware_confirmed = LaunchConfiguration('hardware_confirmed')
    serial_bridge = LaunchConfiguration('serial_bridge')
    live_serial = IfCondition(PythonExpression([
        "'", serial_bridge, "' == 'true' and '", dry_run,
        "' == 'false' and '", hardware_confirmed, "' == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('hardware_confirmed', default_value='false'),
        DeclareLaunchArgument('steering_only', default_value='true'),
        DeclareLaunchArgument('serial_bridge', default_value='false'),
        DeclareLaunchArgument(
            'camera_device', default_value=EnvironmentVariable(
                'KMU_CAMERA_DEVICE', default_value='/dev/video0')),
        DeclareLaunchArgument('model_path', default_value=EnvironmentVariable(
            'KMU_MODEL_PATH', default_value='')),
        DeclareLaunchArgument('lidar_port', default_value=EnvironmentVariable(
            'KMU_LIDAR_DEVICE', default_value='/dev/ttyUSB0')),
        DeclareLaunchArgument(
            'arduino_port', default_value=EnvironmentVariable(
                'KMU_SERIAL_DEVICE', default_value=(
                    '/dev/serial/by-id/REPLACE_WITH_ARDUINO_DEVICE'))),
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        DeclareLaunchArgument('planning_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        DeclareLaunchArgument('throttle_max', default_value='300'),
        DeclareLaunchArgument('steering_min', default_value='-900'),
        DeclareLaunchArgument('steering_max', default_value='900'),
        DeclareLaunchArgument(
            'cone_geometry_confirmed', default_value='false'),
        DeclareLaunchArgument('require_odometry', default_value='false'),
        DeclareLaunchArgument(
            'camera_config', default_value=_share(
                'kmu_track', 'config', 'camera.yaml')),
        DeclareLaunchArgument(
            'perception_config', default_value=_share(
                'kmu_track', 'config', 'perception.yaml')),
        DeclareLaunchArgument(
            'lane_control_config', default_value=_share(
                'kmu_track', 'config', 'lane_control.yaml')),
        DeclareLaunchArgument(
            'sensor_mode_config', default_value=_share(
                'kmu_track', 'config', 'sensor_mode.yaml')),
        DeclareLaunchArgument(
            'cone_planner_config', default_value=_share(
                'lidar_cone_planner', 'config', 'cone_planner.yaml')),
        DeclareLaunchArgument(
            'cone_controller_config', default_value=_share(
                'lidar_cone_planner', 'config', 'cone_controller.yaml')),
        DeclareLaunchArgument(
            'lidar_system_config', default_value=_share(
                'lidar_cone_planner', 'config', 'cone_lidar_cv.yaml')),
        DeclareLaunchArgument(
            'adapter_config', default_value=_share(
                'rc_car_teleop', 'config', 'autonomous_drive.yaml')),

        # Sensor publishers/drivers are unconditional and stay alive through
        # every mission state. Only the downstream subscriptions are switched.
        Node(
            package='kmu_track', executable='usb_camera_source',
            name='usb_camera_source', output='screen',
            parameters=[LaunchConfiguration('camera_config'), {
                'device': LaunchConfiguration('camera_device')}]),
        Node(
            package='rplidar_ros', executable='rplidar_node',
            name='rplidar_node', output='screen', parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 115200,
                'frame_id': LaunchConfiguration('laser_frame'),
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }], remappings=[('scan', LaunchConfiguration('scan_topic'))]),
        Node(
            package='lidar_cone_planner',
            executable='cone_lidar_static_tf',
            name='cone_lidar_static_tf', output='screen',
            parameters=[LaunchConfiguration('lidar_system_config'), {
                'planning_frame': LaunchConfiguration('planning_frame'),
                'laser_frame': LaunchConfiguration('laser_frame')}]),

        Node(
            package='kmu_track', executable='sensor_mode_manager',
            name='sensor_mode_manager', output='screen',
            parameters=[LaunchConfiguration('sensor_mode_config')]),
        Node(
            package='kmu_track', executable='yolo_lane_detector',
            name='yolo_lane_detector', output='screen',
            parameters=[LaunchConfiguration('perception_config'), {
                'model_path': LaunchConfiguration('model_path'),
                'managed_subscription': True}]),
        Node(
            package='lidar_cone_planner', executable='cone_line_planner',
            name='cone_line_planner', output='screen',
            parameters=[LaunchConfiguration('cone_planner_config'), {
                'scan_topic': LaunchConfiguration('scan_topic'),
                'planning_frame': LaunchConfiguration('planning_frame'),
                'managed_subscription': True}]),

        Node(
            package='kmu_track', executable='lane_control',
            name='lane_control', output='screen',
            parameters=[LaunchConfiguration('lane_control_config'), {
                'enabled': True,
                'dry_run': ParameterValue(dry_run, value_type=bool),
                'hardware_confirmed': ParameterValue(
                    hardware_confirmed, value_type=bool),
                'steering_only': ParameterValue(
                    LaunchConfiguration('steering_only'), value_type=bool),
                'ignore_mission_state': False,
                'active_states': ['LANE_FOLLOW'],
                'command_topic': '/vehicle/lane_drive_cmd'}]),
        Node(
            package='lidar_cone_planner', executable='cone_pure_pursuit',
            name='cone_pure_pursuit', output='screen',
            parameters=[LaunchConfiguration('cone_controller_config'), {
                'planning_frame': LaunchConfiguration('planning_frame'),
                'enabled_on_startup': True,
                'geometry_confirmed': ParameterValue(
                    LaunchConfiguration('cone_geometry_confirmed'),
                    value_type=bool),
                'require_odometry': ParameterValue(
                    LaunchConfiguration('require_odometry'), value_type=bool),
                'allow_compat_command': True,
                'enforce_mission_state': True,
                'active_mission_state': 'CONE_SLALOM'}]),
        Node(
            package='rc_car_teleop', executable='ackermann_to_drive_cmd',
            name='ackermann_to_drive_cmd', output='screen',
            parameters=[LaunchConfiguration('adapter_config'), {
                'throttle_max': ParameterValue(
                    LaunchConfiguration('throttle_max'), value_type=int)}]),
        Node(
            package='kmu_track', executable='mode_command_mux',
            name='mode_command_mux', output='screen',
            parameters=[LaunchConfiguration('sensor_mode_config'), {
                'dry_run': ParameterValue(dry_run, value_type=bool)}]),

        Node(
            package='rc_car_teleop', executable='serial_bridge',
            name='serial_bridge', output='screen', condition=live_serial,
            parameters=[{
                'serial_port': LaunchConfiguration('arduino_port'),
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
