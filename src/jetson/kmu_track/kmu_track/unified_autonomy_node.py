"""ROS adapter for lane, obstacle-avoidance and cone Pure Pursuit."""

from __future__ import annotations

import json
import time

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Int32MultiArray, String

from kmu_track.lane_control_core import ACTUATION_GATE_REASONS
from kmu_track.unified_autonomy_core import (
    CompetitionDriveContinuity,
    ConeSwitchConfig,
    ContinuousPurePursuit,
    DriveCountConfig,
    PlannerMode,
    PlannerModeSelector,
    PurePursuitConfig,
    command_to_counts,
)
from kmu_track.obstacle_avoidance_core import (
    ObstacleAvoidanceConfig,
    ObstacleVehicle,
    detect_obstacle_vehicle,
    make_opposite_lane_path,
)


class UnifiedAutonomyNode(Node):
    """Select a planner and drive both paths with one Pure Pursuit instance."""

    def __init__(self) -> None:
        super().__init__('unified_autonomy')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('lane_path_topic', '/planning/lane_path')
        self.declare_parameter('cone_path_topic', '/perception/cone_path')
        self.declare_parameter('cones_topic', 'cone_planner/cones')
        self.declare_parameter('raw_cones_topic', 'cone_planner/raw_cones')
        self.declare_parameter(
            'obstacle_points_topic', '/perception/lidar_obstacle_points')
        self.declare_parameter('output_topic', '/rc_car/drive_cmd')
        self.declare_parameter('lane_command_topic', '/rc_car/ire_lane_cmd')
        self.declare_parameter(
            'lane_status_topic', '/vehicle/lane_control_status')
        self.declare_parameter('lane_command_timeout_sec', 0.25)
        self.declare_parameter('perception_timeout_sec', 0.35)
        self.declare_parameter('active_path_topic', '/planning/active_path')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('auto_arm_drive', False)
        self.declare_parameter('competition_no_stop_enabled', False)
        self._declare_dataclass(ConeSwitchConfig)
        self._declare_dataclass(PurePursuitConfig)
        self._declare_dataclass(DriveCountConfig)
        self._declare_dataclass(ObstacleAvoidanceConfig)

        self.planning_frame = str(
            self.get_parameter('planning_frame').value).strip()
        if not self.planning_frame:
            raise ValueError('planning_frame cannot be empty')
        rate = float(self.get_parameter('control_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self.auto_arm_drive = bool(
            self.get_parameter('auto_arm_drive').value)
        self.competition_no_stop_enabled = bool(
            self.get_parameter('competition_no_stop_enabled').value)
        self.lane_command_timeout_sec = float(
            self.get_parameter('lane_command_timeout_sec').value)
        if self.lane_command_timeout_sec <= 0.0:
            raise ValueError('lane_command_timeout_sec must be positive')
        self.perception_timeout_sec = float(
            self.get_parameter('perception_timeout_sec').value)
        if self.perception_timeout_sec <= 0.0:
            raise ValueError('perception_timeout_sec must be positive')
        self.serial_ready = False

        self.switch_config = self._load_dataclass(ConeSwitchConfig)
        self.pursuit_config = self._load_dataclass(PurePursuitConfig)
        self.count_config = self._load_dataclass(DriveCountConfig)
        self.obstacle_config = self._load_dataclass(
            ObstacleAvoidanceConfig)
        self.selector = PlannerModeSelector(
            self.switch_config,
            obstacle_clear_sec=self.obstacle_config.clear_sec,
        )
        self.controller = ContinuousPurePursuit(self.pursuit_config)
        self.continuity = CompetitionDriveContinuity(
            minimum_throttle_counts=(
                self.count_config.minimum_throttle_counts),
            maximum_throttle_counts=max(
                self.count_config.lane_throttle_counts,
                self.count_config.cone_throttle_counts,
            ),
            maximum_steering_counts=(
                self.count_config.maximum_steering_counts),
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.command_pub = self.create_publisher(
            Int32MultiArray,
            str(self.get_parameter('output_topic').value),
            10,
        )
        self.mode_pub = self.create_publisher(
            String, '/mission/state', latched)
        self.status_pub = self.create_publisher(
            String, '/vehicle/unified_autonomy_status', 10)
        self.active_path_pub = self.create_publisher(
            Path, str(self.get_parameter('active_path_topic').value), 10)
        self.target_pub = self.create_publisher(
            PointStamped, '/planning/pure_pursuit_target', 10)
        self.obstacle_vehicle_pub = self.create_publisher(
            String, '/perception/obstacle_vehicle', 10)
        self.operator_armed_pub = None
        self.operator_deadman_pub = None
        if self.auto_arm_drive:
            self.operator_armed_pub = self.create_publisher(
                Bool, '/rc_car/operator_armed', 10)
            self.operator_deadman_pub = self.create_publisher(
                Bool, '/rc_car/operator_deadman', 10)
            self.create_subscription(
                Bool,
                '/rc_car/serial_ready',
                self._on_serial_ready,
                latched,
            )

        self.lane_path = np.empty((0, 2), dtype=np.float64)
        self.cone_path = np.empty((0, 2), dtype=np.float64)
        self.cones = np.empty((0, 2), dtype=np.float64)
        self.raw_cones = np.empty((0, 2), dtype=np.float64)
        self.lidar_obstacle_points = np.empty((0, 2), dtype=np.float64)
        self.avoidance_path = np.empty((0, 2), dtype=np.float64)
        self.ire_lane_command = (0, 0)
        self._ire_lane_command_at = None
        self._ire_lane_status_at = None
        self._ire_lane_gate_active = False
        self._ire_lane_gate_reason = 'waiting_for_status'
        self._lane_path_at = None
        self._cone_path_at = None
        self._cones_at = None
        self._obstacle_points_at = None
        self._cone_observation_id = 0
        self.lane_path_valid = False
        self.cone_path_valid = False
        self.obstacle_vehicle: ObstacleVehicle | None = None
        self.obstacle_detected = False
        self._obstacle_confirm_count = 0
        self._avoidance_sign = self.obstacle_config.preferred_offset_sign
        self._last_tick = time.monotonic()

        self.create_subscription(
            Path,
            str(self.get_parameter('lane_path_topic').value),
            self._on_lane_path,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter('cone_path_topic').value),
            self._on_cone_path,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            str(self.get_parameter('lane_command_topic').value),
            self._on_ire_lane_command,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('lane_status_topic').value),
            self._on_ire_lane_status,
            10,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('cones_topic').value),
            self._on_cones,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('raw_cones_topic').value),
            self._on_raw_cones,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter('obstacle_points_topic').value),
            self._on_obstacle_points,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / rate, self._control_tick)
        self._publish_mode()
        self.get_logger().info(
            'Unified autonomy ready: IRE LANE/OBSTACLE_AVOID/CONE, '
            'output=%s, lane_command=%s, auto_arm=%s'
            % (
                str(self.get_parameter('output_topic').value),
                str(self.get_parameter('lane_command_topic').value),
                self.auto_arm_drive,
            )
        )
        if self.competition_no_stop_enabled:
            self.get_logger().warn(
                'COMPETITION NO-STOP ACTIVE: after the first valid forward '
                'command, transient perception/IRE loss retains the last '
                'positive throttle and steering command')

    def _on_serial_ready(self, message: Bool) -> None:
        self.serial_ready = bool(message.data)
        self._publish_drive_gate()

    def _publish_drive_gate(self) -> None:
        if (
            self.operator_armed_pub is None
            or self.operator_deadman_pub is None
        ):
            return
        lane_ready = self._ire_lane_ready(time.monotonic())
        mode = getattr(self.selector, 'mode', PlannerMode.LANE)
        keep_armed = bool(
            self.competition_no_stop_enabled and self.continuity.started)
        requested = Bool(data=bool(
            keep_armed
            or (
                self.serial_ready
                and (mode != PlannerMode.LANE or lane_ready)
            )
        ))
        # Deadman first lets the serial bridge remember the request before
        # processing arm. Repeating both at the control rate handles DDS
        # delivery order without a separate operator command.
        self.operator_deadman_pub.publish(requested)
        self.operator_armed_pub.publish(requested)

    def _on_ire_lane_command(self, message: Int32MultiArray) -> None:
        if len(message.data) < 2:
            self.get_logger().warn(
                'Ignoring malformed IRE lane command',
                throttle_duration_sec=2.0,
            )
            return
        self.ire_lane_command = (int(message.data[0]), int(message.data[1]))
        self._ire_lane_command_at = time.monotonic()

    def _on_ire_lane_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warn(
                'Ignoring malformed IRE lane status',
                throttle_duration_sec=2.0,
            )
            return
        reason = str(status.get('gate_reason', 'unknown'))
        self._ire_lane_gate_reason = reason
        self._ire_lane_gate_active = bool(
            status.get('publish', False)
            and reason in ACTUATION_GATE_REASONS
        )
        self._ire_lane_status_at = time.monotonic()

    def _ire_lane_ready(self, now: float) -> bool:
        if (
            self._ire_lane_command_at is None
            or self._ire_lane_status_at is None
        ):
            return False
        return bool(
            self._ire_lane_gate_active
            and now - self._ire_lane_command_at
            <= self.lane_command_timeout_sec
            and now - self._ire_lane_status_at
            <= self.lane_command_timeout_sec
        )

    def _declare_dataclass(self, data_class) -> None:
        for name, field in data_class.__dataclass_fields__.items():
            self.declare_parameter(name, field.default)

    def _load_dataclass(self, data_class):
        return data_class(**{
            name: self.get_parameter(name).value
            for name in data_class.__dataclass_fields__
        })

    def _path_points(self, message: Path) -> np.ndarray:
        if message.header.frame_id != self.planning_frame:
            self.get_logger().warn(
                'Ignoring path in %s; expected %s'
                % (message.header.frame_id, self.planning_frame),
                throttle_duration_sec=2.0,
            )
            return np.empty((0, 2), dtype=np.float64)
        points = np.asarray(tuple(
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ), dtype=np.float64)
        if points.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            return np.empty((0, 2), dtype=np.float64)
        return points if np.all(np.isfinite(points)) else np.empty((0, 2))

    def _on_lane_path(self, message: Path) -> None:
        points = self._path_points(message)
        self.lane_path_valid = len(points) >= 2
        self._lane_path_at = time.monotonic()
        if self.lane_path_valid:
            self.lane_path = points
            if self.selector.mode == PlannerMode.OBSTACLE_AVOID:
                self._refresh_avoidance_path()

    def _on_cone_path(self, message: Path) -> None:
        points = self._path_points(message)
        self.cone_path_valid = len(points) >= 2
        self._cone_path_at = time.monotonic()
        if self.cone_path_valid:
            self.cone_path = points
        self._update_mode()

    def _on_cones(self, message: PoseArray) -> None:
        self.cones = self._pose_array_points(message)
        self._cones_at = time.monotonic()
        self._cone_observation_id += 1
        self._update_mode()

    def _on_raw_cones(self, message: PoseArray) -> None:
        self.raw_cones = self._pose_array_points(message)

    def _pose_array_points(self, message: PoseArray) -> np.ndarray:
        if message.header.frame_id != self.planning_frame:
            return np.empty((0, 2), dtype=np.float64)
        points = np.asarray(tuple(
            (pose.position.x, pose.position.y) for pose in message.poses
        ), dtype=np.float64)
        return (
            points
            if points.size and points.ndim == 2 and np.all(np.isfinite(points))
            else np.empty((0, 2), dtype=np.float64)
        )

    def _all_cones(self) -> np.ndarray:
        groups = [
            values for values in (self.raw_cones, self.cones)
            if len(values)
        ]
        return (
            np.vstack(groups)
            if groups else np.empty((0, 2), dtype=np.float64)
        )

    def _on_obstacle_points(self, message: PoseArray) -> None:
        self.lidar_obstacle_points = self._pose_array_points(message)
        self._obstacle_points_at = time.monotonic()
        observation = detect_obstacle_vehicle(
            self.lidar_obstacle_points,
            self._all_cones(),
            self.lane_path,
            self.obstacle_config,
        )
        if observation is None:
            self._obstacle_confirm_count = 0
        else:
            self._obstacle_confirm_count += 1
        confirmed = (
            observation is not None
            and self._obstacle_confirm_count
            >= self.obstacle_config.confirm_frames
        )
        self.obstacle_detected = bool(confirmed)
        if confirmed:
            self.obstacle_vehicle = observation
        self._update_mode()
        self._publish_obstacle_vehicle(observation, confirmed)

    def _publish_obstacle_vehicle(
        self,
        observation: ObstacleVehicle | None,
        confirmed: bool,
    ) -> None:
        self.obstacle_vehicle_pub.publish(String(data=json.dumps({
            'detected': bool(confirmed),
            'mode': self.selector.mode.value,
            'center_x_m': (
                None if observation is None else observation.center_x_m),
            'center_y_m': (
                None if observation is None else observation.center_y_m),
            'path_distance_m': (
                None if observation is None
                else observation.path_distance_m),
            'cluster_points': (
                0 if observation is None else observation.point_count),
            'cluster_span_m': (
                None if observation is None else observation.span_m),
            'avoidance_sign': (
                None if observation is None
                else observation.avoidance_sign),
        }, separators=(',', ':'))))

    def _refresh_avoidance_path(self) -> None:
        self.avoidance_path = make_opposite_lane_path(
            self.lane_path,
            self._avoidance_sign
            * self.obstacle_config.opposite_lane_offset_m,
            self.obstacle_config.lane_change_distance_m,
        )

    def _fresh(self, observed_at: float | None, now: float) -> bool:
        return bool(
            observed_at is not None
            and now - observed_at <= self.perception_timeout_sec
        )

    def _update_mode(self, now: float | None = None) -> None:
        timestamp = time.monotonic() if now is None else float(now)
        previous = self.selector.mode
        self.selector.update(
            (
                self.cones
                if self._fresh(self._cones_at, timestamp)
                else np.empty((0, 2), dtype=np.float64)
            ),
            cone_path_valid=bool(
                self.cone_path_valid
                and self._fresh(self._cone_path_at, timestamp)
            ),
            obstacle_detected=bool(
                self.obstacle_detected
                and self._fresh(self._obstacle_points_at, timestamp)
            ),
            cone_observation_id=self._cone_observation_id,
            now=timestamp,
        )
        if self.selector.mode != previous:
            if self.selector.mode == PlannerMode.OBSTACLE_AVOID:
                self._avoidance_sign = (
                    self.obstacle_vehicle.avoidance_sign
                    if self.obstacle_vehicle is not None
                    else self.obstacle_config.preferred_offset_sign
                )
                self._refresh_avoidance_path()
            self._publish_mode()
            self.get_logger().info(
                'Planner changed %s -> %s: %s'
                % (
                    previous.value,
                    self.selector.mode.value,
                    self.selector.reason,
                )
            )

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.selector.mode.value))

    def _selected_path(self, now: float) -> np.ndarray:
        if self.selector.mode == PlannerMode.CONE:
            return (
                self.cone_path
                if self._fresh(self._cone_path_at, now)
                else np.empty((0, 2), dtype=np.float64)
            )
        if self.selector.mode == PlannerMode.OBSTACLE_AVOID:
            return (
                self.avoidance_path
                if self._fresh(self._lane_path_at, now)
                else np.empty((0, 2), dtype=np.float64)
            )
        return (
            self.lane_path
            if self._fresh(self._lane_path_at, now)
            else np.empty((0, 2), dtype=np.float64)
        )

    def _publish_active_path(self, path: np.ndarray) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.planning_frame
        for x_m, y_m in path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(x_m)
            pose.pose.position.y = float(y_m)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.active_path_pub.publish(message)

    def _control_tick(self) -> None:
        self._publish_drive_gate()
        now = time.monotonic()
        self._update_mode(now)
        dt = max(1.0e-3, now - self._last_tick)
        self._last_tick = now
        path = self._selected_path(now)
        result = self.controller.command(
            path,
            self.selector.mode,
            dt_s=dt,
        )
        ire_lane_ready = self._ire_lane_ready(now)
        if self.selector.mode == PlannerMode.LANE:
            if ire_lane_ready:
                throttle, steering = self.ire_lane_command
                command_source = 'ire_pid'
                source_valid = throttle > 0
            else:
                throttle, steering = 0, 0
                command_source = 'ire_wait'
                source_valid = False
        else:
            throttle, steering = command_to_counts(
                result,
                self.selector.mode,
                self.pursuit_config,
                self.count_config,
            )
            command_source = 'pure_pursuit'
            source_valid = True
        continuity_reason = 'disabled'
        if self.competition_no_stop_enabled:
            throttle, steering, continuity_reason = self.continuity.apply(
                throttle,
                steering,
                source_valid=source_valid,
                start_allowed=(
                    self.serial_ready if self.auto_arm_drive else True),
            )
            if continuity_reason == 'hold_last_forward':
                command_source = 'competition_hold'
        self.command_pub.publish(
            Int32MultiArray(data=[throttle, steering]))
        self._publish_drive_gate()
        self._publish_active_path(path)

        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.planning_frame
        target.point.x = result.target_x_m
        target.point.y = result.target_y_m
        self.target_pub.publish(target)

        pair = self.selector.last_pair
        self.status_pub.publish(String(data=json.dumps({
            'mode': self.selector.mode.value,
            'auto_arm_drive': self.auto_arm_drive,
            'competition_no_stop_enabled': (
                self.competition_no_stop_enabled),
            'continuous_drive_started': self.continuity.started,
            'continuity_reason': continuity_reason,
            'serial_ready': self.serial_ready,
            'command_source': command_source,
            'ire_lane_ready': ire_lane_ready,
            'ire_lane_gate_active': self._ire_lane_gate_active,
            'ire_lane_gate_reason': self._ire_lane_gate_reason,
            'transition_reason': self.selector.reason,
            'controller_reason': result.reason,
            'selected_path_points': len(path),
            'lane_path_valid': self.lane_path_valid,
            'cone_path_valid': self.cone_path_valid,
            'obstacle_detected': self.obstacle_detected,
            'obstacle_x_m': (
                None if self.obstacle_vehicle is None
                else self.obstacle_vehicle.center_x_m),
            'avoidance_sign': self._avoidance_sign,
            'opposite_lane_offset_m': (
                self.obstacle_config.opposite_lane_offset_m),
            'nearest_pair_m': (
                None if pair is None else pair.center_x_m),
            'pair_width_m': None if pair is None else pair.width_m,
            'valid_pair_count': self.selector.last_pair_count,
            'target_x_m': result.target_x_m,
            'target_y_m': result.target_y_m,
            'curvature_1pm': result.curvature_1pm,
            'speed_mps': result.speed_mps,
            'steering_angle_rad': result.steering_angle_rad,
            'throttle': throttle,
            'steering': steering,
        }, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnifiedAutonomyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
