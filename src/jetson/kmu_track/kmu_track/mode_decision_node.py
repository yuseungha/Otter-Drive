"""Perception-only ROS node that publishes the selected mission mode."""

from __future__ import annotations

import json
import time

from geometry_msgs.msg import PoseArray
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
from std_msgs.msg import String

from kmu_track.mode_decision_core import (
    ConeModeConfig,
    ModeDecisionMachine,
)
from kmu_track.obstacle_avoidance_core import (
    ObstacleAvoidanceConfig,
    ObstacleVehicle,
    detect_obstacle_vehicle,
)


class ModeDecisionNode(Node):
    """Judge LANE/OBSTACLE_AVOID/CONE from existing perception topics."""

    def __init__(self) -> None:
        super().__init__('mode_decision')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('lane_path_topic', '/planning/lane_path')
        self.declare_parameter('cone_path_topic', '/perception/cone_path')
        self.declare_parameter('cones_topic', 'cone_planner/cones')
        self.declare_parameter('raw_cones_topic', 'cone_planner/raw_cones')
        self.declare_parameter(
            'obstacle_points_topic', '/perception/lidar_obstacle_points')
        self.declare_parameter('state_topic', '/mission/state')
        self.declare_parameter(
            'status_topic', '/vehicle/mode_decision_status')
        self.declare_parameter('evaluation_rate_hz', 20.0)
        self.declare_parameter('status_rate_hz', 5.0)
        self.declare_parameter('input_timeout_sec', 0.50)
        self._declare_dataclass(ConeModeConfig)

        # Only the perception fields of ObstacleAvoidanceConfig are exposed.
        # No avoidance path, steering, throttle, or actuator parameter exists
        # in this node.
        defaults = ObstacleAvoidanceConfig()
        for name in (
            'vehicle_half_width_m',
            'detection_min_forward_m',
            'detection_max_forward_m',
            'cone_exclusion_radius_m',
            'cluster_distance_m',
            'minimum_cluster_points',
            'minimum_cluster_span_m',
            'maximum_cluster_span_m',
            'confirm_frames',
            'clear_sec',
        ):
            self.declare_parameter(name, getattr(defaults, name))

        self.planning_frame = str(
            self.get_parameter('planning_frame').value).strip()
        if not self.planning_frame:
            raise ValueError('planning_frame cannot be empty')
        self.input_timeout_sec = self._positive('input_timeout_sec')
        evaluation_rate = self._positive('evaluation_rate_hz')
        status_rate = self._positive('status_rate_hz')

        self.cone_config = self._load_dataclass(ConeModeConfig)
        self.obstacle_config = ObstacleAvoidanceConfig(**{
            name: self.get_parameter(name).value
            for name in (
                'vehicle_half_width_m',
                'detection_min_forward_m',
                'detection_max_forward_m',
                'cone_exclusion_radius_m',
                'cluster_distance_m',
                'minimum_cluster_points',
                'minimum_cluster_span_m',
                'maximum_cluster_span_m',
                'confirm_frames',
                'clear_sec',
            )
        })
        self.obstacle_config.validate()
        self.machine = ModeDecisionMachine(
            self.cone_config,
            obstacle_clear_sec=self.obstacle_config.clear_sec,
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter('state_topic').value), latched)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), latched)

        self.lane_path = np.empty((0, 2), dtype=np.float64)
        self.cone_path = np.empty((0, 2), dtype=np.float64)
        self.cones = np.empty((0, 2), dtype=np.float64)
        self.raw_cones = np.empty((0, 2), dtype=np.float64)
        self.obstacle_points = np.empty((0, 2), dtype=np.float64)
        self.obstacle_vehicle: ObstacleVehicle | None = None
        self.obstacle_detected = False
        self._obstacle_confirm_count = 0
        self._cone_observation_id = 0
        self._last_state = self.machine.mode
        self._last_status_at = float('-inf')
        self._status_period_sec = 1.0 / status_rate

        self._lane_path_at: float | None = None
        self._cone_path_at: float | None = None
        self._cones_at: float | None = None
        self._raw_cones_at: float | None = None
        self._obstacle_points_at: float | None = None

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
        self.create_timer(1.0 / evaluation_rate, self._evaluate)
        self._publish_state('startup_lane')
        self.get_logger().info(
            'Mode decision ready; perception only, no drive output')

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be positive')
        return value

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
        return self._valid_points(points)

    def _pose_array_points(self, message: PoseArray) -> np.ndarray:
        if message.header.frame_id != self.planning_frame:
            return np.empty((0, 2), dtype=np.float64)
        points = np.asarray(tuple(
            (pose.position.x, pose.position.y) for pose in message.poses
        ), dtype=np.float64)
        return self._valid_points(points)

    @staticmethod
    def _valid_points(points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.empty((0, 2), dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            return np.empty((0, 2), dtype=np.float64)
        if not np.all(np.isfinite(points)):
            return np.empty((0, 2), dtype=np.float64)
        return points

    def _on_lane_path(self, message: Path) -> None:
        self.lane_path = self._path_points(message)
        self._lane_path_at = time.monotonic()

    def _on_cone_path(self, message: Path) -> None:
        self.cone_path = self._path_points(message)
        self._cone_path_at = time.monotonic()

    def _on_cones(self, message: PoseArray) -> None:
        self.cones = self._pose_array_points(message)
        self._cones_at = time.monotonic()
        self._cone_observation_id += 1

    def _on_raw_cones(self, message: PoseArray) -> None:
        self.raw_cones = self._pose_array_points(message)
        self._raw_cones_at = time.monotonic()

    def _on_obstacle_points(self, message: PoseArray) -> None:
        now = time.monotonic()
        self.obstacle_points = self._pose_array_points(message)
        self._obstacle_points_at = now
        lane_path = (
            self.lane_path
            if self._fresh(self._lane_path_at, now)
            else np.empty((0, 2), dtype=np.float64)
        )
        cone_groups = []
        if self._fresh(self._raw_cones_at, now) and len(self.raw_cones):
            cone_groups.append(self.raw_cones)
        if self._fresh(self._cones_at, now) and len(self.cones):
            cone_groups.append(self.cones)
        all_cones = (
            np.vstack(cone_groups)
            if cone_groups else np.empty((0, 2), dtype=np.float64)
        )
        observation = detect_obstacle_vehicle(
            self.obstacle_points,
            all_cones,
            lane_path,
            self.obstacle_config,
        )
        if observation is None:
            self._obstacle_confirm_count = 0
            self.obstacle_detected = False
        else:
            self._obstacle_confirm_count += 1
            self.obstacle_detected = bool(
                self._obstacle_confirm_count
                >= self.obstacle_config.confirm_frames
            )
        self.obstacle_vehicle = observation

    def _fresh(self, received_at: float | None, now: float) -> bool:
        return bool(
            received_at is not None
            and now - received_at <= self.input_timeout_sec
        )

    def _evaluate(self) -> None:
        now = time.monotonic()
        cones_fresh = self._fresh(self._cones_at, now)
        cone_path_fresh = self._fresh(self._cone_path_at, now)
        obstacle_fresh = self._fresh(self._obstacle_points_at, now)
        cones = (
            self.cones
            if cones_fresh else np.empty((0, 2), dtype=np.float64)
        )
        cone_path_valid = bool(cone_path_fresh and len(self.cone_path) >= 2)
        obstacle_detected = bool(
            obstacle_fresh and self.obstacle_detected)

        previous = self.machine.mode
        self.machine.update(
            cones,
            cone_path_valid=cone_path_valid,
            obstacle_detected=obstacle_detected,
            cone_observation_id=self._cone_observation_id,
            now=now,
        )
        if self.machine.mode != previous:
            self._publish_state(self.machine.reason)
        if now - self._last_status_at >= self._status_period_sec:
            self._publish_status(now, cone_path_valid, obstacle_detected)
            self._last_status_at = now

    def _publish_state(self, reason: str) -> None:
        self.state_pub.publish(String(data=self.machine.mode.value))
        self._last_state = self.machine.mode
        self.get_logger().info(
            'Mode changed to %s: %s' % (self.machine.mode.value, reason))

    def _age(self, received_at: float | None, now: float):
        return None if received_at is None else max(0.0, now - received_at)

    def _publish_status(
        self,
        now: float,
        cone_path_valid: bool,
        obstacle_detected: bool,
    ) -> None:
        gate = self.machine.last_gate
        obstacle = self.obstacle_vehicle
        self.status_pub.publish(String(data=json.dumps({
            'state': self.machine.mode.value,
            'transition_reason': self.machine.reason,
            'lane_path_valid': bool(
                self._fresh(self._lane_path_at, now)
                and len(self.lane_path) >= 2),
            'cone_path_valid': cone_path_valid,
            'obstacle_detected': obstacle_detected,
            'cone_confirm_count': self.machine.enter_count,
            'nearest_gate_x_m': (
                None if gate is None else gate.center_x_m),
            'nearest_gate_width_m': (
                None if gate is None else gate.width_m),
            'obstacle_x_m': (
                None if obstacle is None else obstacle.center_x_m),
            'input_age_sec': {
                'lane_path': self._age(self._lane_path_at, now),
                'cone_path': self._age(self._cone_path_at, now),
                'cones': self._age(self._cones_at, now),
                'raw_cones': self._age(self._raw_cones_at, now),
                'obstacle_points': self._age(
                    self._obstacle_points_at, now),
            },
        }, separators=(',', ':'))))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeDecisionNode()
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
