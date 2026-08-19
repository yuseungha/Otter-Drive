"""Fail-closed ROS 2 wrapper for the local 2D LiDAR cone planner."""

from collections import deque
from math import atan2, cos, sin
import time
import threading
from typing import Iterable

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .planner_core import (
    ConeTrackFilter,
    PlanResult,
    PlannerConfig,
    detect_cones_from_scan,
    empty_plan_result,
    plan_centerline,
)
from .preview_core import PreviewConfig, PreviewResult, compute_path_preview, invalid_preview
from .cone_end_core import ConeEndConfig, ConeEndDetector


class ConeLinePlanner(Node):
    """Detect persistent cone clusters and publish a safe local center path."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "cone_line_planner", parameter_overrides=parameter_overrides or []
        )

        defaults = PlannerConfig()
        self._config_names = frozenset(vars(defaults))
        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("scan_topic", "scan", read_only)
        self.declare_parameter("path_topic", "cone_planner/center_path", read_only)
        self.declare_parameter("cones_topic", "cone_planner/cones", read_only)
        self.declare_parameter("raw_cones_topic", "cone_planner/raw_cones", read_only)
        self.declare_parameter("markers_topic", "cone_planner/markers", read_only)
        self.declare_parameter("status_topic", "cone_planner/status", read_only)
        self.declare_parameter("planning_frame", "base_link", read_only)
        self.declare_parameter("scan_timeout_s", 0.35, read_only)
        self.declare_parameter("max_scan_age_s", 0.30, read_only)
        self.declare_parameter("max_future_scan_s", 0.05, read_only)
        self.declare_parameter("tf_timeout_s", 0.05, read_only)
        self.declare_parameter("enforce_scan_timestamp", True, read_only)
        self.declare_parameter("watchdog_period_s", 0.05, read_only)
        self.declare_parameter("validation_wheelbase_m", 0.20, read_only)
        self.declare_parameter("lookahead_min_m", 0.25, read_only)
        self.declare_parameter("lookahead_max_m", 0.45, read_only)
        self.declare_parameter("lookahead_time_s", 0.75, read_only)
        self.declare_parameter("validation_speed_mps", 0.0, read_only)
        self.declare_parameter("managed_subscription", False, read_only)
        self.declare_parameter("cone_end_x_min", 0.3)
        self.declare_parameter("cone_end_x_max", 0.7)
        self.declare_parameter("cone_end_left_y_min", 0.15)
        self.declare_parameter("cone_end_left_y_max", 1.0)
        self.declare_parameter("cone_end_right_y_min", -1.0)
        self.declare_parameter("cone_end_right_y_max", -0.15)
        self.declare_parameter("cone_end_empty_scans", 5)
        self.declare_parameter("cone_min_mode_duration_sec", 1.0)

        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)

        self.config = self._config_from_parameters()
        self.track_filter = ConeTrackFilter(self.config)
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.cones_topic = str(self.get_parameter("cones_topic").value)
        self.raw_cones_topic = str(self.get_parameter("raw_cones_topic").value)
        self.markers_topic = str(self.get_parameter("markers_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.scan_timeout_s = self._positive_parameter("scan_timeout_s")
        self.max_scan_age_s = self._positive_parameter("max_scan_age_s")
        self.max_future_scan_s = self._nonnegative_parameter("max_future_scan_s")
        self.tf_timeout_s = self._nonnegative_parameter("tf_timeout_s")
        self.watchdog_period_s = self._positive_parameter("watchdog_period_s")
        self.enforce_scan_timestamp = bool(
            self.get_parameter("enforce_scan_timestamp").value
        )
        self.preview_config = PreviewConfig(
            wheelbase_m=float(
                self.get_parameter("validation_wheelbase_m").value
            ),
            lookahead_min_m=float(self.get_parameter("lookahead_min_m").value),
            lookahead_max_m=float(self.get_parameter("lookahead_max_m").value),
            lookahead_time_s=float(self.get_parameter("lookahead_time_s").value),
            validation_speed_mps=float(
                self.get_parameter("validation_speed_mps").value
            ),
        )
        self.preview_config.validate()
        self.end_detector = ConeEndDetector(ConeEndConfig(
            x_min=float(self.get_parameter("cone_end_x_min").value),
            x_max=float(self.get_parameter("cone_end_x_max").value),
            left_y_min=float(
                self.get_parameter("cone_end_left_y_min").value),
            left_y_max=float(
                self.get_parameter("cone_end_left_y_max").value),
            right_y_min=float(
                self.get_parameter("cone_end_right_y_min").value),
            right_y_max=float(
                self.get_parameter("cone_end_right_y_max").value),
            empty_scans=int(
                self.get_parameter("cone_end_empty_scans").value),
            min_mode_duration_sec=float(
                self.get_parameter("cone_min_mode_duration_sec").value),
        ))
        if not self.planning_frame:
            raise ValueError("planning_frame cannot be empty")

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.path_publisher = self.create_publisher(
            Path, self.path_topic, reliable_latest
        )
        self.mission_path_publisher = self.create_publisher(
            Path, "/perception/cone_path", reliable_latest
        )
        self.cones_publisher = self.create_publisher(
            PoseArray, self.cones_topic, debug_qos
        )
        self.raw_cones_publisher = self.create_publisher(
            PoseArray, self.raw_cones_topic, debug_qos
        )
        self.markers_publisher = self.create_publisher(
            MarkerArray, self.markers_topic, debug_qos
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.status_topic, reliable_latest
        )
        activity_qos = QoSProfile(depth=1)
        activity_qos.reliability = ReliabilityPolicy.RELIABLE
        activity_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.subscription_active_publisher = self.create_publisher(
            Bool, "/perception/lidar_subscription_active", activity_qos
        )
        self.heartbeat_publisher = self.create_publisher(
            Header, "/perception/lidar_heartbeat", 10
        )
        self.cone_finished_publisher = self.create_publisher(
            Bool, "/perception/cone_finished", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._scan_qos = scan_qos
        self._subscription_condition = threading.Condition()
        self._scan_callbacks_in_flight = 0
        self._accept_scan_work = False
        self.scan_subscription = None
        self._managed_subscription = bool(
            self.get_parameter("managed_subscription").value)
        self._mission_mode = "LANE_FOLLOW"
        self._camera_subscription_active = True
        self._started_monotonic = time.monotonic()
        self._last_scan_monotonic: float | None = None
        self._last_scan_stamp_ns: int | None = None
        self._scan_periods_s: deque[float] = deque(maxlen=60)
        self._last_status_log_monotonic = 0.0
        self._watchdog_invalid_sent = False
        self._last_frame_id = ""
        self.watchdog_timer = self.create_timer(
            self.watchdog_period_s, self._watchdog_callback
        )
        self.create_subscription(
            String, "/mission/state", self._mission_state_callback, activity_qos
        )
        self.create_subscription(
            Bool,
            "/perception/camera_subscription_active",
            self._camera_activity_callback,
            activity_qos,
        )
        if self._managed_subscription:
            self.subscription_active_publisher.publish(Bool(data=False))
        else:
            self._activate_scan_subscription()

        self.get_logger().info(
            "Cone planner ready: input=%s, planning_frame=%s, width=%.3f m, "
            "confirmation=%d scans"
            % (
                self.scan_topic,
                self.planning_frame,
                self.config.track_width_m,
                self.config.track_confirmation_scans,
            )
        )

    def _config_from_parameters(self) -> PlannerConfig:
        return PlannerConfig(
            **{
                name: self.get_parameter(name).value
                for name in self._config_names
            }
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not np.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not np.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
        return value

    def _on_parameter_change(self, parameters) -> SetParametersResult:
        updates = {parameter.name: parameter.value for parameter in parameters}
        config_updates = {
            name: value for name, value in updates.items() if name in self._config_names
        }
        if not config_updates:
            return SetParametersResult(successful=True)
        values = vars(self.config).copy()
        values.update(config_updates)
        try:
            new_config = PlannerConfig(**values)
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        self.config = new_config
        self.track_filter = ConeTrackFilter(new_config)
        self.get_logger().warn(
            "Planner parameters changed; cone temporal tracks were reset"
        )
        return SetParametersResult(successful=True)

    @staticmethod
    def _stamp_ns(header: Header) -> int:
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _make_header(self, stamp=None) -> Header:
        header = Header()
        header.frame_id = self.planning_frame
        header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        return header

    def _validate_scan(self, scan: LaserScan) -> tuple[bool, str, float]:
        if not scan.header.frame_id:
            return False, "EMPTY_FRAME", float("nan")
        metadata = (
            scan.angle_min,
            scan.angle_max,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            scan.scan_time,
            scan.time_increment,
        )
        if not all(np.isfinite(float(value)) for value in metadata):
            return False, "BAD_SCAN_GEOMETRY", float("nan")
        if abs(float(scan.angle_increment)) < 1.0e-12 or len(scan.ranges) == 0:
            return False, "BAD_SCAN_GEOMETRY", float("nan")
        if (
            scan.range_min < 0.0
            or scan.range_min >= scan.range_max
            or scan.scan_time < 0.0
            or scan.time_increment < 0.0
        ):
            return False, "BAD_SCAN_GEOMETRY", float("nan")
        expected_angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        angle_tolerance = max(1.0e-4, 1.5 * abs(float(scan.angle_increment)))
        if abs(float(scan.angle_max - expected_angle_max)) > angle_tolerance:
            return False, "BAD_SCAN_GEOMETRY", float("nan")

        stamp_ns = self._stamp_ns(scan.header)
        if self.enforce_scan_timestamp and stamp_ns <= 0:
            return False, "ZERO_STAMP", float("nan")
        if (
            self.enforce_scan_timestamp
            and self._last_scan_stamp_ns is not None
            and stamp_ns <= self._last_scan_stamp_ns
        ):
            return False, "OUT_OF_ORDER_SCAN", float("nan")

        now_ns = self.get_clock().now().nanoseconds
        age_s = (now_ns - stamp_ns) * 1.0e-9 if stamp_ns > 0 else 0.0
        if self.enforce_scan_timestamp and age_s > self.max_scan_age_s:
            return False, "STALE_SCAN", age_s
        if self.enforce_scan_timestamp and age_s < -self.max_future_scan_s:
            return False, "FUTURE_SCAN", age_s
        return True, "OK", age_s

    def _sensor_to_planning(self, scan: LaserScan) -> tuple[float, float, float]:
        if scan.header.frame_id == self.planning_frame:
            return 0.0, 0.0, 0.0
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            scan.header.frame_id,
            Time.from_msg(scan.header.stamp),
            timeout=Duration(seconds=self.tf_timeout_s),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(translation.x), float(translation.y), float(yaw)

    def _mission_state_callback(self, message: String) -> None:
        previous = self._mission_mode
        mode = str(message.data).strip().upper()
        if mode not in {
            "LANE_FOLLOW", "CONE_INIT", "CONE_SLALOM",
            "LANE_REACQUIRE", "SAFE_STOP",
        }:
            mode = "SAFE_STOP"
        self._mission_mode = mode
        if mode == "CONE_SLALOM" and previous != mode:
            self.end_detector.enter(time.monotonic())
        if not self._managed_subscription:
            return
        if mode not in {"CONE_INIT", "CONE_SLALOM"}:
            self._deactivate_scan_subscription()
        else:
            self._reconcile_scan_subscription()

    def _camera_activity_callback(self, message: Bool) -> None:
        self._camera_subscription_active = bool(message.data)
        if self._managed_subscription:
            self._reconcile_scan_subscription()

    def _reconcile_scan_subscription(self) -> None:
        allowed = (
            self._mission_mode in {"CONE_INIT", "CONE_SLALOM"}
            and not self._camera_subscription_active
        )
        if allowed:
            self._activate_scan_subscription()
        else:
            self._deactivate_scan_subscription()

    def _activate_scan_subscription(self) -> None:
        with self._subscription_condition:
            if self.scan_subscription is not None:
                return
            self._accept_scan_work = True
            self._started_monotonic = time.monotonic()
            self._last_scan_monotonic = None
            self._watchdog_invalid_sent = False
            self.scan_subscription = self.create_subscription(
                LaserScan, self.scan_topic, self.scan_callback, self._scan_qos
            )
        self.subscription_active_publisher.publish(Bool(data=True))
        self.get_logger().info("LiDAR perception subscription activated")

    def _deactivate_scan_subscription(self) -> None:
        with self._subscription_condition:
            subscription = self.scan_subscription
            if subscription is None:
                return
            self._accept_scan_work = False
            self.scan_subscription = None
        self.destroy_subscription(subscription)
        with self._subscription_condition:
            while self._scan_callbacks_in_flight:
                self._subscription_condition.wait(timeout=0.10)
        self.track_filter.reset()
        self._last_scan_monotonic = None
        self._last_scan_stamp_ns = None
        self._publish_invalid(
            self._make_header(), "MODE_INACTIVE",
            detail="LaserScan subscription destroyed and callbacks drained",
        )
        self.subscription_active_publisher.publish(Bool(data=False))
        self.get_logger().info(
            "LiDAR perception subscription inactive; callbacks drained")

    def scan_callback(self, scan: LaserScan) -> None:
        with self._subscription_condition:
            if not self._accept_scan_work:
                return
            self._scan_callbacks_in_flight += 1
        try:
            self._process_scan(scan)
        finally:
            with self._subscription_condition:
                self._scan_callbacks_in_flight -= 1
                if self._scan_callbacks_in_flight == 0:
                    self._subscription_condition.notify_all()

    def _process_scan(self, scan: LaserScan) -> None:
        callback_started = time.perf_counter()
        received_monotonic = time.monotonic()
        self._last_scan_monotonic = received_monotonic
        self._watchdog_invalid_sent = False
        self._last_frame_id = scan.header.frame_id
        header = self._make_header(scan.header.stamp)
        self.heartbeat_publisher.publish(scan.header)

        valid_scan, scan_status, scan_age_s = self._validate_scan(scan)
        stamp_ns = self._stamp_ns(scan.header)
        if not valid_scan:
            self.track_filter.reset()
            self._publish_invalid(
                header,
                scan_status,
                scan_age_s=scan_age_s,
                processing_ms=(time.perf_counter() - callback_started) * 1000.0,
            )
            return
        if self._last_scan_stamp_ns is not None:
            period_s = (stamp_ns - self._last_scan_stamp_ns) * 1.0e-9
            if np.isfinite(period_s) and period_s > 0.0:
                self._scan_periods_s.append(period_s)
        self._last_scan_stamp_ns = stamp_ns

        try:
            transform = self._sensor_to_planning(scan)
            raw_candidates = detect_cones_from_scan(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                self.config,
                sensor_to_planning=transform,
                sensor_range_min_m=scan.range_min,
                sensor_range_max_m=scan.range_max,
            )
            confirmed_cones = self.track_filter.update(raw_candidates)
            result = plan_centerline(confirmed_cones, self.config)
            if (
                self._mission_mode == "CONE_SLALOM"
                and self.end_detector.update(
                    confirmed_cones, time.monotonic())
            ):
                self.cone_finished_publisher.publish(Bool(data=True))
                self.get_logger().info(
                    "cone_finished after consecutive empty LiDAR ROIs")
            preview = (
                compute_path_preview(result.path, self.preview_config)
                if result.valid
                else invalid_preview(result.status)
            )
            if result.valid and not preview.valid:
                result = empty_plan_result(
                    "NO_LOOKAHEAD", candidate_count=len(confirmed_cones)
                )
        except TransformException as error:
            self.track_filter.reset()
            self._publish_invalid(
                header,
                "TF_ERROR",
                detail=str(error),
                scan_age_s=scan_age_s,
                processing_ms=(time.perf_counter() - callback_started) * 1000.0,
            )
            return
        except (TypeError, ValueError, FloatingPointError) as error:
            self.track_filter.reset()
            self._publish_invalid(
                header,
                "PROCESSING_ERROR",
                detail=str(error),
                scan_age_s=scan_age_s,
                processing_ms=(time.perf_counter() - callback_started) * 1000.0,
            )
            return

        processing_ms = (time.perf_counter() - callback_started) * 1000.0
        scan_hz = (
            1.0 / float(np.median(self._scan_periods_s))
            if self._scan_periods_s
            else 0.0
        )
        self.raw_cones_publisher.publish(
            self._make_pose_array(header, raw_candidates)
        )
        self.cones_publisher.publish(self._make_pose_array(header, confirmed_cones))
        self.markers_publisher.publish(
            self._make_markers(header, raw_candidates, confirmed_cones, result)
        )
        # An empty Path is an explicit cancellation of the preceding valid path.
        path_points = result.path if result.valid else np.empty((0, 2), dtype=float)
        path_message = self._make_path(header, path_points)
        self.path_publisher.publish(path_message)
        self.mission_path_publisher.publish(path_message)
        self._publish_status(
            header,
            result.status,
            level=DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN,
            values={
                "scan_age_s": scan_age_s,
                "scan_hz": scan_hz,
                "processing_ms": processing_ms,
                "raw_candidates": len(raw_candidates),
                "confirmed_cones": len(confirmed_cones),
                "matched_pairs": result.matched_pair_count,
                "real_pairs": result.real_pair_count,
                "virtual_pairs": result.virtual_pair_count,
                "path_points": len(result.path),
                "path_length_m": result.path_length_m,
                "confidence": result.confidence,
                "max_curvature_1pm": result.max_curvature_1pm,
                "cone_center_radial_offset_m": (
                    self.config.cone_center_radial_offset_m
                ),
                "cone_center_bias_note": "surface_centroid_is_sensor_near",
                "planning_frame": self.planning_frame,
                "scan_transform_time": "scan_stamp",
                "center_offset_policy": "local_tangent_left_normal",
                "racing_line_offset": "disabled",
                **self._preview_values(preview),
            },
        )
        self._rate_limited_log(result, len(raw_candidates), len(confirmed_cones))

    def _watchdog_callback(self) -> None:
        if self.scan_subscription is None:
            return
        now = time.monotonic()
        reference = (
            self._last_scan_monotonic
            if self._last_scan_monotonic is not None
            else self._started_monotonic
        )
        age_s = now - reference
        if age_s <= self.scan_timeout_s or self._watchdog_invalid_sent:
            return
        self._watchdog_invalid_sent = True
        self.track_filter.reset()
        status = "NO_SCAN" if self._last_scan_monotonic is None else "SCAN_TIMEOUT"
        self._publish_invalid(
            self._make_header(),
            status,
            detail=(
                "No LaserScan received since startup"
                if self._last_scan_monotonic is None
                else f"Last LaserScan received {age_s:.3f}s ago"
            ),
            scan_age_s=age_s,
        )

    def destroy_node(self) -> bool:
        self._deactivate_scan_subscription()
        return super().destroy_node()

    def _publish_invalid(
        self,
        header: Header,
        status: str,
        *,
        detail: str = "",
        scan_age_s: float = float("nan"),
        processing_ms: float = 0.0,
    ) -> None:
        path_message = self._make_path(header, np.empty((0, 2)))
        self.path_publisher.publish(path_message)
        self.mission_path_publisher.publish(path_message)
        self.raw_cones_publisher.publish(
            self._make_pose_array(header, np.empty((0, 2)))
        )
        self.cones_publisher.publish(self._make_pose_array(header, np.empty((0, 2))))
        result = empty_plan_result(status)
        self.markers_publisher.publish(
            self._make_markers(
                header, np.empty((0, 2)), np.empty((0, 2)), result
            )
        )
        self._publish_status(
            header,
            status,
            level=DiagnosticStatus.ERROR,
            message=detail or status,
            values={
                "scan_age_s": scan_age_s,
                "processing_ms": processing_ms,
                "last_scan_frame": self._last_frame_id,
                "scan_hz": 0.0,
                "raw_candidates": 0,
                "confirmed_cones": 0,
                "matched_pairs": 0,
                "real_pairs": 0,
                "virtual_pairs": 0,
                "path_points": 0,
                "path_length_m": 0.0,
                "confidence": 0.0,
                "max_curvature_1pm": 0.0,
                "cone_center_radial_offset_m": (
                    self.config.cone_center_radial_offset_m
                ),
                "cone_center_bias_note": "surface_centroid_is_sensor_near",
                "planning_frame": self.planning_frame,
                "scan_transform_time": "scan_stamp",
                "center_offset_policy": "local_tangent_left_normal",
                "racing_line_offset": "disabled",
                **self._preview_values(invalid_preview(status)),
            },
        )
        now = time.monotonic()
        if now - self._last_status_log_monotonic >= 2.0:
            self._last_status_log_monotonic = now
            self.get_logger().error(f"Planner invalid: {status}: {detail}")

    def _publish_status(
        self,
        header: Header,
        status: str,
        *,
        level: int,
        message: str | None = None,
        values: dict[str, object] | None = None,
    ) -> None:
        diagnostic = DiagnosticArray()
        diagnostic.header = header
        item = DiagnosticStatus()
        item.level = level
        item.name = f"{self.get_fully_qualified_name()}: cone planner"
        item.hardware_id = "2d_lidar"
        item.message = message or status
        item.values = [KeyValue(key="status", value=status)]
        for key, value in (values or {}).items():
            if isinstance(value, float):
                text = f"{value:.6g}" if np.isfinite(value) else "nan"
            else:
                text = str(value)
            item.values.append(KeyValue(key=key, value=text))
        diagnostic.status.append(item)
        self.status_publisher.publish(diagnostic)

    def _preview_values(self, preview: PreviewResult) -> dict[str, object]:
        return {
            "lookahead_valid": preview.valid,
            "lookahead_reason": preview.reason,
            "lookahead_m": preview.lookahead_m,
            "lookahead_x_m": preview.target_x_m,
            "lookahead_y_m": preview.target_y_m,
            "target_heading_rad": preview.heading_rad,
            "pure_pursuit_curvature_1pm": preview.curvature_1pm,
            "expected_steering_angle_rad": preview.steering_angle_rad,
            "validation_speed_mps": self.preview_config.validation_speed_mps,
        }

    def _rate_limited_log(
        self, result: PlanResult, raw_count: int, confirmed_count: int
    ) -> None:
        now = time.monotonic()
        if now - self._last_status_log_monotonic < 2.0:
            return
        self._last_status_log_monotonic = now
        self.get_logger().info(
            "status=%s raw=%d confirmed=%d pairs=%d path=%.2fm confidence=%.2f"
            % (
                result.status,
                raw_count,
                confirmed_count,
                result.matched_pair_count,
                result.path_length_m,
                result.confidence,
            )
        )

    @staticmethod
    def _make_pose_array(header: Header, points: np.ndarray) -> PoseArray:
        message = PoseArray()
        message.header = header
        for x, y in points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    @staticmethod
    def _path_yaw(path: np.ndarray, index: int) -> float:
        if index == 0:
            delta = path[1] - path[0]
        elif index == len(path) - 1:
            delta = path[-1] - path[-2]
        else:
            delta = path[index + 1] - path[index - 1]
        return atan2(float(delta[1]), float(delta[0]))

    def _make_path(self, header: Header, points: np.ndarray) -> Path:
        message = Path()
        message.header = header
        for index, (x, y) in enumerate(points):
            yaw = self._path_yaw(points, index)
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = sin(0.5 * yaw)
            pose.pose.orientation.w = cos(0.5 * yaw)
            message.poses.append(pose)
        return message

    @staticmethod
    def _color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
        return ColorRGBA(r=red, g=green, b=blue, a=alpha)

    @staticmethod
    def _points(values: Iterable[Iterable[float]]) -> list[Point]:
        return [Point(x=float(value[0]), y=float(value[1]), z=0.0) for value in values]

    @staticmethod
    def _base_marker(
        header: Header,
        marker_id: int,
        marker_type: int,
        namespace: str,
    ) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _make_markers(
        self,
        header: Header,
        raw_cones: np.ndarray,
        confirmed_cones: np.ndarray,
        result: PlanResult,
    ) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        raw = self._base_marker(header, 0, Marker.POINTS, "raw_candidates")
        raw.scale.x = 0.045
        raw.scale.y = 0.045
        raw.color = self._color(1.0, 0.75, 0.15, 0.55)
        raw.points = self._points(raw_cones)
        markers.markers.append(raw)

        candidates = self._base_marker(
            header, 1, Marker.POINTS, "confirmed_cones"
        )
        candidates.scale.x = 0.08
        candidates.scale.y = 0.08
        candidates.color = self._color(1.0, 0.45, 0.0)
        candidates.points = self._points(confirmed_cones)
        markers.markers.append(candidates)

        left = self._base_marker(header, 2, Marker.LINE_STRIP, "matched_left")
        left.scale.x = 0.035
        left.color = self._color(0.1, 0.45, 1.0)
        left.points = self._points(result.left_boundary)
        markers.markers.append(left)

        right = self._base_marker(header, 3, Marker.LINE_STRIP, "matched_right")
        right.scale.x = 0.035
        right.color = self._color(1.0, 0.15, 0.12)
        right.points = self._points(result.right_boundary)
        markers.markers.append(right)

        pairs = self._base_marker(header, 4, Marker.LINE_LIST, "matched_pairs")
        pairs.scale.x = 0.015
        pairs.color = self._color(0.65, 0.65, 0.65, 0.8)
        for left_point, right_point in zip(
            result.left_boundary, result.right_boundary
        ):
            pairs.points.extend(self._points((left_point, right_point)))
        markers.markers.append(pairs)

        raw_center = self._base_marker(header, 5, Marker.POINTS, "raw_center")
        raw_center.scale.x = 0.07
        raw_center.scale.y = 0.07
        raw_center.color = self._color(0.9, 0.1, 1.0)
        raw_center.points = self._points(result.raw_centerline)
        markers.markers.append(raw_center)

        path = self._base_marker(header, 6, Marker.LINE_STRIP, "center_path")
        path.scale.x = 0.05
        path.color = self._color(0.1, 1.0, 0.2)
        path.points = self._points(result.path)
        markers.markers.append(path)

        virtual_left = self._base_marker(
            header, 8, Marker.POINTS, "virtual_left"
        )
        virtual_left.scale.x = 0.11
        virtual_left.scale.y = 0.11
        virtual_left.color = self._color(0.2, 0.9, 1.0, 0.85)
        virtual_right = self._base_marker(
            header, 9, Marker.POINTS, "virtual_right"
        )
        virtual_right.scale.x = 0.11
        virtual_right.scale.y = 0.11
        virtual_right.color = self._color(1.0, 0.25, 0.85, 0.85)
        if result.virtual_pair_count:
            virtual_left.points = self._points(
                result.left_boundary[-result.virtual_pair_count :]
            )
            virtual_right.points = self._points(
                result.right_boundary[-result.virtual_pair_count :]
            )
        markers.markers.extend((virtual_left, virtual_right))

        label = self._base_marker(header, 7, Marker.TEXT_VIEW_FACING, "status")
        label.pose.position.x = 0.25
        label.pose.position.y = 0.0
        label.pose.position.z = 0.35
        label.scale.z = 0.16
        label.color = self._color(
            0.2 if result.valid else 1.0,
            1.0 if result.valid else 0.25,
            0.2,
        )
        label.text = (
            f"{result.status}  pairs={result.matched_pair_count}  "
            f"virtual={result.virtual_pair_count}  conf={result.confidence:.2f}"
        )
        markers.markers.append(label)
        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeLinePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble may report SIGTERM as an invalid-context RCLError after its
        # signal handler has already shut the context down.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
