"""Fail-closed ROS 2 controller for cone-planner local paths."""

from dataclasses import dataclass
from math import hypot, isfinite, tan
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool

from .pure_pursuit_core import (
    ControllerConfig,
    ControlResult,
    compute_pure_pursuit,
)

try:
    from ackermann_msgs.msg import AckermannDriveStamped

    ACKERMANN_MSGS_AVAILABLE = True
except ImportError:  # rosdep installs this for normal deployments.
    AckermannDriveStamped = None
    ACKERMANN_MSGS_AVAILABLE = False


@dataclass(frozen=True)
class _PathRecord:
    stamp_ns: int
    points: np.ndarray
    received_monotonic: float


@dataclass(frozen=True)
class _StatusRecord:
    stamp_ns: int
    confidence: float
    virtual_path: bool
    received_monotonic: float


@dataclass(frozen=True)
class _ActivePair:
    path: _PathRecord
    status: _StatusRecord
    pair_stamp_ns: int


class ConePurePursuit(Node):
    """Pair stamped planner outputs and publish bounded Ackermann commands."""

    def __init__(self) -> None:
        super().__init__("cone_pure_pursuit")
        read_only = ParameterDescriptor(read_only=True)

        self.declare_parameter(
            "path_topic", "cone_planner/center_path", read_only
        )
        self.declare_parameter("planner_status_topic", "cone_planner/status", read_only)
        self.declare_parameter("command_topic", "cone_controller/command", read_only)
        self.declare_parameter(
            "compat_command_topic", "cone_controller/command_vector", read_only
        )
        self.declare_parameter("allow_compat_command", False, read_only)
        self.declare_parameter("controller_status_topic", "cone_controller/status", read_only)
        self.declare_parameter("odom_topic", "odom", read_only)
        self.declare_parameter("planning_frame", "base_link", read_only)

        self.declare_parameter("control_frequency_hz", 50.0, read_only)
        self.declare_parameter("path_timeout_s", 0.25, read_only)
        self.declare_parameter("status_timeout_s", 0.25, read_only)
        self.declare_parameter("receipt_timeout_s", 0.30, read_only)
        self.declare_parameter("max_stamp_skew_s", 0.005, read_only)
        self.declare_parameter("max_future_stamp_s", 0.05, read_only)
        self.declare_parameter("max_uncompensated_motion_m", 0.025, read_only)
        self.declare_parameter("minimum_stop_hold_s", 0.50, read_only)
        self.declare_parameter("max_path_points", 500, read_only)
        self.declare_parameter("valid_frames_before_motion", 3, read_only)

        self.declare_parameter("require_odometry", False, read_only)
        self.declare_parameter("odom_timeout_s", 0.25, read_only)
        self.declare_parameter("odom_child_frame", "base_link", read_only)
        self.declare_parameter("geometry_confirmed", False, read_only)
        self.declare_parameter("enabled_on_startup", False, read_only)
        self.declare_parameter("active_mission_state", "CONE_SLALOM", read_only)
        self.declare_parameter("enforce_mission_state", False, read_only)
        self.declare_parameter("planner_max_curvature_1pm", 3.5, read_only)

        defaults = ControllerConfig()
        self._controller_config_names = frozenset(vars(defaults))
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value, read_only)
        self.config = ControllerConfig(
            **{
                name: self.get_parameter(name).value
                for name in self._controller_config_names
            }
        )

        self.path_topic = str(self.get_parameter("path_topic").value)
        self.planner_status_topic = str(
            self.get_parameter("planner_status_topic").value
        )
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.compat_command_topic = str(
            self.get_parameter("compat_command_topic").value
        )
        self.allow_compat_command = bool(
            self.get_parameter("allow_compat_command").value
        )
        self.controller_status_topic = str(
            self.get_parameter("controller_status_topic").value
        )
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_child_frame = str(
            self.get_parameter("odom_child_frame").value
        )
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        if not self.planning_frame:
            raise ValueError("planning_frame cannot be empty")

        self.control_frequency_hz = self._positive_float("control_frequency_hz")
        self.path_timeout_s = self._positive_float("path_timeout_s")
        self.status_timeout_s = self._positive_float("status_timeout_s")
        self.receipt_timeout_s = self._positive_float("receipt_timeout_s")
        self.max_stamp_skew_s = self._nonnegative_float("max_stamp_skew_s")
        self.max_future_stamp_s = self._nonnegative_float("max_future_stamp_s")
        self.max_uncompensated_motion_m = self._positive_float(
            "max_uncompensated_motion_m"
        )
        self.minimum_stop_hold_s = self._nonnegative_float(
            "minimum_stop_hold_s"
        )
        self.odom_timeout_s = self._positive_float("odom_timeout_s")
        self.planner_max_curvature_1pm = self._positive_float(
            "planner_max_curvature_1pm"
        )
        self.max_path_points = self._positive_int("max_path_points")
        self.valid_frames_before_motion = self._positive_int(
            "valid_frames_before_motion"
        )
        self.require_odometry = bool(self.get_parameter("require_odometry").value)
        self.geometry_confirmed = bool(
            self.get_parameter("geometry_confirmed").value
        )
        enabled_on_startup = bool(
            self.get_parameter("enabled_on_startup").value
        )
        self.active_mission_state = str(
            self.get_parameter("active_mission_state").value
        ).strip().upper()
        self.enforce_mission_state = bool(
            self.get_parameter("enforce_mission_state").value)
        self._mission_state = "SAFE_STOP"

        reachable_curvature = tan(self.config.max_steering_angle_rad) / (
            self.config.wheelbase_m
        )
        if (
            self.geometry_confirmed
            and self.planner_max_curvature_1pm > reachable_curvature + 1.0e-6
        ):
            raise ValueError(
                "planner_max_curvature_1pm exceeds measured steering geometry"
            )
        command_transport_ready = (
            ACKERMANN_MSGS_AVAILABLE or self.allow_compat_command
        )
        self._enabled = (
            enabled_on_startup
            and self.geometry_confirmed
            and command_transport_ready
        )
        if enabled_on_startup and not self.geometry_confirmed:
            self.get_logger().error(
                "enabled_on_startup was requested but geometry_confirmed is false; "
                "controller remains disabled"
            )
        if enabled_on_startup and not command_transport_ready:
            self.get_logger().error(
                "enabled_on_startup was requested but ackermann_msgs is missing "
                "and allow_compat_command is false; controller remains disabled"
            )

        reliable_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_publisher = (
            self.create_publisher(
                AckermannDriveStamped, self.command_topic, reliable_latest
            )
            if ACKERMANN_MSGS_AVAILABLE
            else None
        )
        self.compat_command_publisher = self.create_publisher(
            Vector3Stamped, self.compat_command_topic, reliable_latest
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.controller_status_topic, reliable_latest
        )
        self.path_subscription = self.create_subscription(
            Path, self.path_topic, self._path_callback, reliable_latest
        )
        self.status_subscription = self.create_subscription(
            DiagnosticArray,
            self.planner_status_topic,
            self._planner_status_callback,
            reliable_latest,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, sensor_latest
        )
        mission_qos = QoSProfile(depth=1)
        mission_qos.reliability = ReliabilityPolicy.RELIABLE
        mission_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.mission_subscription = self.create_subscription(
            String, "/mission/state", self._mission_state_callback, mission_qos
        )
        self.enable_service = self.create_service(
            SetBool, "cone_controller/set_enabled", self._set_enabled_callback
        )

        self._pending_paths: dict[int, _PathRecord] = {}
        self._pending_statuses: dict[int, _StatusRecord] = {}
        self._active_pair: _ActivePair | None = None
        self._highest_seen_stamp_ns = 0
        self._invalid_watermark_ns = 0
        self._last_recovery_stamp_ns = 0
        self._last_valid_pair_monotonic: float | None = None
        self._valid_frame_count = 0
        self._last_input_reason = "STARTUP"
        self._last_ros_now_ns = self.get_clock().now().nanoseconds

        self._odom_speed_mps = 0.0
        self._last_odom_monotonic: float | None = None
        self._last_odom_stamp_ns: int | None = None
        self._last_command_speed_mps = 0.0
        self._last_command_steering_rad = 0.0
        self._last_control_monotonic = time.monotonic()
        self._stop_hold_until_monotonic = (
            self._last_control_monotonic + self.minimum_stop_hold_s
        )
        self._last_log_monotonic = 0.0

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency_hz,
            self._control_callback,
            clock=steady_clock,
        )
        self._publish_stop("STARTUP_DISABLED")
        if not ACKERMANN_MSGS_AVAILABLE:
            self.get_logger().warn(
                "ackermann_msgs is not installed; arming is refused unless "
                "allow_compat_command was explicitly enabled"
            )
        self.get_logger().info(
            "Cone controller ready but %s: command=%s, frame=%s, geometry=%s"
            % (
                "enabled" if self._enabled else "disabled",
                self.command_topic,
                self.planning_frame,
                "confirmed" if self.geometry_confirmed else "UNCONFIRMED",
            )
        )

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and > 0")
        return value

    def _nonnegative_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and >= 0")
        return value

    def _positive_int(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or int(value) != value or int(value) <= 0:
            raise ValueError(f"{name} must be an integer > 0")
        return int(value)

    @staticmethod
    def _stamp_ns(header: Header) -> int:
        return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)

    def _validate_stamp(self, stamp_ns: int, max_age_s: float) -> tuple[bool, str, float]:
        if stamp_ns <= 0:
            return False, "ZERO_STAMP", float("nan")
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) * 1.0e-9
        if age_s > max_age_s:
            return False, "STALE_STAMP", age_s
        if age_s < -self.max_future_stamp_s:
            return False, "FUTURE_STAMP", age_s
        return True, "OK", age_s

    def _check_ros_clock_epoch(self) -> None:
        """Fail closed and reset stamp watermarks after a ROS clock rewind."""

        now_ns = self.get_clock().now().nanoseconds
        if now_ns + 1_000_000 < self._last_ros_now_ns:
            self._pending_paths.clear()
            self._pending_statuses.clear()
            self._active_pair = None
            self._highest_seen_stamp_ns = 0
            self._invalid_watermark_ns = 0
            self._last_recovery_stamp_ns = 0
            self._last_valid_pair_monotonic = None
            self._valid_frame_count = 0
            self._last_odom_stamp_ns = None
            self._last_odom_monotonic = None
            self._last_input_reason = "ROS_CLOCK_REWIND"
            self._stop_hold_until_monotonic = (
                time.monotonic() + self.minimum_stop_hold_s
            )
            self._publish_stop("ROS_CLOCK_REWIND")
        self._last_ros_now_ns = now_ns

    def _path_callback(self, message: Path) -> None:
        if (
            self.enforce_mission_state
            and self._mission_state != self.active_mission_state
        ):
            return
        self._check_ros_clock_epoch()
        now_monotonic = time.monotonic()
        stamp_ns = self._stamp_ns(message.header)
        valid_stamp, stamp_reason, _ = self._validate_stamp(
            stamp_ns, self.path_timeout_s
        )
        if not valid_stamp:
            self._invalidate(f"PATH_{stamp_reason}")
            return
        self._highest_seen_stamp_ns = max(self._highest_seen_stamp_ns, stamp_ns)
        if message.header.frame_id != self.planning_frame:
            self._invalidate("PATH_FRAME_MISMATCH", stamp_ns)
            return
        if not 2 <= len(message.poses) <= self.max_path_points:
            self._invalidate("EMPTY_OR_BAD_PATH", stamp_ns)
            return

        points: list[tuple[float, float]] = []
        for pose in message.poses:
            x = float(pose.pose.position.x)
            y = float(pose.pose.position.y)
            if not isfinite(x) or not isfinite(y):
                self._invalidate("NONFINITE_PATH", stamp_ns)
                return
            if pose.header.frame_id and pose.header.frame_id != self.planning_frame:
                self._invalidate("POSE_FRAME_MISMATCH", stamp_ns)
                return
            pose_stamp_ns = self._stamp_ns(pose.header)
            if (
                pose_stamp_ns > 0
                and abs(pose_stamp_ns - stamp_ns)
                > int(self.max_stamp_skew_s * 1.0e9)
            ):
                self._invalidate("POSE_STAMP_MISMATCH", stamp_ns)
                return
            points.append((x, y))

        if stamp_ns <= self._invalid_watermark_ns:
            return
        self._pending_paths[stamp_ns] = _PathRecord(
            stamp_ns,
            np.asarray(points, dtype=float),
            now_monotonic,
        )
        self._prune_pending()
        self._try_activate_pair()

    def _planner_status_callback(self, message: DiagnosticArray) -> None:
        if (
            self.enforce_mission_state
            and self._mission_state != self.active_mission_state
        ):
            return
        self._check_ros_clock_epoch()
        now_monotonic = time.monotonic()
        stamp_ns = self._stamp_ns(message.header)
        valid_stamp, stamp_reason, _ = self._validate_stamp(
            stamp_ns, self.status_timeout_s
        )
        if not valid_stamp:
            self._invalidate(f"STATUS_{stamp_reason}")
            return
        self._highest_seen_stamp_ns = max(self._highest_seen_stamp_ns, stamp_ns)
        if message.header.frame_id != self.planning_frame:
            self._invalidate("STATUS_FRAME_MISMATCH", stamp_ns)
            return

        selected: DiagnosticStatus | None = None
        selected_values: dict[str, str] = {}
        for item in message.status:
            values = {entry.key: entry.value for entry in item.values}
            if "status" in values:
                selected = item
                selected_values = values
                break
        if selected is None:
            self._invalidate("MISSING_PLANNER_STATUS", stamp_ns)
            return
        planner_status = selected_values.get("status", "")
        if selected.level != DiagnosticStatus.OK or planner_status not in {
            "OK",
            "OK_VIRTUAL",
        }:
            self._invalidate(f"PLANNER_{planner_status or 'INVALID'}", stamp_ns)
            return
        try:
            confidence = float(selected_values["confidence"])
        except (KeyError, TypeError, ValueError):
            self._invalidate("MISSING_CONFIDENCE", stamp_ns)
            return
        if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            self._invalidate("BAD_CONFIDENCE", stamp_ns)
            return
        if confidence < self.config.min_plan_confidence:
            self._invalidate("LOW_CONFIDENCE", stamp_ns)
            return

        if stamp_ns <= self._invalid_watermark_ns:
            return
        self._pending_statuses[stamp_ns] = _StatusRecord(
            stamp_ns,
            confidence,
            planner_status == "OK_VIRTUAL",
            now_monotonic,
        )
        self._prune_pending()
        self._try_activate_pair()

    def _odom_callback(self, message: Odometry) -> None:
        self._check_ros_clock_epoch()
        stamp_ns = self._stamp_ns(message.header)
        valid_stamp, stamp_reason, _ = self._validate_stamp(
            stamp_ns, self.odom_timeout_s
        )
        if not valid_stamp:
            self._last_odom_monotonic = None
            self._last_odom_stamp_ns = None
            if self.require_odometry:
                self._invalidate(f"ODOM_{stamp_reason}")
            return
        if (
            self._last_odom_stamp_ns is not None
            and stamp_ns <= self._last_odom_stamp_ns
        ):
            self._last_odom_monotonic = None
            if self.require_odometry:
                self._invalidate("OUT_OF_ORDER_ODOMETRY")
            return
        if (
            self.odom_child_frame
            and message.child_frame_id != self.odom_child_frame
        ):
            self._last_odom_monotonic = None
            if self.require_odometry:
                self._invalidate("ODOMETRY_FRAME_MISMATCH")
            return
        linear = message.twist.twist.linear
        speed = hypot(float(linear.x), float(linear.y))
        if not isfinite(speed):
            self._last_odom_monotonic = None
            self._publish_stop("BAD_ODOMETRY")
            return
        self._odom_speed_mps = speed
        self._last_odom_monotonic = time.monotonic()
        self._last_odom_stamp_ns = stamp_ns

    def _mission_state_callback(self, message: String) -> None:
        state = str(message.data).strip().upper()
        if state == self._mission_state:
            return
        self._mission_state = state
        if not self.enforce_mission_state:
            return
        # Never carry a CONE_INIT path or steering command into CONE_SLALOM.
        self._invalidate("MISSION_STATE_CHANGED")

    def _prune_pending(self) -> None:
        for values in (self._pending_paths, self._pending_statuses):
            for stamp in sorted(values)[:-8]:
                del values[stamp]
            for stamp in list(values):
                if stamp <= self._invalid_watermark_ns:
                    del values[stamp]

    def _try_activate_pair(self) -> None:
        choices: list[tuple[int, _PathRecord, _StatusRecord]] = []
        for path in self._pending_paths.values():
            for status in self._pending_statuses.values():
                if path.stamp_ns == status.stamp_ns:
                    pair_stamp = max(path.stamp_ns, status.stamp_ns)
                    if (
                        pair_stamp > self._invalid_watermark_ns
                        and pair_stamp > self._last_recovery_stamp_ns
                        and pair_stamp == self._highest_seen_stamp_ns
                    ):
                        choices.append((pair_stamp, path, status))
        if not choices:
            return
        pair_stamp, path, status = max(choices, key=lambda value: value[0])
        now_monotonic = time.monotonic()
        if (
            self._last_valid_pair_monotonic is None
            or now_monotonic - self._last_valid_pair_monotonic
            > self.receipt_timeout_s
        ):
            self._valid_frame_count = 0
        self._last_valid_pair_monotonic = now_monotonic
        self._last_recovery_stamp_ns = pair_stamp
        self._valid_frame_count += 1
        self._last_input_reason = "VALIDATING"
        if self._valid_frame_count >= self.valid_frames_before_motion:
            self._active_pair = _ActivePair(path, status, pair_stamp)
            self._last_input_reason = "OK"
        self._pending_paths = {
            stamp: value
            for stamp, value in self._pending_paths.items()
            if stamp > pair_stamp
        }
        self._pending_statuses = {
            stamp: value
            for stamp, value in self._pending_statuses.items()
            if stamp > pair_stamp
        }

    def _invalidate(self, reason: str, stamp_ns: int = 0) -> None:
        self._invalid_watermark_ns = max(
            self._invalid_watermark_ns,
            self._highest_seen_stamp_ns,
            max(0, stamp_ns),
        )
        self._pending_paths.clear()
        self._pending_statuses.clear()
        self._active_pair = None
        self._valid_frame_count = 0
        self._last_valid_pair_monotonic = None
        self._last_input_reason = reason
        self._stop_hold_until_monotonic = max(
            self._stop_hold_until_monotonic,
            time.monotonic() + self.minimum_stop_hold_s,
        )
        self._publish_stop(reason)

    def _set_enabled_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if request.data and not self.geometry_confirmed:
            self._enabled = False
            response.success = False
            response.message = (
                "Refused: set measured wheelbase/steering limits and "
                "geometry_confirmed:=true in the controller config"
            )
            self._publish_stop("GEOMETRY_UNCONFIRMED")
            return response
        if (
            request.data
            and not ACKERMANN_MSGS_AVAILABLE
            and not self.allow_compat_command
        ):
            self._enabled = False
            response.success = False
            response.message = (
                "Refused: install ackermann_msgs, or explicitly configure a "
                "custom Vector3 bridge and allow_compat_command:=true"
            )
            self._publish_stop("ACKERMANN_MSGS_MISSING")
            return response

        self._enabled = bool(request.data)
        self._active_pair = None
        self._pending_paths.clear()
        self._pending_statuses.clear()
        self._valid_frame_count = 0
        self._last_valid_pair_monotonic = None
        self._last_recovery_stamp_ns = max(
            self._last_recovery_stamp_ns, self._highest_seen_stamp_ns
        )
        self._last_input_reason = "WAITING_FOR_VALID_FRAMES"
        self._stop_hold_until_monotonic = (
            time.monotonic() + self.minimum_stop_hold_s
        )
        self._publish_stop("ENABLED_WAITING" if self._enabled else "DISABLED")
        response.success = True
        response.message = (
            "Enabled; waiting for new consecutive valid planner frames"
            if self._enabled
            else "Disabled and stopped"
        )
        return response

    def _control_callback(self) -> None:
        self._check_ros_clock_epoch()
        now_monotonic = time.monotonic()
        dt_s = float(
            np.clip(now_monotonic - self._last_control_monotonic, 0.001, 0.10)
        )
        self._last_control_monotonic = now_monotonic

        if (
            self.enforce_mission_state
            and self._mission_state != self.active_mission_state
        ):
            self._publish_stop("MISSION_STATE_INACTIVE")
            return

        if not self._enabled:
            self._publish_stop("DISABLED")
            return
        if not self.geometry_confirmed:
            self._publish_stop("GEOMETRY_UNCONFIRMED")
            return
        if self._active_pair is None:
            self._publish_stop(
                "WAITING_FOR_VALID_FRAMES"
                if self._valid_frame_count
                else self._last_input_reason
            )
            return
        if now_monotonic < self._stop_hold_until_monotonic:
            self._publish_stop("STOP_HOLD")
            return

        active = self._active_pair
        path_receipt_age = now_monotonic - active.path.received_monotonic
        status_receipt_age = now_monotonic - active.status.received_monotonic
        if (
            path_receipt_age > self.receipt_timeout_s
            or status_receipt_age > self.receipt_timeout_s
        ):
            self._invalidate("INPUT_RECEIPT_TIMEOUT", active.pair_stamp_ns)
            return

        valid_path_stamp, path_reason, path_age_s = self._validate_stamp(
            active.path.stamp_ns, self.path_timeout_s
        )
        valid_status_stamp, status_reason, _ = self._validate_stamp(
            active.status.stamp_ns, self.status_timeout_s
        )
        if not valid_path_stamp:
            self._invalidate(f"PATH_{path_reason}", active.pair_stamp_ns)
            return
        if not valid_status_stamp:
            self._invalidate(f"STATUS_{status_reason}", active.pair_stamp_ns)
            return

        if self.require_odometry:
            if self._last_odom_monotonic is None:
                self._invalidate("NO_ODOMETRY", active.pair_stamp_ns)
                return
            if now_monotonic - self._last_odom_monotonic > self.odom_timeout_s:
                self._invalidate("ODOMETRY_TIMEOUT", active.pair_stamp_ns)
                return
            current_speed = self._odom_speed_mps
        else:
            current_speed = self._last_command_speed_mps

        motion_bound_speed = (
            current_speed if self.require_odometry else self.config.max_speed_mps
        )
        uncompensated_motion = motion_bound_speed * max(0.0, path_age_s)
        if uncompensated_motion > self.max_uncompensated_motion_m:
            self._invalidate("UNCOMPENSATED_PATH_MOTION", active.pair_stamp_ns)
            return

        result = compute_pure_pursuit(
            active.path.points,
            current_speed_mps=current_speed,
            plan_confidence=active.status.confidence,
            previous_speed_mps=self._last_command_speed_mps,
            previous_steering_angle_rad=self._last_command_steering_rad,
            dt_s=dt_s,
            config=self.config,
            virtual_path=active.status.virtual_path,
        )
        if not result.valid:
            self._invalidate(result.reason, active.pair_stamp_ns)
            return
        self._publish_control(result, path_age_s, uncompensated_motion, dt_s)

    def _command_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.planning_frame
        return header

    def _publish_control(
        self,
        result: ControlResult,
        path_age_s: float,
        uncompensated_motion_m: float,
        dt_s: float,
    ) -> None:
        previous_speed = self._last_command_speed_mps
        previous_steering = self._last_command_steering_rad
        header = self._command_header()
        acceleration = float(
            (result.speed_mps - previous_speed) / max(dt_s, 1.0e-6)
        )
        steering_velocity = float(
            (result.steering_angle_rad - previous_steering) / max(dt_s, 1.0e-6)
        )
        if self.command_publisher is not None:
            command = AckermannDriveStamped()
            command.header = header
            command.drive.speed = float(result.speed_mps)
            command.drive.steering_angle = float(result.steering_angle_rad)
            command.drive.acceleration = acceleration
            command.drive.steering_angle_velocity = steering_velocity
            self.command_publisher.publish(command)
        if self.allow_compat_command:
            compatibility = Vector3Stamped()
            compatibility.header = header
            # Explicit custom bridge contract: x=speed [m/s], y=road-wheel
            # steering angle [rad], z is reserved and remains zero.
            compatibility.vector.x = float(result.speed_mps)
            compatibility.vector.y = float(result.steering_angle_rad)
            self.compat_command_publisher.publish(compatibility)
        self._last_command_speed_mps = result.speed_mps
        self._last_command_steering_rad = result.steering_angle_rad
        self._publish_controller_status(
            DiagnosticStatus.OK,
            result.reason,
            {
                "enabled": self._enabled,
                "speed_mps": result.speed_mps,
                "steering_angle_rad": result.steering_angle_rad,
                "raw_steering_angle_rad": result.raw_steering_angle_rad,
                "curvature_1pm": result.curvature_1pm,
                "lookahead_m": result.lookahead_m,
                "target_x_m": result.target_x_m,
                "target_y_m": result.target_y_m,
                "remaining_path_m": result.remaining_path_m,
                "path_age_s": path_age_s,
                "uncompensated_motion_m": uncompensated_motion_m,
                "plan_confidence": (
                    self._active_pair.status.confidence
                    if self._active_pair is not None
                    else float("nan")
                ),
            },
        )

    def _publish_stop(self, reason: str) -> None:
        header = self._command_header()
        if (
            hasattr(self, "command_publisher")
            and self.command_publisher is not None
        ):
            command = AckermannDriveStamped()
            command.header = header
            command.drive.speed = 0.0
            command.drive.steering_angle = 0.0
            command.drive.acceleration = 0.0
            command.drive.steering_angle_velocity = 0.0
            self.command_publisher.publish(command)
        if hasattr(self, "compat_command_publisher"):
            compatibility = Vector3Stamped()
            compatibility.header = header
            compatibility.vector.x = 0.0
            compatibility.vector.y = 0.0
            self.compat_command_publisher.publish(compatibility)
        self._last_command_speed_mps = 0.0
        self._last_command_steering_rad = 0.0
        if hasattr(self, "status_publisher"):
            level = (
                DiagnosticStatus.WARN
                if reason
                in {
                    "DISABLED",
                    "STARTUP_DISABLED",
                    "ENABLED_WAITING",
                    "WAITING_FOR_VALID_FRAMES",
                    "VALIDATING",
                    "STOP_HOLD",
                }
                else DiagnosticStatus.ERROR
            )
            self._publish_controller_status(
                level,
                reason,
                {
                    "enabled": self._enabled,
                    "speed_mps": 0.0,
                    "steering_angle_rad": 0.0,
                    "valid_frames": self._valid_frame_count,
                    "ackermann_msgs_available": ACKERMANN_MSGS_AVAILABLE,
                    "allow_compat_command": self.allow_compat_command,
                },
            )
        now = time.monotonic()
        if hasattr(self, "_last_log_monotonic") and now - self._last_log_monotonic >= 2.0:
            self._last_log_monotonic = now
            self.get_logger().warn(f"Controller stop: {reason}")

    def _publish_controller_status(
        self, level: int, reason: str, values: dict[str, object]
    ) -> None:
        message = DiagnosticArray()
        message.header = self._command_header()
        status = DiagnosticStatus()
        status.level = level
        status.name = f"{self.get_fully_qualified_name()}: cone controller"
        status.hardware_id = "ackermann_vehicle"
        status.message = reason
        status.values.append(KeyValue(key="status", value=reason))
        for key, value in values.items():
            if isinstance(value, float):
                text = f"{value:.6g}" if isfinite(value) else "nan"
            else:
                text = str(value)
            status.values.append(KeyValue(key=key, value=text))
        message.status.append(status)
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConePurePursuit()
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
        try:
            if rclpy.ok():
                node._publish_stop("SHUTDOWN")
        except Exception:
            # The context can become invalid between the ok() check and DDS
            # publish. Preserve genuine errors while suppressing that race.
            if rclpy.ok():
                raise
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
