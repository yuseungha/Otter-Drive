"""ROS 2 closed-loop synthetic cone world for the LiDAR planner.

The node deliberately uses wall/ROS time rather than publishing ``/clock``.
Physics timers use a steady clock so pausing or rewinding ROS time cannot make
the vehicle integrate a large time step.  The generated world is deterministic
for a given scan index and random seed; only message delivery timing is live.
"""

from math import atan2, cos, isfinite, sin
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    Point,
    PoseStamped,
    TransformStamped,
    Vector3Stamped,
)
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
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .synthetic_world_core import (
    Pose2D,
    SyntheticWorld,
    VehicleState,
    WorldConfig,
    bicycle_yaw_rate,
    make_arc_course,
    make_s_course,
    make_straight_course,
    obstacle_clearance_m,
    project_to_polyline,
    step_bicycle,
)


class SyntheticConeWorld(Node):
    """Publish simulated LiDAR, vehicle state, truth, and safety metrics."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "synthetic_cone_world",
            parameter_overrides=parameter_overrides,
        )
        read_only = ParameterDescriptor(read_only=True)

        self._declare("scan_topic", "scan", read_only)
        self._declare("odom_topic", "odom", read_only)
        self._declare(
            "command_topic", "cone_controller/command_vector", read_only
        )
        self._declare(
            "ground_truth_path_topic",
            "synthetic_world/ground_truth_path",
            read_only,
        )
        self._declare("marker_topic", "synthetic_world/markers", read_only)
        self._declare("status_topic", "synthetic_world/status", read_only)
        self._declare(
            "controller_enable_service",
            "cone_controller/set_enabled",
            read_only,
        )

        self._declare("odom_frame", "sim_odom", read_only)
        self._declare("base_frame", "sim_base_link", read_only)
        self._declare("laser_frame", "sim_laser", read_only)

        self._declare("scenario", "straight", read_only)
        self._declare("course_length_m", 4.5, read_only)
        self._declare("track_width_m", 0.60, read_only)
        self._declare("cone_spacing_m", 0.297, read_only)
        self._declare("cone_radius_m", 0.025, read_only)
        self._declare("first_cone_distance_m", 0.30, read_only)
        self._declare("arc_radius_m", 3.0, read_only)
        self._declare("s_bend_amplitude_m", 0.35, read_only)

        self._declare("physics_frequency_hz", 50.0, read_only)
        self._declare("scan_frequency_hz", 10.0, read_only)
        self._declare("metrics_frequency_hz", 10.0, read_only)
        self._declare("wheelbase_m", 0.20, read_only)
        self._declare("vehicle_footprint_radius_m", 0.25, read_only)
        self._declare("initial_x_m", 0.0, read_only)
        self._declare("initial_y_m", 0.0, read_only)
        self._declare("initial_yaw_rad", 0.0, read_only)

        self._declare("max_command_speed_mps", 0.30, read_only)
        self._declare("max_command_steering_rad", 0.70, read_only)
        self._declare("max_command_age_s", 0.25, read_only)
        self._declare("max_future_command_s", 0.05, read_only)
        self._declare("command_receipt_timeout_s", 0.30, read_only)
        self._declare("command_zero_tolerance", 1.0e-9, read_only)
        self._declare("max_accel_mps2", 0.30, read_only)
        self._declare("max_decel_mps2", 0.80, read_only)
        self._declare("max_steering_rate_rad_s", 2.0, read_only)

        self._declare("angle_min_rad", -np.pi, read_only)
        self._declare("angle_max_rad", np.pi, read_only)
        self._declare("beam_count", 1081, read_only)
        self._declare("range_min_m", 0.15, read_only)
        self._declare("range_max_m", 5.0, read_only)
        self._declare("laser_x_m", 0.0, read_only)
        self._declare("laser_y_m", 0.0, read_only)
        self._declare("laser_yaw_rad", 0.0, read_only)
        self._declare("random_seed", 7, read_only)
        self._declare("gaussian_noise_std_m", 0.002, read_only)
        self._declare("range_quantization_m", 0.001, read_only)
        self._declare("beam_dropout_probability", 0.0, read_only)
        self._declare("same_range_replica_probability", 0.0, read_only)
        self._declare("same_range_replica_span_beams", 0, read_only)
        self._declare("glint_probability", 0.0, read_only)
        self._declare("glint_range_min_m", 0.15, read_only)
        self._declare("glint_range_max_m", 2.0, read_only)

        self._declare("drop_scan_after_s", -1.0, read_only)
        self._declare("drop_scan_duration_s", 0.0, read_only)
        self._declare("auto_enable_controller", False, read_only)
        self._declare("auto_enable_delay_s", 1.0, read_only)

        self.scan_topic = self._text("scan_topic")
        self.odom_topic = self._text("odom_topic")
        self.command_topic = self._text("command_topic")
        self.ground_truth_path_topic = self._text("ground_truth_path_topic")
        self.marker_topic = self._text("marker_topic")
        self.status_topic = self._text("status_topic")
        self.controller_enable_service = self._text(
            "controller_enable_service"
        )
        self.odom_frame = self._text("odom_frame")
        self.base_frame = self._text("base_frame")
        self.laser_frame = self._text("laser_frame")
        if len({self.odom_frame, self.base_frame, self.laser_frame}) != 3:
            raise ValueError("odom, base, and laser frames must be distinct")

        self.physics_frequency_hz = self._positive("physics_frequency_hz")
        self.scan_frequency_hz = self._positive("scan_frequency_hz")
        self.metrics_frequency_hz = self._positive("metrics_frequency_hz")
        self.wheelbase_m = self._positive("wheelbase_m")
        self.vehicle_footprint_radius_m = self._nonnegative(
            "vehicle_footprint_radius_m"
        )
        self.max_command_speed_mps = self._positive(
            "max_command_speed_mps"
        )
        self.max_command_steering_rad = self._positive(
            "max_command_steering_rad"
        )
        if self.max_command_steering_rad >= 0.5 * np.pi:
            raise ValueError("max_command_steering_rad must be < pi/2")
        self.max_command_age_s = self._positive("max_command_age_s")
        self.max_future_command_s = self._nonnegative(
            "max_future_command_s"
        )
        self.command_receipt_timeout_s = self._positive(
            "command_receipt_timeout_s"
        )
        self.command_zero_tolerance = self._nonnegative(
            "command_zero_tolerance"
        )
        self.max_accel_mps2 = self._positive("max_accel_mps2")
        self.max_decel_mps2 = self._positive("max_decel_mps2")
        self.max_steering_rate_rad_s = self._positive(
            "max_steering_rate_rad_s"
        )
        self.drop_scan_after_s = self._finite("drop_scan_after_s")
        self.drop_scan_duration_s = self._nonnegative(
            "drop_scan_duration_s"
        )
        self.auto_enable_controller = bool(
            self.get_parameter("auto_enable_controller").value
        )
        self.auto_enable_delay_s = self._nonnegative(
            "auto_enable_delay_s"
        )

        sensor_pose = Pose2D(
            self._finite("laser_x_m"),
            self._finite("laser_y_m"),
            self._finite("laser_yaw_rad"),
        )
        world_config = WorldConfig(
            angle_min_rad=self._finite("angle_min_rad"),
            angle_max_rad=self._finite("angle_max_rad"),
            beam_count=self._integer("beam_count", minimum=2),
            range_min_m=self._nonnegative("range_min_m"),
            range_max_m=self._positive("range_max_m"),
            sensor_pose_in_vehicle=sensor_pose,
            random_seed=self._integer("random_seed", minimum=0),
            gaussian_noise_std_m=self._nonnegative(
                "gaussian_noise_std_m"
            ),
            range_quantization_m=self._nonnegative(
                "range_quantization_m"
            ),
            beam_dropout_probability=self._probability(
                "beam_dropout_probability"
            ),
            same_range_replica_probability=self._probability(
                "same_range_replica_probability"
            ),
            same_range_replica_span_beams=self._integer(
                "same_range_replica_span_beams", minimum=0
            ),
            glint_probability=self._probability("glint_probability"),
            glint_range_min_m=self._nonnegative("glint_range_min_m"),
            glint_range_max_m=self._positive("glint_range_max_m"),
        )
        self.course = self._make_course()
        self.world = SyntheticWorld(self.course.obstacles(), world_config)
        self._obstacles = self.course.obstacles()
        self.state = VehicleState(
            self._finite("initial_x_m"),
            self._finite("initial_y_m"),
            self._finite("initial_yaw_rad"),
            0.0,
        )

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        truth_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.scan_publisher = self.create_publisher(
            LaserScan, self.scan_topic, sensor_qos
        )
        self.odom_publisher = self.create_publisher(
            Odometry, self.odom_topic, sensor_qos
        )
        self.truth_path_publisher = self.create_publisher(
            Path, self.ground_truth_path_topic, truth_qos
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, self.marker_topic, truth_qos
        )
        self.status_publisher = self.create_publisher(
            DiagnosticArray, self.status_topic, reliable_qos
        )
        self.command_subscription = self.create_subscription(
            Vector3Stamped,
            self.command_topic,
            self._command_callback,
            reliable_qos,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self._started_monotonic = time.monotonic()
        self._last_stamp_ns = 0
        self._scan_index = 0
        self._scan_count = 0
        self._dropped_scan_count = 0
        self._received_command_count = 0
        self._rejected_command_count = 0
        self._last_command_stamp_ns = 0
        self._last_command_receipt_monotonic = None
        self._command_valid = False
        self._command_reason = "NO_COMMAND"
        self._target_speed_mps = 0.0
        self._target_steering_rad = 0.0
        self._applied_speed_mps = 0.0
        self._applied_steering_rad = 0.0
        self._collision = False
        self._collision_count = 0
        self._minimum_clearance_m = float("inf")
        self._maximum_abs_lateral_m = 0.0
        self._enable_future = None
        self._enable_succeeded = False
        self._last_enable_attempt_monotonic = float("-inf")

        self._publish_static_transform(sensor_pose)
        initial_stamp = self._next_stamp()
        self.truth_path_publisher.publish(
            self._make_truth_path(initial_stamp)
        )
        self.marker_publisher.publish(self._make_markers(initial_stamp))

        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.physics_timer = self.create_timer(
            1.0 / self.physics_frequency_hz,
            self._physics_callback,
            clock=steady_clock,
        )
        self.scan_timer = self.create_timer(
            1.0 / self.scan_frequency_hz,
            self._scan_callback,
            clock=steady_clock,
        )
        self.metrics_timer = self.create_timer(
            1.0 / self.metrics_frequency_hz,
            self._metrics_callback,
            clock=steady_clock,
        )
        self.enable_client = self.create_client(
            SetBool, self.controller_enable_service
        )
        self.enable_timer = self.create_timer(
            0.25,
            self._auto_enable_callback,
            clock=steady_clock,
        )

        self.get_logger().info(
            "Synthetic world ready: scenario=%s scan=%.1fHz physics=%.1fHz"
            % (
                self.course.name,
                self.scan_frequency_hz,
                self.physics_frequency_hz,
            )
        )

    def _declare(self, name: str, default, descriptor) -> None:
        self.declare_parameter(name, default, descriptor)

    def _text(self, name: str) -> str:
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError(f"{name} cannot be empty")
        return value

    def _finite(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not isfinite(value):
            raise ValueError(f"{name} must be finite")
        return value

    def _positive(self, name: str) -> float:
        value = self._finite(name)
        if value <= 0.0:
            raise ValueError(f"{name} must be > 0")
        return value

    def _nonnegative(self, name: str) -> float:
        value = self._finite(name)
        if value < 0.0:
            raise ValueError(f"{name} must be >= 0")
        return value

    def _integer(self, name: str, *, minimum: int) -> int:
        value = self.get_parameter(name).value
        if (
            isinstance(value, bool)
            or int(value) != value
            or int(value) < minimum
        ):
            raise ValueError(f"{name} must be an integer >= {minimum}")
        return int(value)

    def _probability(self, name: str) -> float:
        value = self._finite(name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be in [0, 1]")
        return value

    def _make_course(self):
        common = {
            "length_m": self._positive("course_length_m"),
            "track_width_m": self._positive("track_width_m"),
            "cone_spacing_m": self._positive("cone_spacing_m"),
            "cone_radius_m": self._positive("cone_radius_m"),
            "first_cone_distance_m": self._nonnegative(
                "first_cone_distance_m"
            ),
        }
        scenario = self._text("scenario")
        if scenario == "straight":
            return make_straight_course(**common)
        if scenario in {"left_arc", "right_arc"}:
            return make_arc_course(
                **common,
                radius_m=self._positive("arc_radius_m"),
                turn_left=scenario == "left_arc",
            )
        if scenario == "s_bend":
            return make_s_course(
                **common,
                amplitude_m=self._positive("s_bend_amplitude_m"),
            )
        raise ValueError(
            "scenario must be straight, left_arc, right_arc, or s_bend"
        )

    @staticmethod
    def _stamp_ns(header: Header) -> int:
        return int(header.stamp.sec) * 1_000_000_000 + int(
            header.stamp.nanosec
        )

    def _next_stamp(self):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = max(now_ns, self._last_stamp_ns + 1)
        self._last_stamp_ns = stamp_ns
        return Time(
            nanoseconds=stamp_ns,
            clock_type=self.get_clock().clock_type,
        ).to_msg()

    @staticmethod
    def _yaw_quaternion(yaw: float):
        from geometry_msgs.msg import Quaternion

        return Quaternion(z=sin(0.5 * yaw), w=cos(0.5 * yaw))

    def _header(self, stamp, frame_id: str) -> Header:
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    def _publish_static_transform(self, sensor_pose: Pose2D) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.laser_frame
        transform.transform.translation.x = sensor_pose.x
        transform.transform.translation.y = sensor_pose.y
        transform.transform.rotation = self._yaw_quaternion(sensor_pose.yaw)
        self.static_tf_broadcaster.sendTransform(transform)

    def _command_callback(self, message: Vector3Stamped) -> None:
        self._received_command_count += 1
        stamp_ns = self._stamp_ns(message.header)
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= 0:
            self._reject_command("ZERO_COMMAND_STAMP")
            return
        age_s = (now_ns - stamp_ns) * 1.0e-9
        if age_s > self.max_command_age_s:
            self._reject_command("STALE_COMMAND")
            return
        if age_s < -self.max_future_command_s:
            self._reject_command("FUTURE_COMMAND")
            return
        if stamp_ns <= self._last_command_stamp_ns:
            self._reject_command("OUT_OF_ORDER_COMMAND")
            return
        if message.header.frame_id != self.base_frame:
            self._reject_command("COMMAND_FRAME_MISMATCH")
            return

        speed = float(message.vector.x)
        steering = float(message.vector.y)
        reserved = float(message.vector.z)
        if not all(isfinite(value) for value in (speed, steering, reserved)):
            self._reject_command("NONFINITE_COMMAND")
            return
        if abs(reserved) > self.command_zero_tolerance:
            self._reject_command("COMMAND_RESERVED_NONZERO")
            return
        if not 0.0 <= speed <= self.max_command_speed_mps:
            self._reject_command("COMMAND_SPEED_BOUNDS")
            return
        if abs(steering) > self.max_command_steering_rad:
            self._reject_command("COMMAND_STEERING_BOUNDS")
            return

        self._last_command_stamp_ns = stamp_ns
        self._target_speed_mps = speed
        self._target_steering_rad = steering
        self._last_command_receipt_monotonic = time.monotonic()
        self._command_valid = True
        self._command_reason = "OK"

    def _reject_command(self, reason: str) -> None:
        self._rejected_command_count += 1
        self._target_speed_mps = 0.0
        self._target_steering_rad = 0.0
        self._command_valid = False
        self._command_reason = reason

    def _apply_command_timeout(self, now_monotonic: float) -> None:
        if self._last_command_receipt_monotonic is None:
            self._target_speed_mps = 0.0
            self._target_steering_rad = 0.0
            self._command_valid = False
            self._command_reason = "NO_COMMAND"
            return
        if (
            now_monotonic - self._last_command_receipt_monotonic
            > self.command_receipt_timeout_s
        ):
            self._target_speed_mps = 0.0
            self._target_steering_rad = 0.0
            self._command_valid = False
            self._command_reason = "COMMAND_RECEIPT_TIMEOUT"

    @staticmethod
    def _slew(
        current: float,
        target: float,
        positive_rate: float,
        negative_rate: float,
        dt_s: float,
    ) -> float:
        difference = target - current
        rate = positive_rate if difference >= 0.0 else negative_rate
        limit = rate * dt_s
        return current + float(np.clip(difference, -limit, limit))

    def _physics_callback(self) -> None:
        now_monotonic = time.monotonic()
        self._apply_command_timeout(now_monotonic)
        dt_s = 1.0 / self.physics_frequency_hz
        self._applied_speed_mps = self._slew(
            self._applied_speed_mps,
            self._target_speed_mps,
            self.max_accel_mps2,
            self.max_decel_mps2,
            dt_s,
        )
        steering_step = self.max_steering_rate_rad_s * dt_s
        self._applied_steering_rad += float(
            np.clip(
                self._target_steering_rad - self._applied_steering_rad,
                -steering_step,
                steering_step,
            )
        )
        self.state = step_bicycle(
            self.state,
            self._applied_speed_mps,
            self._applied_steering_rad,
            dt_s,
            self.wheelbase_m,
        )
        self._update_metrics()
        stamp = self._next_stamp()
        self._publish_dynamic_transform(stamp)
        self.odom_publisher.publish(self._make_odometry(stamp))

    def _scan_drop_active(self, now_monotonic: float) -> bool:
        if self.drop_scan_after_s < 0.0 or self.drop_scan_duration_s <= 0.0:
            return False
        elapsed = now_monotonic - self._started_monotonic
        return (
            self.drop_scan_after_s
            <= elapsed
            < self.drop_scan_after_s + self.drop_scan_duration_s
        )

    def _scan_callback(self) -> None:
        if self._scan_drop_active(time.monotonic()):
            self._dropped_scan_count += 1
            return
        frame = self.world.scan(self.state.pose, self._scan_index)
        self._scan_index += 1
        stamp = self._next_stamp()
        # A transform with exactly the scan stamp prevents extrapolation at the
        # newest TF boundary when the scan and physics timers interleave.
        self._publish_dynamic_transform(stamp)
        self.scan_publisher.publish(self._make_scan(frame, stamp))
        self._scan_count += 1

    def _metrics_callback(self) -> None:
        stamp = self._next_stamp()
        self.marker_publisher.publish(self._make_markers(stamp))
        self.status_publisher.publish(self._make_status(stamp))

    def _update_metrics(self) -> None:
        projection = project_to_polyline(
            (self.state.x, self.state.y), self.course.centerline
        )
        clearance = obstacle_clearance_m(
            (self.state.x, self.state.y),
            self._obstacles,
            self.vehicle_footprint_radius_m,
        )
        collision = clearance <= 0.0
        if collision and not self._collision:
            self._collision_count += 1
        self._collision = collision
        self._minimum_clearance_m = min(self._minimum_clearance_m, clearance)
        self._maximum_abs_lateral_m = max(
            self._maximum_abs_lateral_m, projection.absolute_lateral_m
        )

    def _publish_dynamic_transform(self, stamp) -> None:
        transform = TransformStamped()
        transform.header = self._header(stamp, self.odom_frame)
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.state.x
        transform.transform.translation.y = self.state.y
        transform.transform.rotation = self._yaw_quaternion(self.state.yaw)
        self.tf_broadcaster.sendTransform(transform)

    def _make_odometry(self, stamp) -> Odometry:
        message = Odometry()
        message.header = self._header(stamp, self.odom_frame)
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.state.x
        message.pose.pose.position.y = self.state.y
        message.pose.pose.orientation = self._yaw_quaternion(self.state.yaw)
        message.twist.twist.linear.x = self._applied_speed_mps
        message.twist.twist.angular.z = bicycle_yaw_rate(
            self._applied_speed_mps,
            self._applied_steering_rad,
            self.wheelbase_m,
        )
        message.pose.covariance[0] = 1.0e-8
        message.pose.covariance[7] = 1.0e-8
        message.pose.covariance[35] = 1.0e-8
        message.twist.covariance[0] = 1.0e-8
        message.twist.covariance[35] = 1.0e-8
        return message

    def _make_scan(self, frame, stamp) -> LaserScan:
        message = LaserScan()
        message.header = self._header(stamp, self.laser_frame)
        message.angle_min = float(frame.angle_min_rad)
        message.angle_max = float(frame.angle_max_rad)
        message.angle_increment = float(frame.angle_increment_rad)
        message.scan_time = float(1.0 / self.scan_frequency_hz)
        message.time_increment = float(
            message.scan_time / max(1, len(frame.ranges) - 1)
        )
        message.range_min = float(frame.range_min_m)
        message.range_max = float(frame.range_max_m)
        message.ranges = [float(value) for value in frame.ranges]
        return message

    def _make_truth_path(self, stamp) -> Path:
        message = Path()
        message.header = self._header(stamp, self.odom_frame)
        points = self.course.centerline
        for index, point in enumerate(points):
            if index == 0:
                direction = points[1] - points[0]
            elif index == len(points) - 1:
                direction = points[-1] - points[-2]
            else:
                direction = points[index + 1] - points[index - 1]
            yaw = atan2(float(direction[1]), float(direction[0]))
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.orientation = self._yaw_quaternion(yaw)
            message.poses.append(pose)
        return message

    @staticmethod
    def _color(red: float, green: float, blue: float,
               alpha: float = 1.0) -> ColorRGBA:
        return ColorRGBA(r=red, g=green, b=blue, a=alpha)

    @staticmethod
    def _points(points, z: float = 0.0) -> list[Point]:
        return [
            Point(x=float(point[0]), y=float(point[1]), z=z)
            for point in points
        ]

    def _base_marker(self, header: Header, marker_id: int,
                     marker_type: int, name: str) -> Marker:
        marker = Marker()
        marker.header = header
        marker.ns = name
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _make_markers(self, stamp) -> MarkerArray:
        header = self._header(stamp, self.odom_frame)
        result = MarkerArray()

        center = self._base_marker(header, 0, Marker.LINE_STRIP, "centerline")
        center.scale.x = 0.025
        center.color = self._color(0.15, 1.0, 0.2)
        center.points = self._points(self.course.centerline, 0.015)

        left = self._base_marker(header, 1, Marker.SPHERE_LIST, "left_cones")
        left.scale.x = 2.0 * self.course.cone_radius_m
        left.scale.y = 2.0 * self.course.cone_radius_m
        left.scale.z = 0.10
        left.color = self._color(0.15, 0.4, 1.0)
        left.points = self._points(self.course.left_cones, 0.05)

        right = self._base_marker(header, 2, Marker.SPHERE_LIST, "right_cones")
        right.scale.x = 2.0 * self.course.cone_radius_m
        right.scale.y = 2.0 * self.course.cone_radius_m
        right.scale.z = 0.10
        right.color = self._color(1.0, 0.25, 0.1)
        right.points = self._points(self.course.right_cones, 0.05)

        vehicle = self._base_marker(header, 3, Marker.CYLINDER, "vehicle")
        vehicle.pose.position.x = self.state.x
        vehicle.pose.position.y = self.state.y
        vehicle.pose.position.z = 0.03
        vehicle.pose.orientation = self._yaw_quaternion(self.state.yaw)
        diameter = 2.0 * self.vehicle_footprint_radius_m
        vehicle.scale.x = diameter
        vehicle.scale.y = diameter
        vehicle.scale.z = 0.06
        vehicle.color = self._color(
            1.0 if self._collision else 0.9,
            0.05 if self._collision else 0.9,
            0.05,
            0.75,
        )
        result.markers.extend((center, left, right, vehicle))
        return result

    def _make_status(self, stamp) -> DiagnosticArray:
        projection = project_to_polyline(
            (self.state.x, self.state.y), self.course.centerline
        )
        clearance = obstacle_clearance_m(
            (self.state.x, self.state.y),
            self._obstacles,
            self.vehicle_footprint_radius_m,
        )
        drop_active = self._scan_drop_active(time.monotonic())
        if self._collision:
            level = DiagnosticStatus.ERROR
            reason = "COLLISION"
        elif drop_active or not self._command_valid:
            level = DiagnosticStatus.WARN
            reason = (
                "SCAN_DROP_ACTIVE" if drop_active else self._command_reason
            )
        else:
            level = DiagnosticStatus.OK
            reason = "OK"
        if self._last_command_receipt_monotonic is None:
            command_receipt_age_s = float("inf")
        else:
            command_receipt_age_s = (
                time.monotonic() - self._last_command_receipt_monotonic
            )
        values = {
            "status": reason,
            "scenario": self.course.name,
            "along_track_m": projection.along_track_m,
            "lateral_error_m": projection.signed_lateral_m,
            "absolute_lateral_error_m": projection.absolute_lateral_m,
            "clearance_m": clearance,
            "collision": self._collision,
            "collision_count": self._collision_count,
            "minimum_clearance_m": self._minimum_clearance_m,
            "maximum_abs_lateral_m": self._maximum_abs_lateral_m,
            "scan_count": self._scan_count,
            "dropped_scan_count": self._dropped_scan_count,
            "received_command_count": self._received_command_count,
            "rejected_command_count": self._rejected_command_count,
            "command_valid": self._command_valid,
            "command_reason": self._command_reason,
            "command_receipt_age_s": command_receipt_age_s,
            "target_speed_mps": self._target_speed_mps,
            "target_steering_rad": self._target_steering_rad,
            "applied_speed_mps": self._applied_speed_mps,
            "applied_steering_rad": self._applied_steering_rad,
            "x_m": self.state.x,
            "y_m": self.state.y,
            "yaw_rad": self.state.yaw,
        }
        item = DiagnosticStatus()
        item.level = level
        item.name = f"{self.get_fully_qualified_name()}: synthetic world"
        item.hardware_id = "deterministic_2d_simulator"
        item.message = reason
        for key, value in values.items():
            if isinstance(value, float):
                text = f"{value:.6g}" if isfinite(value) else "inf"
            else:
                text = str(value)
            item.values.append(KeyValue(key=key, value=text))
        message = DiagnosticArray()
        message.header = self._header(stamp, self.odom_frame)
        message.status.append(item)
        return message

    def _auto_enable_callback(self) -> None:
        if not self.auto_enable_controller or self._enable_succeeded:
            return
        now_monotonic = time.monotonic()
        if now_monotonic - self._started_monotonic < self.auto_enable_delay_s:
            return
        if self._enable_future is not None:
            if not self._enable_future.done():
                return
            try:
                response = self._enable_future.result()
            except Exception as error:  # service transport failure
                self.get_logger().warn(
                    f"Controller enable call failed: {error}"
                )
            else:
                if response is not None and response.success:
                    self._enable_succeeded = True
                    self.get_logger().info(
                        "Controller enable service succeeded"
                    )
                    return
                message = (
                    response.message if response is not None else "no response"
                )
                self.get_logger().warn(
                    f"Controller refused automatic enable: {message}"
                )
            self._enable_future = None
        if now_monotonic - self._last_enable_attempt_monotonic < 0.5:
            return
        self._last_enable_attempt_monotonic = now_monotonic
        if not self.enable_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = True
        self._enable_future = self.enable_client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticConeWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Humble may surface SIGTERM as RCLError after its signal handler has
        # already invalidated the context instead of ExternalShutdownException.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
