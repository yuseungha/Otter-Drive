"""Independent OpenCV bird's-eye viewer for cone-planner validation."""

from math import atan2, degrees
import os
from pathlib import Path as FilePath
import time

import cv2
import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from .viewer_core import BevGeometry, metric_to_pixel, transform_scan_points


class ConeCvViewer(Node):
    """Render subscribed data on a timer so planner callbacks remain isolated."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "cone_cv_viewer", parameter_overrides=parameter_overrides or []
        )
        read_only = ParameterDescriptor(read_only=True)
        topic_defaults = {
            "scan_topic": "scan",
            "cones_topic": "cone_planner/cones",
            "raw_cones_topic": "cone_planner/raw_cones",
            "path_topic": "cone_planner/center_path",
            "markers_topic": "cone_planner/markers",
            "status_topic": "cone_planner/status",
            "mission_state_topic": "/mission/state",
            "planning_frame": "base_link",
        }
        for name, value in topic_defaults.items():
            self.declare_parameter(name, value, read_only)
        self.declare_parameter("viewer_enabled", True, read_only)
        self.declare_parameter("viewer_width_px", 900, read_only)
        self.declare_parameter("viewer_height_px", 900, read_only)
        self.declare_parameter("viewer_range_forward_m", 2.5, read_only)
        self.declare_parameter("viewer_range_lateral_m", 1.5, read_only)
        self.declare_parameter("viewer_record_path", "", read_only)
        self.declare_parameter("viewer_render_hz", 20.0, read_only)
        self.declare_parameter("viewer_tf_timeout_s", 0.02, read_only)

        self.geometry = BevGeometry(
            width_px=int(self.get_parameter("viewer_width_px").value),
            height_px=int(self.get_parameter("viewer_height_px").value),
            range_forward_m=float(
                self.get_parameter("viewer_range_forward_m").value
            ),
            range_lateral_m=float(
                self.get_parameter("viewer_range_lateral_m").value
            ),
        )
        self.geometry.validate()
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        render_hz = float(self.get_parameter("viewer_render_hz").value)
        self.tf_timeout_s = float(self.get_parameter("viewer_tf_timeout_s").value)
        if not np.isfinite(render_hz) or render_hz <= 0.0:
            raise ValueError("viewer_render_hz must be finite and > 0")
        if not np.isfinite(self.tf_timeout_s) or self.tf_timeout_s < 0.0:
            raise ValueError("viewer_tf_timeout_s must be finite and >= 0")

        requested_gui = bool(self.get_parameter("viewer_enabled").value)
        display_available = bool(
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        )
        self.gui_active = requested_gui and display_available
        self.record_path = str(self.get_parameter("viewer_record_path").value).strip()
        if requested_gui and not display_available:
            if not self.record_path:
                self.record_path = "/tmp/cone_cv_viewer_latest.png"
            self.get_logger().warn(
                "DISPLAY/WAYLAND_DISPLAY is absent; using headless output: %s"
                % self.record_path
            )
        elif not requested_gui:
            self.get_logger().info("OpenCV GUI disabled by viewer_enabled=false")
        self.renderer_active = self.gui_active or bool(self.record_path)

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
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            sensor_qos,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("cones_topic").value),
            self._cones_callback,
            sensor_qos,
        )
        self.create_subscription(
            PoseArray,
            str(self.get_parameter("raw_cones_topic").value),
            self._raw_cones_callback,
            sensor_qos,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            reliable_qos,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter("markers_topic").value),
            self._markers_callback,
            sensor_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            str(self.get_parameter("status_topic").value),
            self._status_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mission_state_topic").value),
            self._mission_state_callback,
            latched_qos,
        )

        empty = np.empty((0, 2), dtype=float)
        self.raw_scan = empty
        self.raw_candidates = empty
        self.confirmed_cones = empty
        self.path = empty
        self.marker_points: dict[str, np.ndarray] = {}
        self.status_values: dict[str, str] = {"status": "NO_DATA"}
        self.mission_state = "WAITING"
        self._last_frame_time = time.perf_counter()
        self._fps = 0.0
        self._writer = None
        self._window_name = "Cone LiDAR BEV"
        if self.gui_active:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
        self.render_timer = self.create_timer(1.0 / render_hz, self._render_timer)

    @staticmethod
    def _pose_points(message: PoseArray) -> np.ndarray:
        if not message.poses:
            return np.empty((0, 2), dtype=float)
        return np.asarray(
            [[pose.position.x, pose.position.y] for pose in message.poses],
            dtype=float,
        )

    def _lookup_scan_transform(self, scan: LaserScan) -> tuple[float, float, float]:
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

    def _scan_callback(self, scan: LaserScan) -> None:
        try:
            transform = self._lookup_scan_transform(scan)
            self.raw_scan = transform_scan_points(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                transform,
            )
        except (TransformException, TypeError, ValueError):
            self.raw_scan = np.empty((0, 2), dtype=float)

    def _cones_callback(self, message: PoseArray) -> None:
        self.confirmed_cones = self._pose_points(message)

    def _raw_cones_callback(self, message: PoseArray) -> None:
        self.raw_candidates = self._pose_points(message)

    def _path_callback(self, message: Path) -> None:
        if not message.poses:
            self.path = np.empty((0, 2), dtype=float)
            return
        self.path = np.asarray(
            [
                [pose.pose.position.x, pose.pose.position.y]
                for pose in message.poses
            ],
            dtype=float,
        )

    def _markers_callback(self, message: MarkerArray) -> None:
        points: dict[str, np.ndarray] = {}
        for marker in message.markers:
            if not marker.ns or not marker.points:
                continue
            points[marker.ns] = np.asarray(
                [[point.x, point.y] for point in marker.points], dtype=float
            )
        self.marker_points = points

    def _status_callback(self, message: DiagnosticArray) -> None:
        if not message.status:
            self.status_values = {"status": "NO_STATUS"}
            return
        self.status_values = {
            value.key: value.value for value in message.status[0].values
        }
        self.status_values.setdefault("status", message.status[0].message)

    def _mission_state_callback(self, message: String) -> None:
        state = str(message.data).strip().upper()
        self.mission_state = state if state else "UNKNOWN"

    def _pixel(self, point) -> tuple[int, int]:
        return metric_to_pixel(float(point[0]), float(point[1]), self.geometry)

    def _visible(self, point) -> bool:
        return (
            0.0 <= float(point[0]) <= self.geometry.range_forward_m
            and abs(float(point[1])) <= self.geometry.range_lateral_m
        )

    def _draw_points(self, image, points, color, radius=4, thickness=-1) -> None:
        for point in points:
            if self._visible(point):
                cv2.circle(image, self._pixel(point), radius, color, thickness, cv2.LINE_AA)

    def _draw_line(self, image, points, color, thickness=2) -> None:
        visible = [self._pixel(point) for point in points if self._visible(point)]
        if len(visible) >= 2:
            cv2.polylines(
                image, [np.asarray(visible, dtype=np.int32)], False, color, thickness, cv2.LINE_AA
            )

    def _draw_direction_arrows(
        self, image, points, color, *, thickness=2
    ) -> None:
        """Draw one-way arrows along a planned polyline."""
        visible = [
            np.asarray(self._pixel(point), dtype=float)
            for point in points
            if self._visible(point)
        ]
        if len(visible) < 2:
            return
        cumulative = 0.0
        next_arrow = 90.0
        spacing_px = 115.0
        for start, end in zip(visible[:-1], visible[1:]):
            delta = end - start
            length = float(np.linalg.norm(delta))
            if length < 1.0:
                continue
            while cumulative + length >= next_arrow:
                fraction = (next_arrow - cumulative) / length
                tip = start + fraction * delta
                tail = tip - min(34.0, 0.55 * length) * delta / length
                cv2.arrowedLine(
                    image,
                    tuple(np.rint(tail).astype(int)),
                    tuple(np.rint(tip).astype(int)),
                    color,
                    thickness,
                    cv2.LINE_AA,
                    tipLength=0.32,
                )
                next_arrow += spacing_px
            cumulative += length

    def _draw_pair_links(self, image, points) -> None:
        """Show the cross-course pairs whose midpoints define the route."""
        self._draw_segment_links(image, points, (150, 150, 150), thickness=1)

    def _draw_segment_links(self, image, points, color, *, thickness=2) -> None:
        """Draw a LINE_LIST-style point array without joining separate segments."""
        for index in range(0, len(points) - 1, 2):
            first, second = points[index], points[index + 1]
            if self._visible(first) and self._visible(second):
                cv2.line(
                    image,
                    self._pixel(first),
                    self._pixel(second),
                    color,
                    thickness,
                    cv2.LINE_AA,
                )

    def _draw_grid(self, image) -> None:
        grid = (45, 45, 45)
        forward = 0.0
        while forward <= self.geometry.range_forward_m + 1.0e-9:
            cv2.line(
                image,
                self._pixel((forward, -self.geometry.range_lateral_m)),
                self._pixel((forward, self.geometry.range_lateral_m)),
                grid,
                1,
            )
            forward += 0.25
        lateral = -self.geometry.range_lateral_m
        while lateral <= self.geometry.range_lateral_m + 1.0e-9:
            cv2.line(
                image,
                self._pixel((0.0, lateral)),
                self._pixel((self.geometry.range_forward_m, lateral)),
                grid,
                1,
            )
            lateral += 0.25

    def render_frame(self) -> np.ndarray:
        image = np.zeros(
            (self.geometry.height_px, self.geometry.width_px, 3), dtype=np.uint8
        )
        self._draw_grid(image)
        self._draw_points(image, self.raw_scan, (115, 115, 115), radius=1)
        self._draw_points(image, self.raw_candidates, (0, 215, 255), radius=5)

        left = self.marker_points.get("matched_left", np.empty((0, 2)))
        right = self.marker_points.get("matched_right", np.empty((0, 2)))
        observed_boundaries = self.marker_points.get(
            "observed_boundaries", np.empty((0, 2))
        )
        pair_links = self.marker_points.get("matched_pairs", np.empty((0, 2)))
        raw_center = self.marker_points.get("raw_center", np.empty((0, 2)))
        virtual_left = self.marker_points.get("virtual_left", np.empty((0, 2)))
        virtual_right = self.marker_points.get("virtual_right", np.empty((0, 2)))
        self._draw_segment_links(
            image, observed_boundaries, (0, 215, 240), thickness=3
        )
        self._draw_pair_links(image, pair_links)
        self._draw_line(image, left, (255, 100, 20), 2)
        self._draw_line(image, right, (20, 30, 255), 2)
        self._draw_points(image, left, (255, 100, 20), radius=6)
        self._draw_points(image, right, (20, 30, 255), radius=6)
        self._draw_points(image, virtual_left, (255, 255, 0), radius=7)
        self._draw_points(image, virtual_right, (255, 0, 255), radius=7)

        status = self.status_values.get("status", "NO_STATUS")
        path_valid = status in {"OK", "OK_VIRTUAL"} and len(self.path) >= 2
        if path_valid:
            self._draw_line(image, self.path, (20, 255, 40), 4)
            self._draw_direction_arrows(
                image, self.path, (20, 255, 40), thickness=3
            )
        else:
            # The planner intentionally publishes an empty control Path when a
            # safety gate fails.  Its matched station midpoints are still useful
            # as a clearly labelled, non-drivable direction preview.
            self._draw_points(image, raw_center, (255, 40, 255), radius=6)
            if len(raw_center) >= 1:
                preview = np.vstack((np.zeros((1, 2), dtype=float), raw_center))
                self._draw_line(image, preview, (0, 170, 255), 3)
                self._draw_direction_arrows(
                    image, preview, (0, 170, 255), thickness=2
                )

        origin = self._pixel((0.0, 0.0))
        cv2.rectangle(
            image,
            (origin[0] - 22, max(0, origin[1] - 42)),
            (origin[0] + 22, origin[1]),
            (230, 230, 230),
            2,
        )
        values = self.status_values
        try:
            target = (
                float(values.get("lookahead_x_m", "0")),
                float(values.get("lookahead_y_m", "0")),
            )
            heading = float(values.get("target_heading_rad", "0"))
            steering = float(values.get("expected_steering_angle_rad", "0"))
            confidence = float(values.get("confidence", "0"))
            processing_ms = float(values.get("processing_ms", "0"))
        except ValueError:
            target = (0.0, 0.0)
            heading = steering = confidence = processing_ms = 0.0

        if status in {"OK", "OK_VIRTUAL"} and self._visible(target):
            target_px = self._pixel(target)
            cv2.circle(image, target_px, 9, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.arrowedLine(
                image, origin, target_px, (0, 255, 255), 3, cv2.LINE_AA, tipLength=0.12
            )

        now = time.perf_counter()
        dt = now - self._last_frame_time
        self._last_frame_time = now
        instant_fps = 1.0 / dt if dt > 1.0e-6 else 0.0
        self._fps = instant_fps if self._fps <= 0.0 else 0.85 * self._fps + 0.15 * instant_fps
        lines = (
            f"status: {status}",
            f"target: ({target[0]:+.3f}, {target[1]:+.3f}) m",
            f"heading: {degrees(heading):+.2f} deg",
            f"steering: {degrees(steering):+.2f} deg",
            f"confidence: {confidence:.3f}",
            f"FPS: {self._fps:.1f}  planner: {processing_ms:.2f} ms",
            "raw/confirmed/virtual: %s/%s/%s"
            % (
                values.get("raw_candidates", "0"),
                values.get("confirmed_cones", "0"),
                values.get("virtual_pairs", "0"),
            ),
        )
        for index, text in enumerate(lines):
            cv2.putText(
                image,
                text,
                (18, 28 + 25 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (235, 235, 235),
                1,
                cv2.LINE_AA,
            )

        mode_colors = {
            "LANE": (40, 210, 40),
            "CONE": (0, 190, 255),
            "OBSTACLE_AVOID": (40, 40, 255),
        }
        mode_text = f"MODE: {self.mission_state}"
        mode_color = mode_colors.get(self.mission_state, (180, 180, 180))
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 1.05
        thickness = 2
        text_size, baseline = cv2.getTextSize(
            mode_text, font, font_scale, thickness
        )
        text_x = max(18, self.geometry.width_px - text_size[0] - 24)
        text_y = 42
        cv2.rectangle(
            image,
            (text_x - 10, text_y - text_size[1] - 9),
            (text_x + text_size[0] + 10, text_y + baseline + 7),
            (15, 15, 15),
            -1,
        )
        cv2.rectangle(
            image,
            (text_x - 10, text_y - text_size[1] - 9),
            (text_x + text_size[0] + 10, text_y + baseline + 7),
            mode_color,
            2,
        )
        cv2.putText(
            image,
            mode_text,
            (text_x, text_y),
            font,
            font_scale,
            mode_color,
            thickness,
            cv2.LINE_AA,
        )
        if status not in {"OK", "OK_VIRTUAL"}:
            text = (
                "CENTER PREVIEW / STOP"
                if len(raw_center) >= 1
                else "STOP / INVALID"
            )
            size = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 1.05, 3)[0]
            cv2.putText(
                image,
                text,
                ((self.geometry.width_px - size[0]) // 2, self.geometry.height_px // 2),
                cv2.FONT_HERSHEY_DUPLEX,
                1.05,
                (0, 170, 255) if len(raw_center) >= 1 else (0, 0, 255),
                3,
                cv2.LINE_AA,
            )
        return image

    def _record(self, frame: np.ndarray) -> None:
        if not self.record_path:
            return
        destination = FilePath(self.record_path).expanduser()
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.suffix.lower() in {".avi", ".mp4", ".mkv"}:
            if self._writer is None:
                codec = "MJPG" if destination.suffix.lower() == ".avi" else "mp4v"
                self._writer = cv2.VideoWriter(
                    str(destination),
                    cv2.VideoWriter_fourcc(*codec),
                    float(self.get_parameter("viewer_render_hz").value),
                    (self.geometry.width_px, self.geometry.height_px),
                )
                if not self._writer.isOpened():
                    self._writer = None
                    self.get_logger().error("Could not open video output: %s" % destination)
                    self.record_path = ""
                    return
            self._writer.write(frame)
        else:
            # Keep the last complete frame if the node is interrupted while
            # encoding the next PNG/JPEG (common during SSH validation).
            temporary = destination.with_name(
                destination.stem + ".tmp" + destination.suffix
            )
            if cv2.imwrite(str(temporary), frame):
                os.replace(temporary, destination)

    def _render_timer(self) -> None:
        if not self.renderer_active:
            return
        frame = self.render_frame()
        self._record(frame)
        if self.gui_active:
            cv2.imshow(self._window_name, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.get_logger().info("q pressed; shutting down cone_cv_viewer")
                rclpy.shutdown()

    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        if self.gui_active:
            cv2.destroyWindow(self._window_name)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeCvViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
