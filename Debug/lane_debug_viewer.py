#!/usr/bin/env python3
"""OpenCV dashboard for inspecting the lane detector without actuation."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
from time import monotonic
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String


def image_message_to_numpy(message: Image) -> np.ndarray:
    """Convert common ROS image encodings without cv_bridge."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    rows = data.reshape(message.height, message.step)
    if message.encoding in {'bgr8', 'rgb8'}:
        image = rows[:, :message.width * 3].reshape(
            message.height, message.width, 3).copy()
        if message.encoding == 'rgb8':
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image
    if message.encoding in {'mono8', '8UC1'}:
        return rows[:, :message.width].copy()
    raise ValueError(f'unsupported image encoding: {message.encoding}')


def fit_image(image: Optional[np.ndarray], width: int, height: int,
              label: str) -> np.ndarray:
    """Letterbox an image into a labelled dashboard tile."""
    tile = np.full((height, width, 3), 24, dtype=np.uint8)
    if image is None or image.size == 0:
        cv2.putText(
            tile, 'WAITING FOR TOPIC', (20, height // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 180, 255), 2,
            cv2.LINE_AA)
    else:
        source = image
        if source.ndim == 2:
            source = cv2.cvtColor(source, cv2.COLOR_GRAY2BGR)
        scale = min(width / source.shape[1], height / source.shape[0])
        resized = cv2.resize(
            source,
            (max(1, int(source.shape[1] * scale)),
             max(1, int(source.shape[0] * scale))),
            interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR)
        x = (width - resized.shape[1]) // 2
        y = (height - resized.shape[0]) // 2
        tile[y:y + resized.shape[0], x:x + resized.shape[1]] = resized

    cv2.rectangle(tile, (0, 0), (width - 1, height - 1), (75, 75, 75), 1)
    cv2.rectangle(tile, (0, 0), (width - 1, 34), (12, 12, 12), -1)
    cv2.putText(
        tile, label, (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
        (235, 235, 235), 2, cv2.LINE_AA)
    return tile


class LaneDebugViewer(Node):
    """Display lane images and health metrics in a single OpenCV window."""

    def __init__(self) -> None:
        super().__init__('lane_debug_viewer')
        self._declare_parameters()
        self.window_name = str(self.get_parameter('window_name').value)
        self.window_width = max(
            800, int(self.get_parameter('window_width').value))
        self.window_height = max(
            600, int(self.get_parameter('window_height').value))
        self.stale_timeout = max(
            0.1, float(self.get_parameter('stale_timeout_sec').value))
        self.screenshot_directory = Path(os.path.expanduser(str(
            self.get_parameter('screenshot_directory').value)))

        self.raw_image: Optional[np.ndarray] = None
        self.debug_image: Optional[np.ndarray] = None
        self.overlay_image: Optional[np.ndarray] = None
        self.binary_image: Optional[np.ndarray] = None
        self.valid: Optional[bool] = None
        self.confidence: Optional[float] = None
        self.center_error: Optional[float] = None
        self.heading_error: Optional[float] = None
        self.inference_ms: Optional[float] = None
        self.detection_summary = 'waiting'
        self.last_image_time: Optional[float] = None
        self.last_metric_time: Optional[float] = None
        self.frame_rate = 0.0
        self._previous_frame_time: Optional[float] = None
        self._paused = False
        self._last_dashboard: Optional[np.ndarray] = None

        self._create_subscriptions()
        refresh_hz = max(
            1.0, float(self.get_parameter('refresh_rate_hz').value))
        self.timer = self.create_timer(1.0 / refresh_hz, self._render)
        self.get_logger().info(
            'Lane debug viewer ready. Keys: Q/Esc quit, Space pause, '
            'S screenshot')

    def _declare_parameters(self) -> None:
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('debug_topic', '/lane/yolo_debug')
        self.declare_parameter('overlay_topic', '/lane/lane_overlay')
        self.declare_parameter('binary_topic', '/lane/debug_binary')
        self.declare_parameter('valid_topic', '/lane/valid')
        self.declare_parameter('confidence_topic', '/lane/confidence')
        self.declare_parameter('center_error_topic', '/lane/center_error')
        self.declare_parameter('heading_error_topic', '/lane/heading_error')
        self.declare_parameter('inference_ms_topic', '/lane/inference_ms')
        self.declare_parameter(
            'detections_topic', '/lane/yolo_detections')
        self.declare_parameter('window_name', 'KMU Lane Debug')
        self.declare_parameter('window_width', 1440)
        self.declare_parameter('window_height', 900)
        self.declare_parameter('refresh_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_sec', 1.0)
        self.declare_parameter('screenshot_directory', '/tmp/lane_debug')

    def _create_subscriptions(self) -> None:
        image_topics = (
            ('image_topic', self._on_raw_image),
            ('debug_topic', self._on_debug_image),
            ('overlay_topic', self._on_overlay_image),
            ('binary_topic', self._on_binary_image),
        )
        for parameter, callback in image_topics:
            self.create_subscription(
                Image, str(self.get_parameter(parameter).value), callback,
                qos_profile_sensor_data)

        metric_topics = (
            (Bool, 'valid_topic', self._on_valid),
            (Float32, 'confidence_topic', self._on_confidence),
            (Float32, 'center_error_topic', self._on_center_error),
            (Float32, 'heading_error_topic', self._on_heading_error),
            (Float32, 'inference_ms_topic', self._on_inference_ms),
            (String, 'detections_topic', self._on_detections),
        )
        for message_type, parameter, callback in metric_topics:
            self.create_subscription(
                message_type, str(self.get_parameter(parameter).value),
                callback, 10)

    def _decode_image(self, message: Image) -> Optional[np.ndarray]:
        try:
            return image_message_to_numpy(message)
        except (ValueError, BufferError) as error:
            self.get_logger().warning(
                f'Cannot decode image: {error}',
                throttle_duration_sec=2.0)
            return None

    def _on_raw_image(self, message: Image) -> None:
        self.raw_image = self._decode_image(message)

    def _on_debug_image(self, message: Image) -> None:
        image = self._decode_image(message)
        if image is None:
            return
        self.debug_image = image
        now = monotonic()
        if self._previous_frame_time is not None:
            instantaneous = 1.0 / max(now - self._previous_frame_time, 1e-6)
            self.frame_rate = (
                instantaneous if self.frame_rate == 0.0
                else 0.85 * self.frame_rate + 0.15 * instantaneous)
        self._previous_frame_time = now
        self.last_image_time = now

    def _on_overlay_image(self, message: Image) -> None:
        self.overlay_image = self._decode_image(message)

    def _on_binary_image(self, message: Image) -> None:
        self.binary_image = self._decode_image(message)

    def _touch_metrics(self) -> None:
        self.last_metric_time = monotonic()

    def _on_valid(self, message: Bool) -> None:
        self.valid = bool(message.data)
        self._touch_metrics()

    def _on_confidence(self, message: Float32) -> None:
        self.confidence = float(message.data)
        self._touch_metrics()

    def _on_center_error(self, message: Float32) -> None:
        self.center_error = float(message.data)
        self._touch_metrics()

    def _on_heading_error(self, message: Float32) -> None:
        self.heading_error = float(message.data)
        self._touch_metrics()

    def _on_inference_ms(self, message: Float32) -> None:
        self.inference_ms = float(message.data)
        self._touch_metrics()

    def _on_detections(self, message: String) -> None:
        self.detection_summary = self._summarize_detections(message.data)
        self._touch_metrics()

    @staticmethod
    def _summarize_detections(payload: str) -> str:
        try:
            value = json.loads(payload)
        except (json.JSONDecodeError, TypeError):
            return payload[:72] if payload else 'none'
        if isinstance(value, list):
            return f'{len(value)} detection(s)'
        if isinstance(value, dict):
            for key in ('detections', 'instances', 'classes'):
                items = value.get(key)
                if isinstance(items, list):
                    return f'{len(items)} {key}'
            return ', '.join(str(key) for key in list(value)[:5]) or 'none'
        return str(value)[:72]

    @staticmethod
    def _metric(value: Optional[float], precision: int = 3) -> str:
        return '--' if value is None else f'{value:.{precision}f}'

    def _status_panel(self, width: int, height: int) -> np.ndarray:
        panel = np.full((height, width, 3), (28, 28, 28), dtype=np.uint8)
        now = monotonic()
        image_age = None if self.last_image_time is None else (
            now - self.last_image_time)
        metric_age = None if self.last_metric_time is None else (
            now - self.last_metric_time)
        stale = image_age is None or image_age > self.stale_timeout

        if stale:
            state_text, state_color = 'NO SIGNAL / STALE', (0, 165, 255)
        elif self.valid is True:
            state_text, state_color = 'LANE VALID', (60, 220, 80)
        elif self.valid is False:
            state_text, state_color = 'LANE INVALID', (50, 70, 240)
        else:
            state_text, state_color = 'WAITING FOR STATUS', (0, 165, 255)

        cv2.rectangle(panel, (0, 0), (width - 1, height - 1),
                      (75, 75, 75), 1)
        cv2.rectangle(panel, (0, 0), (width - 1, 58), state_color, -1)
        cv2.putText(panel, state_text, (18, 39),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (10, 10, 10), 2,
                    cv2.LINE_AA)

        lines = [
            ('Confidence', self._metric(self.confidence, 3)),
            ('Center error', self._metric(self.center_error, 3)),
            ('Heading error', self._metric(self.heading_error, 3)),
            ('Inference', f'{self._metric(self.inference_ms, 1)} ms'),
            ('Debug FPS', self._metric(self.frame_rate, 1)),
            ('Image age', '--' if image_age is None else f'{image_age:.2f} s'),
            ('Metric age', '--' if metric_age is None else f'{metric_age:.2f} s'),
            ('Detections', self.detection_summary),
        ]
        y = 94
        for label, value in lines:
            cv2.putText(panel, label, (18, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (170, 170, 170), 1, cv2.LINE_AA)
            cv2.putText(panel, value[:36], (190, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                        (235, 235, 235), 2, cv2.LINE_AA)
            y += 32

        hint = 'Q/Esc quit | Space pause | S screenshot'
        cv2.putText(panel, hint, (18, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (135, 200, 255), 1,
                    cv2.LINE_AA)
        if self._paused:
            cv2.putText(panel, 'PAUSED', (width - 120, 39),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 2,
                        cv2.LINE_AA)
        return panel

    def _build_dashboard(self) -> np.ndarray:
        half_width = self.window_width // 2
        half_height = self.window_height // 2
        primary = self.debug_image if self.debug_image is not None else (
            self.raw_image)
        secondary = self.overlay_image if self.overlay_image is not None else (
            self.raw_image)
        top = np.hstack((
            fit_image(primary, half_width, half_height, 'YOLO DETECTION'),
            fit_image(secondary, self.window_width - half_width, half_height,
                      'LANE PATH OVERLAY'),
        ))
        bottom = np.hstack((
            fit_image(self.binary_image, half_width,
                      self.window_height - half_height, 'BINARY MASK'),
            self._status_panel(
                self.window_width - half_width,
                self.window_height - half_height),
        ))
        return np.vstack((top, bottom))

    def _render(self) -> None:
        if not self._paused or self._last_dashboard is None:
            self._last_dashboard = self._build_dashboard()
        cv2.imshow(self.window_name, self._last_dashboard)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            self.get_logger().info('Viewer closed by keyboard')
            rclpy.shutdown()
        elif key == ord(' '):
            self._paused = not self._paused
        elif key in (ord('s'), ord('S')):
            self._save_screenshot()

    def _save_screenshot(self) -> None:
        if self._last_dashboard is None:
            return
        try:
            self.screenshot_directory.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            target = self.screenshot_directory / f'lane_debug_{timestamp}.png'
            if not cv2.imwrite(str(target), self._last_dashboard):
                raise OSError('cv2.imwrite returned false')
            self.get_logger().info(f'Screenshot saved: {target}')
        except OSError as error:
            self.get_logger().error(f'Cannot save screenshot: {error}')

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the lane debug dashboard."""
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError(
            'Lane debug viewer needs a desktop display. Set DISPLAY or use '
            'X11 forwarding when running in Docker/SSH.')
    rclpy.init(args=args)
    node = LaneDebugViewer()
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
