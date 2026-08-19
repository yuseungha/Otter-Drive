"""Publish frames from a USB V4L2 camera as a ROS 2 image topic."""

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from kmu_track.image_message import bgr_to_image_message


def opencv_camera_source(device: str):
    """Convert /dev/videoN to the numeric index some V4L2 builds require."""
    prefix = '/dev/video'
    if device.startswith(prefix):
        index = device[len(prefix):]
        if index.isdigit():
            return int(index)
    return device


class UsbCameraNode(Node):
    """Read a Logitech or compatible USB camera with automatic reconnect."""

    def __init__(self) -> None:
        super().__init__('usb_camera_source')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.device = str(self.get_parameter('device').value)
        self.capture_source = opencv_camera_source(self.device)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        if self.fps <= 0.0:
            raise ValueError('fps must be positive')
        if self.reconnect_interval <= 0.0:
            raise ValueError('reconnect_interval_sec must be positive')

        self.capture = None
        self.last_open_attempt = float('-inf')
        self.consecutive_failures = 0
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            10,
        )
        self.timer = self.create_timer(1.0 / self.fps, self._on_timer)
        self.get_logger().info(
            f'USB camera source waiting for {self.device}')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _open_if_due(self) -> None:
        if self.capture is not None:
            return
        now = self._now_sec()
        if now - self.last_open_attempt < self.reconnect_interval:
            return
        self.last_open_attempt = now
        capture = cv2.VideoCapture(self.capture_source, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            self.get_logger().warn(
                f'Cannot open camera {self.device}; retrying',
                throttle_duration_sec=5.0,
            )
            return
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture
        self.consecutive_failures = 0
        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Camera opened: {self.device} {actual_width}x{actual_height}')

    def _disconnect(self, reason: str) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.get_logger().error(f'Camera disconnected: {reason}')

    def _on_timer(self) -> None:
        self._open_if_due()
        if self.capture is None:
            return
        ok, frame = self.capture.read()
        if not ok or frame is None or not frame.size:
            self.consecutive_failures += 1
            if self.consecutive_failures >= 10:
                self._disconnect('ten consecutive read failures')
            return
        self.consecutive_failures = 0
        message = bgr_to_image_message(
            frame, self.get_clock().now().to_msg(), self.frame_id)
        self.publisher.publish(message)

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsbCameraNode()
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
