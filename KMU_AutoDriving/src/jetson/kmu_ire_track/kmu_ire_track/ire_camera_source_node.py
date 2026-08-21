"""Low-latency 1080p USB camera source for the IRE lane pipeline."""

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

from kmu_track.image_message import bgr_to_image_message
from kmu_track.usb_camera_node import opencv_camera_source


def camera_qos() -> QoSProfile:
    """Keep only the newest camera frame and never wait for a slow reader."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class IreCameraSourceNode(Node):
    """Publish MJPEG camera frames with a one-frame ROS queue."""

    def __init__(self) -> None:
        super().__init__('ire_camera_source')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('reconnect_interval_sec', 1.0)

        self.device = str(self.get_parameter('device').value)
        self.capture_source = opencv_camera_source(self.device)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value).upper()
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value)
        if self.width <= 0 or self.height <= 0 or self.fps <= 0.0:
            raise ValueError('camera width, height, and fps must be positive')
        if len(self.fourcc) != 4:
            raise ValueError('camera fourcc must contain exactly four characters')

        self.capture = None
        self.last_open_attempt = float('-inf')
        self.consecutive_failures = 0
        self.publisher = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            camera_qos(),
        )
        self.timer = self.create_timer(1.0 / self.fps, self._on_timer)
        self.get_logger().info(
            f'IRE camera waiting for {self.device} at '
            f'{self.width}x{self.height}@{self.fps:g} {self.fourcc}')

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
            self.get_logger().warning(
                f'Cannot open camera {self.device}; retrying',
                throttle_duration_sec=5.0,
            )
            return
        capture.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.fourcc),
        )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture = capture
        self.consecutive_failures = 0
        actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
        actual_format = ''.join(
            chr((actual_fourcc >> (8 * index)) & 0xFF)
            for index in range(4)
        )
        self.get_logger().info(
            f'IRE camera opened: {self.device} '
            f'{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}@'
            f'{capture.get(cv2.CAP_PROP_FPS):g} {actual_format}')

    def _disconnect(self, reason: str) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.get_logger().error(f'IRE camera disconnected: {reason}')

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
        self.publisher.publish(bgr_to_image_message(
            frame,
            self.get_clock().now().to_msg(),
            self.frame_id,
        ))

    def destroy_node(self) -> bool:
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        return super().destroy_node()


def main(args=None) -> None:
    """Run the IRE USB camera publisher."""
    rclpy.init(args=args)
    node = IreCameraSourceNode()
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
