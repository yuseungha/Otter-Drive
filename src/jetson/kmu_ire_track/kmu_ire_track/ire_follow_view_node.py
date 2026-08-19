"""Single-panel IRE follow-view window."""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


def image_message_to_bgr(message: Image) -> np.ndarray:
    """Convert a bgr8 or rgb8 ROS image, including padded rows."""
    if message.encoding not in {'bgr8', 'rgb8'}:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height, message.step)
    image = rows[:, :message.width * 3].reshape(
        message.height, message.width, 3).copy()
    if message.encoding == 'rgb8':
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def view_qos() -> QoSProfile:
    """Discard stale overlays instead of making perception wait."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class IreFollowViewNode(Node):
    """Display only panel 1: the planner overlay."""

    def __init__(self) -> None:
        super().__init__('ire_follow_view')
        self.declare_parameter('image_topic', '/lane/lane_overlay')
        self.declare_parameter('window_name', 'IRE Follow View')
        self.declare_parameter('display_width', 640)
        self.declare_parameter('display_height', 360)
        self.window_name = str(self.get_parameter('window_name').value)
        self.display_width = max(
            160, int(self.get_parameter('display_width').value))
        self.display_height = max(
            90, int(self.get_parameter('display_height').value))
        self.latest_frame = None
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            view_qos(),
        )
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(
            self.window_name, self.display_width, self.display_height)
        self.timer = self.create_timer(1.0 / 30.0, self._render)
        self.get_logger().info(
            f'Single follow view ready: {self.display_width}x'
            f'{self.display_height} | Q/Esc quit')

    def _on_image(self, message: Image) -> None:
        try:
            self.latest_frame = image_message_to_bgr(message)
        except (ValueError, cv2.error) as error:
            self.get_logger().error(
                f'follow-view image conversion failed: {error}',
                throttle_duration_sec=2.0,
            )

    def _render(self) -> None:
        if self.latest_frame is not None:
            display = self.latest_frame
            if display.shape[:2] != (
                self.display_height, self.display_width
            ):
                display = cv2.resize(
                    display,
                    (self.display_width, self.display_height),
                    interpolation=cv2.INTER_AREA,
                )
            cv2.imshow(self.window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key in {ord('q'), ord('Q'), 27}:
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the single planner view."""
    rclpy.init(args=args)
    node = IreFollowViewNode()
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
