"""Publish frames from a video file as a ROS 2 camera topic."""

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String, UInt32


class VideoSourceNode(Node):
    """Replay an MP4 at its recorded frame rate."""

    def __init__(self) -> None:
        super().__init__('video_source')
        self.declare_parameter('video_path', '')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('loop', True)
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)

        self.video_path = Path(str(self.get_parameter('video_path').value))
        if not self.video_path.is_file():
            raise FileNotFoundError(f'video does not exist: {self.video_path}')

        self.capture = cv2.VideoCapture(str(self.video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f'cannot decode video: {self.video_path}')

        self.fps = float(self.capture.get(cv2.CAP_PROP_FPS))
        if self.fps <= 0.0:
            self.fps = 15.0
        self.frame_count = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.frame_index = 0
        self.paused = False
        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data,
        )
        self.frame_pub = self.create_publisher(UInt32, '/video/frame_index', 10)
        self.time_pub = self.create_publisher(Float32, '/video/time_sec', 10)
        paused_qos = QoSProfile(depth=1)
        paused_qos.reliability = ReliabilityPolicy.RELIABLE
        paused_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.paused_pub = self.create_publisher(Bool, '/video/paused', paused_qos)
        self.create_subscription(String, '/video/command', self._on_command, 10)

        playback_rate = max(
            0.05, float(self.get_parameter('playback_rate').value))
        self.timer = self.create_timer(
            1.0 / (self.fps * playback_rate), self._publish_next_frame)
        self.paused_pub.publish(Bool(data=False))
        duration = self.frame_count / self.fps if self.frame_count > 0 else 0.0
        self.get_logger().info(
            f'Video ready: {self.video_path} | {self.fps:.2f} FPS | '
            f'{duration:.1f} s')

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in {'pause', 'toggle_pause'}:
            self.paused = not self.paused if command == 'toggle_pause' else True
            self.paused_pub.publish(Bool(data=self.paused))
            self.get_logger().info('Paused' if self.paused else 'Playing')
        elif command in {'play', 'resume'}:
            self.paused = False
            self.paused_pub.publish(Bool(data=False))
            self.get_logger().info('Playing')
        elif command in {'restart', 'reset'}:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_index = 0
            self.get_logger().info('Video restarted')

    def _rewind_or_stop(self) -> bool:
        if bool(self.get_parameter('loop').value):
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_index = 0
            self.get_logger().info('Looping video')
            return True
        self.paused = True
        self.paused_pub.publish(Bool(data=True))
        self.get_logger().info('End of video')
        return False

    def _publish_next_frame(self) -> None:
        if self.paused:
            return
        ok, frame = self.capture.read()
        if not ok:
            if not self._rewind_or_stop():
                return
            ok, frame = self.capture.read()
            if not ok:
                self.get_logger().error('Failed to decode the first frame after rewind')
                self.paused = True
                return

        output_width = int(self.get_parameter('output_width').value)
        output_height = int(self.get_parameter('output_height').value)
        if output_width > 0 and output_height > 0:
            frame = cv2.resize(
                frame,
                (output_width, output_height),
                interpolation=cv2.INTER_AREA,
            )
        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        self.image_pub.publish(message)
        self.frame_pub.publish(UInt32(data=max(0, self.frame_index)))
        self.time_pub.publish(Float32(data=float(self.frame_index / self.fps)))
        self.frame_index += 1

    def destroy_node(self) -> bool:
        self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VideoSourceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
