"""Publish controllable frames from a video file as a ROS 2 camera topic."""

from pathlib import Path

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String, UInt32

from kmu_track.image_message import bgr_to_image_message


PLAYBACK_RATES = (0.25, 0.50, 1.0, 1.50, 2.0)


class VideoSourceNode(Node):
    """Replay an MP4 with pause, frame-step, rewind, and speed controls."""

    def __init__(self) -> None:
        super().__init__('video_source')
        self.declare_parameter('video_path', '')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('frame_id', 'front_camera')
        self.declare_parameter('loop', True)
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('output_width', 0)
        self.declare_parameter('output_height', 0)
        self.declare_parameter('wait_for_lane_ready', False)

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
        self.duration_sec = (
            self.frame_count / self.fps if self.frame_count > 0 else 0.0)
        requested_rate = float(self.get_parameter('playback_rate').value)
        self.playback_rate = min(
            PLAYBACK_RATES, key=lambda value: abs(value - requested_rate))
        self.frame_index = 0
        self.last_published_index = -1
        self.paused = False
        self.lane_ready = not bool(
            self.get_parameter('wait_for_lane_ready').value)
        self._playback_accumulator = 1.0
        self.image_pub = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data,
        )
        self.frame_pub = self.create_publisher(UInt32, '/video/frame_index', 10)
        self.time_pub = self.create_publisher(Float32, '/video/time_sec', 10)
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.paused_pub = self.create_publisher(Bool, '/video/paused', state_qos)
        self.rate_pub = self.create_publisher(
            Float32, '/video/playback_rate', state_qos)
        self.duration_pub = self.create_publisher(
            Float32, '/video/duration_sec', state_qos)
        self.create_subscription(String, '/video/command', self._on_command, 10)
        self.create_subscription(
            Bool, '/lane/detector_ready', self._on_lane_ready, state_qos)

        maximum_rate = max(PLAYBACK_RATES)
        self.timer = self.create_timer(
            1.0 / (self.fps * maximum_rate), self._on_timer)
        self._publish_state()
        self.get_logger().info(
            f'Video ready: {self.video_path} | {self.fps:.2f} FPS | '
            f'{self.duration_sec:.1f} s | rate={self.playback_rate:.2f}x')

    def _on_lane_ready(self, message: Bool) -> None:
        if bool(message.data) and not self.lane_ready:
            self.lane_ready = True
            self._playback_accumulator = 1.0
            self.get_logger().info('Lane detector ready; video playback started')

    def _publish_state(self) -> None:
        self.paused_pub.publish(Bool(data=self.paused))
        self.rate_pub.publish(Float32(data=float(self.playback_rate)))
        self.duration_pub.publish(Float32(data=float(self.duration_sec)))

    def _set_rate(self, direction: int) -> None:
        current = min(
            range(len(PLAYBACK_RATES)),
            key=lambda index: abs(PLAYBACK_RATES[index] - self.playback_rate),
        )
        current = max(0, min(len(PLAYBACK_RATES) - 1, current + direction))
        self.playback_rate = PLAYBACK_RATES[current]
        self.rate_pub.publish(Float32(data=float(self.playback_rate)))
        self.get_logger().info(f'Playback rate {self.playback_rate:.2f}x')

    def _on_command(self, message: String) -> None:
        command = message.data.strip().lower()
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
            self.last_published_index = -1
            self._playback_accumulator = 1.0
            if self.paused:
                self._publish_index(0)
            self.get_logger().info('Video restarted')
        elif command == 'step_forward' and self.paused:
            target = min(self.frame_count - 1, self.last_published_index + 1)
            self._publish_index(max(0, target))
        elif command == 'step_back' and self.paused:
            target = max(0, self.last_published_index - 1)
            self._publish_index(target)
        elif command == 'rate_up':
            self._set_rate(1)
        elif command == 'rate_down':
            self._set_rate(-1)

    def _rewind_or_stop(self) -> bool:
        if bool(self.get_parameter('loop').value):
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_index = 0
            self.last_published_index = -1
            self.get_logger().info('Looping video')
            return True
        self.paused = True
        self.paused_pub.publish(Bool(data=True))
        self.get_logger().info('End of video')
        return False

    def _publish_index(self, index: int) -> bool:
        index = max(0, min(max(0, self.frame_count - 1), int(index)))
        if index != self.frame_index:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, index)
            self.frame_index = index
        ok, frame = self.capture.read()
        if not ok:
            return False
        output_width = int(self.get_parameter('output_width').value)
        output_height = int(self.get_parameter('output_height').value)
        if output_width > 0 and output_height > 0:
            frame = cv2.resize(
                frame,
                (output_width, output_height),
                interpolation=cv2.INTER_AREA,
            )
        message = bgr_to_image_message(
            frame,
            self.get_clock().now().to_msg(),
            str(self.get_parameter('frame_id').value),
        )
        self.image_pub.publish(message)
        self.frame_pub.publish(UInt32(data=index))
        self.time_pub.publish(Float32(data=float(index / self.fps)))
        self.last_published_index = index
        self.frame_index = index + 1
        return True

    def _on_timer(self) -> None:
        if self.paused or not self.lane_ready:
            return
        self._playback_accumulator += self.playback_rate / max(PLAYBACK_RATES)
        if self._playback_accumulator < 1.0:
            return
        self._playback_accumulator -= 1.0
        if self.frame_count > 0 and self.frame_index >= self.frame_count:
            if not self._rewind_or_stop():
                return
        if not self._publish_index(self.frame_index):
            if self._rewind_or_stop():
                self._publish_index(0)

    def destroy_node(self) -> bool:
        self.capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the controllable video source node."""
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
