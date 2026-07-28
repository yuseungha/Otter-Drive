#!/usr/bin/env python3
"""Publish a recorded video as ``sensor_msgs/Image`` messages."""

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoPublisherNode(Node):
    """Streams a video file on a ROS 2 image topic at its native or requested FPS."""

    def __init__(self) -> None:
        super().__init__('video_publisher_node')

        self.declare_parameter('video_path', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('fps', 0.0)
        self.declare_parameter('loop', True)

        video_path = Path(self.get_parameter('video_path').value).expanduser()
        if not video_path.is_file():
            raise FileNotFoundError(f'Video file was not found: {video_path}')

        self._capture = cv2.VideoCapture(str(video_path))
        if not self._capture.isOpened():
            raise RuntimeError(f'Could not open video: {video_path}')

        native_fps = self._capture.get(cv2.CAP_PROP_FPS)
        requested_fps = float(self.get_parameter('fps').value)
        self._fps = requested_fps if requested_fps > 0.0 else native_fps
        if self._fps <= 0.0:
            self.get_logger().warning('Video FPS is unavailable; using 30 FPS.')
            self._fps = 30.0

        topic = self.get_parameter('image_topic').value
        self._publisher = self.create_publisher(Image, topic, 10)
        self._bridge = CvBridge()
        self._frame_id = self.get_parameter('frame_id').value
        self._loop = bool(self.get_parameter('loop').value)
        self._timer = self.create_timer(1.0 / self._fps, self._publish_frame)

        width = int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Publishing {video_path.name} ({width}x{height} @ {self._fps:.2f} FPS) on {topic}'
        )

    def _publish_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            if not self._loop:
                self.get_logger().info('End of video reached; stopping publisher.')
                self._timer.cancel()
                return
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self._capture.read()
            if not ok:
                self.get_logger().error('Could not restart video playback.')
                self._timer.cancel()
                return

        message = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)

    def destroy_node(self) -> bool:
        if hasattr(self, '_capture'):
            self._capture.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VideoPublisherNode()
        rclpy.spin(node)
    except (FileNotFoundError, RuntimeError) as error:
        rclpy.logging.get_logger('video_publisher_node').fatal(str(error))
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
