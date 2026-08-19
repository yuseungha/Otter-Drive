"""Fixed-ROI HSV traffic-light detector."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from kmu_track.traffic_light_core import SignalState, TrafficLightDetector


class TrafficLightDetectorNode(Node):
    """Detect the start green and shortcut left-arrow lamp in fixed ROIs."""

    def __init__(self) -> None:
        super().__init__('traffic_light_detector')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('start_roi', [0.35, 0.05, 0.30, 0.30])
        self.declare_parameter('left_roi', [0.35, 0.05, 0.30, 0.30])
        self.declare_parameter('min_green_ratio', 0.02)
        self.declare_parameter('min_red_ratio', 0.02)
        self.declare_parameter('confirm_red_frames', 5)
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('lost_signal_frames', 1)
        self.declare_parameter('min_blob_area', 50.0)
        self.declare_parameter('morphology_kernel', 3)
        self.declare_parameter('left_arrow_min_aspect_ratio', 1.20)
        self.declare_parameter('left_arrow_max_solidity', 0.90)
        self.declare_parameter('left_arrow_min_direction_ratio', 1.20)
        self.declare_parameter(
            'left_arrow_line_max_head_aspect_ratio', 1.10)
        self.declare_parameter(
            'left_arrow_line_min_shaft_aspect_ratio', 1.50)
        self.declare_parameter('start_requires_red', True)
        self.declare_parameter('left_requires_red', False)

        self.bridge = CvBridge()
        common = {
            'min_green_ratio': float(
                self.get_parameter('min_green_ratio').value),
            'min_red_ratio': float(self.get_parameter('min_red_ratio').value),
            'min_blob_area': float(self.get_parameter('min_blob_area').value),
            'morphology_kernel': int(
                self.get_parameter('morphology_kernel').value),
            'confirm_red_frames': int(
                self.get_parameter('confirm_red_frames').value),
            'confirm_green_frames': int(
                self.get_parameter('confirm_frames').value),
            'lost_signal_frames': int(
                self.get_parameter('lost_signal_frames').value),
            'left_arrow_min_aspect_ratio': float(
                self.get_parameter('left_arrow_min_aspect_ratio').value),
            'left_arrow_max_solidity': float(
                self.get_parameter('left_arrow_max_solidity').value),
            'left_arrow_min_direction_ratio': float(
                self.get_parameter('left_arrow_min_direction_ratio').value),
            'left_arrow_line_max_head_aspect_ratio': float(
                self.get_parameter(
                    'left_arrow_line_max_head_aspect_ratio').value),
            'left_arrow_line_min_shaft_aspect_ratio': float(
                self.get_parameter(
                    'left_arrow_line_min_shaft_aspect_ratio').value),
        }
        self.start_detector = TrafficLightDetector(
            roi=self.get_parameter('start_roi').value,
            require_red_before_green=bool(
                self.get_parameter('start_requires_red').value),
            **common,
        )
        self.left_detector = TrafficLightDetector(
            roi=self.get_parameter('left_roi').value,
            require_red_before_green=bool(
                self.get_parameter('left_requires_red').value),
            **common,
        )

        self.start_pub = self.create_publisher(
            Bool, '/perception/start_signal', 10)
        self.left_pub = self.create_publisher(
            Bool, '/perception/left_signal', 10)
        self.start_ratio_pub = self.create_publisher(
            Float32, '/perception/start_green_ratio', 10)
        self.left_ratio_pub = self.create_publisher(
            Float32, '/perception/left_green_ratio', 10)
        self.left_arrow_score_pub = self.create_publisher(
            Float32, '/perception/left_arrow_score', 10)
        self.state_pub = self.create_publisher(
            String, '/perception/traffic_light_state', 10)
        self.reason_pub = self.create_publisher(
            String, '/perception/traffic_light_reason', 10)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Traffic-light detector ready on "
            f"{self.get_parameter('image_topic').value}")

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            start = self.start_detector.analyze(bgr)
            left = self.left_detector.analyze(bgr)
        except (ValueError, RuntimeError, cv2.error) as error:
            self.get_logger().error(f'traffic light detection failed: {error}')
            return

        self.start_pub.publish(Bool(data=start.state == SignalState.GO))
        self.left_pub.publish(
            Bool(data=left.state == SignalState.TURN_LEFT))
        self.start_ratio_pub.publish(
            Float32(data=float(start.evidence.green_ratio)))
        self.left_ratio_pub.publish(
            Float32(data=float(left.evidence.green_ratio)))
        self.left_arrow_score_pub.publish(
            Float32(data=float(left.evidence.left_arrow_score)))
        overall = left if left.state == SignalState.TURN_LEFT else start
        self.state_pub.publish(String(data=overall.state.value))
        self.reason_pub.publish(String(data=overall.reason))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrafficLightDetectorNode()
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
