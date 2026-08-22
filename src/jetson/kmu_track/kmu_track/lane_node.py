"""ROS 2 ROI, BEV, and HSV lane preprocessor."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from kmu_track.lane_core import preprocess_lane_frame


class LanePreprocessorNode(Node):
    """Publish BEV and adjustable HSV lane masks from the front camera."""

    def __init__(self) -> None:
        super().__init__('lane_preprocessor')
        self.declare_parameter('image_topic', '/camera/front/image_raw')
        self.declare_parameter('roi_top_ratio', 0.45)
        self.declare_parameter('bev_width', 640)
        self.declare_parameter('bev_height', 480)
        self.declare_parameter(
            'bev_source', [0.05, 0.98, 0.95, 0.98, 0.62, 0.05, 0.38, 0.05])
        self.declare_parameter(
            'bev_destination', [0.20, 0.99, 0.80, 0.99, 0.80, 0.0, 0.20, 0.0])
        self.declare_parameter('white_lower_hsv', [0, 0, 175])
        self.declare_parameter('white_upper_hsv', [180, 80, 255])
        self.declare_parameter('yellow_lower_hsv', [12, 70, 70])
        self.declare_parameter('yellow_upper_hsv', [42, 255, 255])
        self.declare_parameter('morphology_kernel', 3)
        self.declare_parameter('publish_individual_masks', True)

        self.bridge = CvBridge()
        self.thresholds = self._parameters_to_thresholds()
        self.bev_pub = self.create_publisher(
            Image, '/lane/bev_image', qos_profile_sensor_data)
        self.white_pub = self.create_publisher(
            Image, '/lane/white_mask', qos_profile_sensor_data)
        self.yellow_pub = self.create_publisher(
            Image, '/lane/yellow_mask', qos_profile_sensor_data)
        self.binary_pub = self.create_publisher(
            Image, '/lane/debug_binary', qos_profile_sensor_data)
        threshold_qos = QoSProfile(depth=1)
        threshold_qos.reliability = ReliabilityPolicy.RELIABLE
        threshold_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.threshold_pub = self.create_publisher(
            Int32MultiArray, '/lane/hsv_thresholds/current', threshold_qos)
        self.create_subscription(
            Int32MultiArray,
            '/lane/hsv_thresholds/set',
            self._on_thresholds,
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Lane preprocessor ready on "
            f"{self.get_parameter('image_topic').value}")
        self._publish_thresholds()

    def _parameters_to_thresholds(self):
        values = []
        for parameter in (
            'white_lower_hsv',
            'white_upper_hsv',
            'yellow_lower_hsv',
            'yellow_upper_hsv',
        ):
            values.extend(int(value) for value in self.get_parameter(parameter).value)
        return values

    def _publish_thresholds(self) -> None:
        self.threshold_pub.publish(Int32MultiArray(data=self.thresholds))

    def _on_thresholds(self, msg: Int32MultiArray) -> None:
        if len(msg.data) != 12:
            self.get_logger().warn('HSV threshold update must contain 12 integers')
            return
        values = [int(value) for value in msg.data]
        hue_indices = {0, 3, 6, 9}
        for index, value in enumerate(values):
            maximum = 180 if index in hue_indices else 255
            values[index] = max(0, min(maximum, value))
        self.thresholds = values
        self._publish_thresholds()

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = preprocess_lane_frame(
                bgr,
                roi_top_ratio=float(self.get_parameter('roi_top_ratio').value),
                bev_width=int(self.get_parameter('bev_width').value),
                bev_height=int(self.get_parameter('bev_height').value),
                bev_source=self.get_parameter('bev_source').value,
                bev_destination=self.get_parameter('bev_destination').value,
                white_lower=self.thresholds[0:3],
                white_upper=self.thresholds[3:6],
                yellow_lower=self.thresholds[6:9],
                yellow_upper=self.thresholds[9:12],
                morphology_kernel=int(
                    self.get_parameter('morphology_kernel').value),
            )
        except (ValueError, cv2.error) as error:
            self.get_logger().error(f'lane detection failed: {error}')
            return

        bev_msg = self.bridge.cv2_to_imgmsg(result.bev, encoding='bgr8')
        bev_msg.header = msg.header
        self.bev_pub.publish(bev_msg)
        binary_msg = self.bridge.cv2_to_imgmsg(result.binary, encoding='mono8')
        binary_msg.header = msg.header
        self.binary_pub.publish(binary_msg)
        if bool(self.get_parameter('publish_individual_masks').value):
            white_msg = self.bridge.cv2_to_imgmsg(
                result.white_mask, encoding='mono8')
            yellow_msg = self.bridge.cv2_to_imgmsg(
                result.yellow_mask, encoding='mono8')
            white_msg.header = msg.header
            yellow_msg.header = msg.header
            self.white_pub.publish(white_msg)
            self.yellow_pub.publish(yellow_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanePreprocessorNode()
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
