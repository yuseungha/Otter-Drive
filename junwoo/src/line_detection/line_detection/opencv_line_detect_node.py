#!/usr/bin/env python3
"""
OpenCV Line Detection Node (BEV First -> Masking Pipeline)
ROS 2 Humble Node for Autonomous Driving Line Detection
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge


class OpenCVLineDetectNode(Node):
    def __init__(self):
        super().__init__('opencv_line_detect_node')

        # Declare ROS 2 Parameters for BEV (Bird's Eye View / IPM) Transformation
        self.declare_parameter('bev_width', 640)
        self.declare_parameter('bev_height', 480)
        
        # Source ROI Points for Perspective Transform [Top-Left, Top-Right, Bottom-Right, Bottom-Left] (Normalized 0.0 ~ 1.0)
        self.declare_parameter('src_top_left', [0.35, 0.55])
        self.declare_parameter('src_top_right', [0.65, 0.55])
        self.declare_parameter('src_bottom_right', [0.95, 0.95])
        self.declare_parameter('src_bottom_left', [0.05, 0.95])

        # HSV Masking Parameters for Yellow & White Lane Lines
        self.declare_parameter('yellow_hsv_low', [15, 70, 70])
        self.declare_parameter('yellow_hsv_high', [35, 255, 255])
        self.declare_parameter('white_hsv_low', [0, 0, 180])
        self.declare_parameter('white_hsv_high', [180, 50, 255])

        # Get Parameters
        self.bev_w = self.get_parameter('bev_width').value
        self.bev_h = self.get_parameter('bev_height').value
        
        self.src_tl = self.get_parameter('src_top_left').value
        self.src_tr = self.get_parameter('src_top_right').value
        self.src_br = self.get_parameter('src_bottom_right').value
        self.src_bl = self.get_parameter('src_bottom_left').value

        self.yellow_low = np.array(self.get_parameter('yellow_hsv_low').value, dtype=np.uint8)
        self.yellow_high = np.array(self.get_parameter('yellow_hsv_high').value, dtype=np.uint8)
        self.white_low = np.array(self.get_parameter('white_hsv_low').value, dtype=np.uint8)
        self.white_high = np.array(self.get_parameter('white_hsv_high').value, dtype=np.uint8)

        # CvBridge Initialization
        self.bridge = CvBridge()

        # Subscribers & Publishers
        self.sub_image = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.pub_line_poses = self.create_publisher(
            PoseArray,
            '/opencv/line_detections',
            10
        )

        self.pub_bev_image = self.create_publisher(
            Image,
            '/opencv/bev_image',
            10
        )

        self.pub_masked_image = self.create_publisher(
            Image,
            '/opencv/masked_image',
            10
        )

        self.get_logger().info('OpenCV Line Detect Node initialized with BEV -> Masking Pipeline!')

    def compute_bev_transform(self, img_w, img_h):
        """Calculate Perspective Transformation Matrix M and Inverse M"""
        src_pts = np.float32([
            [self.src_tl[0] * img_w, self.src_tl[1] * img_h],
            [self.src_tr[0] * img_w, self.src_tr[1] * img_h],
            [self.src_br[0] * img_w, self.src_br[1] * img_h],
            [self.src_bl[0] * img_w, self.src_bl[1] * img_h]
        ])

        dst_pts = np.float32([
            [0, 0],
            [self.bev_w, 0],
            [self.bev_w, self.bev_h],
            [0, self.bev_h]
        ])

        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        M_inv = cv2.getPerspectiveTransform(dst_pts, src_pts)
        return M, M_inv

    def process_bev_and_masking(self, frame):
        """1단계: BEV(Bird's Eye View) 변환 -> 2단계: 색상 & 에지 마스킹"""
        img_h, img_w = frame.shape[:2]

        # STEP 1: BEV (Bird's Eye View / Inverse Perspective Mapping) Transform First
        M, _ = self.compute_bev_transform(img_w, img_h)
        bev_img = cv2.warpPerspective(frame, M, (self.bev_w, self.bev_h))

        # STEP 2: Masking after BEV Transformation
        # Convert BEV Image to HSV color space
        hsv_bev = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)

        # Create HSV Color Masks (Yellow Lane + White Lane)
        mask_yellow = cv2.inRange(hsv_bev, self.yellow_low, self.yellow_high)
        mask_white = cv2.inRange(hsv_bev, self.white_low, self.white_high)
        color_mask = cv2.bitwise_or(mask_yellow, mask_white)

        # Apply Canny Edge Detection on BEV image
        gray_bev = cv2.cvtColor(bev_img, cv2.COLOR_BGR2GRAY)
        blur_bev = cv2.GaussianBlur(gray_bev, (5, 5), 0)
        edges_bev = cv2.Canny(blur_bev, 50, 150)

        # Combined Mask: (Color Mask) AND (Edges) + Morphological Filtering
        combined_mask = cv2.bitwise_and(color_mask, color_mask, mask=edges_bev)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        cleaned_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel)

        return bev_img, cleaned_mask

    def extract_lines_from_mask(self, mask):
        """3단계: 마스킹된 BEV 이미지에서 라인 포인트 추출 (Hough / Sliding Window)"""
        lines = cv2.HoughLinesP(
            mask,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=20,
            maxLineGap=15
        )

        detected_poses = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Convert BEV pixel coordinates to normalized pose points
                p1 = Pose()
                p1.position.x = float(x1) / self.bev_w
                p1.position.y = float(y1) / self.bev_h
                p1.position.z = 0.0

                p2 = Pose()
                p2.position.x = float(x2) / self.bev_w
                p2.position.y = float(y2) / self.bev_h
                p2.position.z = 0.0

                detected_poses.extend([p1, p2])

        return detected_poses

    def image_callback(self, msg):
        try:
            # ROS 2 Image -> OpenCV BGR
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion error: {e}')
            return

        # Execute BEV -> Masking Pipeline
        bev_img, masked_img = self.process_bev_and_masking(frame)

        # Extract line points
        line_poses = self.extract_lines_from_mask(masked_img)

        # Publish PoseArray
        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        pose_array_msg.header.frame_id = 'bev_frame'
        pose_array_msg.poses = line_poses
        self.pub_line_poses.publish(pose_array_msg)

        # Publish Debug BEV & Masked Images
        try:
            bev_msg = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
            bev_msg.header = msg.header
            self.pub_bev_image.publish(bev_msg)

            masked_msg = self.bridge.cv2_to_imgmsg(masked_img, encoding='mono8')
            masked_msg.header = msg.header
            self.pub_masked_image.publish(masked_msg)
        except Exception as e:
            self.get_logger().warn(f'Error publishing debug images: {e}')


from rclpy.executors import ExternalShutdownException


def main(args=None):
    rclpy.init(args=args)
    node = OpenCVLineDetectNode()
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
