#!/usr/bin/env python3
"""Fuse YOLO lane ROIs with an OpenCV BEV mask to generate a centreline path."""

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class LinePlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('line_planner_node')
        self.declare_parameter('mask_topic', '/lane_detection/binary_mask')
        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('fused_mask_topic', '/planner/fused_mask')
        self.declare_parameter('lookahead_start_ratio', 0.55)
        self.declare_parameter('sample_count', 18)
        self.declare_parameter('lane_width_ratio', 0.45)
        self.declare_parameter('yolo_min_confidence', 0.50)
        self.declare_parameter('yolo_sync_tolerance_sec', 0.35)
        self.declare_parameter('yolo_roi_dilation_px', 12)
        self.declare_parameter('lane_classes', ['lane1', 'lane2'])
        self.declare_parameter('src_pts', [.40, .55, .60, .55, .99, .98, .01, .98])
        self.declare_parameter('dst_pts', [.18, .0, .82, .0, .82, 1., .18, 1.])
        self._bridge = CvBridge()
        self._latest_yolo = None
        self._start_ratio = float(self.get_parameter('lookahead_start_ratio').value)
        self._sample_count = int(self.get_parameter('sample_count').value)
        self._lane_width_ratio = float(self.get_parameter('lane_width_ratio').value)
        self._yolo_min_confidence = float(self.get_parameter('yolo_min_confidence').value)
        self._yolo_sync_tolerance = float(self.get_parameter('yolo_sync_tolerance_sec').value)
        self._yolo_roi_dilation = int(self.get_parameter('yolo_roi_dilation_px').value)
        self._lane_classes = set(self.get_parameter('lane_classes').value)
        self._src_pts = self.get_parameter('src_pts').value
        self._dst_pts = self.get_parameter('dst_pts').value
        mask_topic = str(self.get_parameter('mask_topic').value)
        yolo_topic = str(self.get_parameter('yolo_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        fused_mask_topic = str(self.get_parameter('fused_mask_topic').value)
        self.create_subscription(Image, mask_topic, self._on_mask, 1)
        self.create_subscription(Detection2DArray, yolo_topic, self._on_yolo, 1)
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self._fused_mask_pub = self.create_publisher(Image, fused_mask_topic, 10)
        self.get_logger().info(f'Line planner fusion: {mask_topic} + {yolo_topic} -> {path_topic}')

    def _on_yolo(self, message: Detection2DArray) -> None:
        self._latest_yolo = message

    @staticmethod
    def _time_in_seconds(header) -> float:
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def _yolo_roi_in_bev(self, header, width, height):
        if self._latest_yolo is None:
            return None
        if abs(self._time_in_seconds(header) - self._time_in_seconds(self._latest_yolo.header)) > self._yolo_sync_tolerance:
            return None

        raw_roi = np.zeros((height, width), dtype=np.uint8)
        for detection in self._latest_yolo.detections:
            if not detection.results:
                continue
            hypothesis = detection.results[0].hypothesis
            if hypothesis.class_id not in self._lane_classes or hypothesis.score < self._yolo_min_confidence:
                continue
            box = detection.bbox
            x1 = int(np.clip(box.center.position.x - box.size_x / 2.0, 0, width - 1))
            y1 = int(np.clip(box.center.position.y - box.size_y / 2.0, 0, height - 1))
            x2 = int(np.clip(box.center.position.x + box.size_x / 2.0, 0, width - 1))
            y2 = int(np.clip(box.center.position.y + box.size_y / 2.0, 0, height - 1))
            cv2.rectangle(raw_roi, (x1, y1), (x2, y2), 255, thickness=-1)

        if not np.any(raw_roi):
            return None
        src = np.float32(self._src_pts).reshape(4, 2) * np.float32([width, height])
        dst = np.float32(self._dst_pts).reshape(4, 2) * np.float32([width, height])
        matrix = cv2.getPerspectiveTransform(src, dst)
        roi_bev = cv2.warpPerspective(raw_roi, matrix, (width, height))
        kernel_size = max(1, self._yolo_roi_dilation * 2 + 1)
        return cv2.dilate(roi_bev, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)

    def _fuse_mask(self, mask, header):
        yolo_roi = self._yolo_roi_in_bev(header, mask.shape[1], mask.shape[0])
        if yolo_roi is None:
            return mask
        yolo_supported = cv2.bitwise_and(mask, yolo_roi)
        # Keep OpenCV-only operation if the latest YOLO box does not overlap the
        # BEV lane pixels enough; this avoids a stale/misaligned box erasing lanes.
        minimum_pixels = max(30, int(np.count_nonzero(mask) * 0.05))
        return yolo_supported if np.count_nonzero(yolo_supported) >= minimum_pixels else mask

    def _on_mask(self, message: Image) -> None:
        try:
            mask = self._bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
        except Exception as error:
            self.get_logger().warning(f'Mask conversion failed: {error}')
            return

        fused_mask = self._fuse_mask(mask, message.header)
        fused_message = self._bridge.cv2_to_imgmsg(fused_mask, encoding='mono8')
        fused_message.header = message.header
        self._fused_mask_pub.publish(fused_message)

        height, width = fused_mask.shape[:2]
        rows = np.linspace(int(height * self._start_ratio), height - 1, self._sample_count).astype(int)
        half_lane_width = width * self._lane_width_ratio * 0.5
        centre_points = []

        for row in rows:
            band = fused_mask[max(0, row - 2):min(height, row + 3)]
            xs = np.where(band > 0)[1]
            left = xs[xs < width / 2]
            right = xs[xs >= width / 2]
            if left.size and right.size:
                centre_x = (np.median(left) + np.median(right)) * 0.5
            elif left.size:
                centre_x = np.median(left) + half_lane_width
            elif right.size:
                centre_x = np.median(right) - half_lane_width
            else:
                continue
            centre_points.append((float(np.clip(centre_x, 0, width - 1)) / width, float(row) / height))

        if len(centre_points) < 3:
            return

        # A polynomial centreline prevents isolated mask noise from becoming a
        # visible steering kink in the tracking path.
        point_array = np.asarray(centre_points)
        degree = min(2, len(point_array) - 1)
        smooth_fit = np.polyfit(point_array[:, 1], point_array[:, 0], degree)
        centre_points = [
            (float(np.clip(np.polyval(smooth_fit, y), 0.0, 1.0)), y)
            for _, y in centre_points
        ]

        path = Path()
        path.header = message.header
        path.header.frame_id = 'bev_normalized'
        for x, y in centre_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LinePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
