#!/usr/bin/env python3
"""ROS2 Self-Driving Debug & Visualization HUD Node.

Subscribes to camera frames, YOLO object detections, lane detection debug images,
steering error topics, and vehicle cmd_vel commands to render a 4-split
real-time HUD (Heads-Up Display) window and publish a composite ROS2 debug image topic.
"""

import time
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray


class DebugNode(Node):
    """Integrates vision, lane, YOLO, and vehicle telemetry into a 4-split HUD GUI."""

    def __init__(self) -> None:
        super().__init__('debug_node')

        # Declare parameters for topics and visualization settings
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.declare_parameter('lane_debug_topic', '/lane_detection/debug_image')
        self.declare_parameter('lane_mask_topic', '/lane_detection/binary_mask')
        self.declare_parameter('steering_error_topic', '/lane_detection/steering_error')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('planned_path_topic', '/planned_path')
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_composite', True)
        self.declare_parameter('composite_topic', '/debug/composite_image')

        # Read parameter values
        image_topic = str(self.get_parameter('image_topic').value)
        yolo_topic = str(self.get_parameter('yolo_topic').value)
        lane_debug_topic = str(self.get_parameter('lane_debug_topic').value)
        lane_mask_topic = str(self.get_parameter('lane_mask_topic').value)
        steering_error_topic = str(self.get_parameter('steering_error_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        planned_path_topic = str(self.get_parameter('planned_path_topic').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._publish_composite = bool(self.get_parameter('publish_composite').value)
        composite_topic = str(self.get_parameter('composite_topic').value)

        self._bridge = CvBridge()

        # Telemetry & State variables
        self._latest_cam_frame = None
        self._latest_yolo_detections = None
        self._latest_lane_debug = None
        self._latest_lane_mask = None
        self._latest_steering_error = 0.0
        self._latest_cmd_vel = Twist()
        self._latest_planned_path = None
        self._frame_count = 0
        self._fps = 0.0
        self._last_fps_time = time.time()

        # Subscriptions
        self.create_subscription(Image, image_topic, self._cb_cam_image, 1)
        self.create_subscription(Detection2DArray, yolo_topic, self._cb_yolo_detections, 1)
        self.create_subscription(Image, lane_debug_topic, self._cb_lane_debug, 1)
        self.create_subscription(Image, lane_mask_topic, self._cb_lane_mask, 1)
        self.create_subscription(Float32, steering_error_topic, self._cb_steering_error, 1)
        self.create_subscription(Twist, cmd_vel_topic, self._cb_cmd_vel, 1)
        self.create_subscription(Path, planned_path_topic, self._cb_planned_path, 1)

        # Publisher for composite debug HUD
        if self._publish_composite:
            self._composite_pub = self.create_publisher(Image, composite_topic, 1)

        # Timer for HUD rendering (30 FPS -> 0.033s)
        self.create_timer(0.033, self._render_hud)

        self.get_logger().info(
            f'Debug Visualization Node initialized. Subscribed to: {image_topic}, {yolo_topic}, {lane_debug_topic}'
        )

    def _cb_cam_image(self, msg: Image) -> None:
        try:
            self._latest_cam_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().error(f'Camera image conversion error: {err}')

    def _cb_yolo_detections(self, msg: Detection2DArray) -> None:
        self._latest_yolo_detections = msg

    def _cb_lane_debug(self, msg: Image) -> None:
        try:
            self._latest_lane_debug = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as err:
            self.get_logger().error(f'Lane debug image conversion error: {err}')

    def _cb_lane_mask(self, msg: Image) -> None:
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            self._latest_lane_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        except Exception as err:
            self.get_logger().error(f'Lane mask conversion error: {err}')

    def _cb_steering_error(self, msg: Float32) -> None:
        self._latest_steering_error = float(msg.data)

    def _cb_cmd_vel(self, msg: Twist) -> None:
        self._latest_cmd_vel = msg

    def _cb_planned_path(self, msg: Path) -> None:
        self._latest_planned_path = msg

    def _render_hud(self) -> None:
        # FPS calculation
        self._frame_count += 1
        now = time.time()
        if now - self._last_fps_time >= 1.0:
            self._fps = self._frame_count / (now - self._last_fps_time)
            self._frame_count = 0
            self._last_fps_time = now

        target_w, target_h = 640, 360

        # --- Tile 1: Camera + YOLO Bounding Boxes ---
        tile1 = (
            self._latest_cam_frame.copy()
            if self._latest_cam_frame is not None
            else np.zeros((target_h, target_w, 3), dtype=np.uint8)
        )
        tile1 = cv2.resize(tile1, (target_w, target_h))

        if self._latest_cam_frame is not None and self._latest_yolo_detections is not None:
            orig_h, orig_w = self._latest_cam_frame.shape[:2]
            scale_x = target_w / float(orig_w)
            scale_y = target_h / float(orig_h)

            for det in self._latest_yolo_detections.detections:
                cx = det.bbox.center.position.x * scale_x
                cy = det.bbox.center.position.y * scale_y
                w = det.bbox.size_x * scale_x
                h = det.bbox.size_y * scale_y

                x1, y1 = int(cx - w / 2), int(cy - h / 2)
                x2, y2 = int(cx + w / 2), int(cy + h / 2)

                label_text = "Object"
                score = 0.0
                if len(det.results) > 0:
                    hyp = det.results[0]
                    # Support both nested and flattened vision_msgs structures
                    class_id = getattr(hyp, 'hypothesis', hyp).class_id
                    score = getattr(hyp, 'hypothesis', hyp).score
                    label_text = f"{class_id}: {score:.2f}"

                # Draw green bounding box & label
                cv2.rectangle(tile1, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(tile1, (x1, max(0, y1 - 20)), (x1 + len(label_text) * 10, y1), (0, 255, 0), -1)
                cv2.putText(
                    tile1, label_text, (x1 + 2, max(12, y1 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA
                )

        cv2.putText(tile1, "[1] Camera + YOLO Detection", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Tile 2: Lane Binary Mask ---
        tile2 = (
            cv2.resize(self._latest_lane_mask, (target_w, target_h))
            if self._latest_lane_mask is not None
            else np.zeros((target_h, target_w, 3), dtype=np.uint8)
        )
        cv2.putText(tile2, "[2] Lane Color Binary Mask", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- Tile 3: Lane BEV & Polynomial Debug View ---
        tile3 = (
            cv2.resize(self._latest_lane_debug, (target_w, target_h))
            if self._latest_lane_debug is not None
            else np.zeros((target_h, target_w, 3), dtype=np.uint8)
        )
        cv2.putText(tile3, "[3] Lane Bird's Eye View & Fitting", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if self._latest_planned_path is not None and len(self._latest_planned_path.poses) >= 2:
            path_points = np.array([
                [int(pose.pose.position.x * target_w), int(pose.pose.position.y * target_h)]
                for pose in self._latest_planned_path.poses
            ], dtype=np.int32)
            cv2.polylines(tile3, [path_points], False, (255, 255, 0), 4, cv2.LINE_AA)
            cv2.putText(tile3, "Tracking Path", (10, target_h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        # --- Tile 4: Telemetry HUD & Status Panel ---
        tile4 = np.full((target_h, target_w, 3), 30, dtype=np.uint8)
        cv2.putText(tile4, "[4] Telemetry & Vehicle Control HUD", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # System Metrics
        cv2.putText(tile4, f"FPS: {self._fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Steering Error Gauge Bar
        err = self._latest_steering_error
        cv2.putText(tile4, f"Steering Error: {err:+.3f}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        center_x = 320
        bar_y = 115
        cv2.rectangle(tile4, (center_x - 150, bar_y), (center_x + 150, bar_y + 15), (70, 70, 70), -1)
        cv2.line(tile4, (center_x, bar_y - 3), (center_x, bar_y + 18), (255, 255, 255), 2)
        err_offset = int(np.clip(err * 100, -150, 150))
        bar_color = (0, 255, 0) if abs(err) < 0.2 else ((0, 255, 255) if abs(err) < 0.5 else (0, 0, 255))
        if err_offset >= 0:
            cv2.rectangle(tile4, (center_x, bar_y), (center_x + err_offset, bar_y + 15), bar_color, -1)
        else:
            cv2.rectangle(tile4, (center_x + err_offset, bar_y), (center_x, bar_y + 15), bar_color, -1)

        # Cmd_Vel Telemetry
        v_x = self._latest_cmd_vel.linear.x
        w_z = self._latest_cmd_vel.angular.z
        cv2.putText(tile4, f"Linear Speed (v_x): {v_x:.2f} m/s", (10, 165), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)
        cv2.putText(tile4, f"Angular Vel (w_z):  {w_z:.2f} rad/s", (10, 195), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 1)

        # YOLO Detection Summary
        num_det = len(self._latest_yolo_detections.detections) if self._latest_yolo_detections is not None else 0
        cv2.putText(tile4, f"YOLO Detected Objects: {num_det}", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 100), 1)

        # Status Badge
        status_text = "STATUS: TRACKING ACTIVE" if self._latest_cam_frame is not None else "STATUS: WAITING FOR CAMERA"
        badge_color = (0, 200, 0) if self._latest_cam_frame is not None else (0, 0, 200)
        cv2.rectangle(tile4, (10, 280), (350, 320), badge_color, -1)
        cv2.putText(tile4, status_text, (20, 307), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # --- Combine 4 Tiles Into 2x2 Grid (1280 x 720) ---
        top_row = np.hstack((tile1, tile2))
        bottom_row = np.hstack((tile3, tile4))
        composite = np.vstack((top_row, bottom_row))

        # Publish composite ROS2 image topic
        if self._publish_composite:
            try:
                comp_msg = self._bridge.cv2_to_imgmsg(composite, encoding='bgr8')
                self._composite_pub.publish(comp_msg)
            except Exception as err:
                self.get_logger().error(f'Composite image publish error: {err}')

        # Show GUI Window if enabled
        if self._show_window:
            try:
                cv2.imshow("ROS2 Autonomous Vehicle Debug HUD", composite)
                cv2.waitKey(1)
            except Exception as err:
                self.get_logger().warn(f'OpenCV imshow GUI disabled or unavailable: {err}')
                self._show_window = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DebugNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
