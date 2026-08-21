#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mission Waypoint Follower Node for 1/10 RC Autonomous Vehicle
Executes a 6-phase FSM sequence:
  Phase 1: Forward Pure Pursuit (WP 1 -> 27)
  Phase 2: Reverse Pure Pursuit Mission A (WP 28 -> 33) -> 3.0s Parking Wait
  Phase 3: Forward Pure Pursuit (WP 34 -> 45)
  Phase 4: Reverse Pure Pursuit Mission B (WP 46 -> 51) -> 3.0s Parking Wait
  Phase 5: Forward Pure Pursuit (WP 52 -> 79)
  Phase 6: Forward Return to Start (WP 79 -> WP 1) -> Complete & Shutdown
"""

import os
import sys
import math
import csv
import enum
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, Image
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Optional imports for Xycar / Ackermann
try:
    from xycar_msgs.msg import XycarMotor, XycarUltrasonic
    HAS_XYCAR_MSGS = True
except ImportError:
    HAS_XYCAR_MSGS = False

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False


class FSMState(enum.Enum):
    PHASE_1_FWD = 1       # Waypoints 1 -> 27 (Forward)
    PHASE_2_REV_A = 2     # Waypoints 28 -> 33 (Reverse Parking A)
    PHASE_2_ALIGN_A = 3   # Mission A: Alignment (Horizontal 가로 정렬 & Center on WP 33)
    PHASE_2_WAIT = 4      # Mission A: 3.0s Parking Wait (AFTER alignment complete)
    PHASE_3_FWD = 5       # Waypoints 34 -> 55 (Forward)
    PHASE_4_REV_B = 6     # Waypoints 56 -> 61 (Reverse Parking B)
    PHASE_4_ALIGN_B = 7   # Mission B: Alignment (Vertical 세로 정렬 & Center on WP 61)
    PHASE_4_WAIT = 8      # Mission B: 3.0s Parking Wait (AFTER alignment complete)
    PHASE_5_FWD = 9       # Waypoints 62 -> 79 (Forward)
    PHASE_6_RETURN = 10   # Towards WP 1 (Return to Start)
    MISSION_COMPLETE = 11 # Full stop & complete


class Waypoint:
    def __init__(self, index: int, x: float, y: float, yaw: float, speed: float):
        self.index = index
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed


class MissionWaypointFollower(Node):
    def __init__(self):
        super().__init__('mission_waypoint_follower')

        # -----------------------------------------------------------
        # Parameters Declaration
        # -----------------------------------------------------------
        self.declare_parameter('waypoints_csv_path', 'waypoints.csv')
        self.declare_parameter('wheelbase', 0.315)
        self.declare_parameter('vehicle_length', 0.530)
        self.declare_parameter('vehicle_width', 0.260)
        self.declare_parameter('vehicle_height', 0.220)
        self.declare_parameter('max_steer_rad', 0.610865)
        self.declare_parameter('xycar_max_steer_command', 650)
        self.declare_parameter('xycar_max_speed_command', 650)
        self.declare_parameter('xycar_steer_scale', 0.000939793)
        self.declare_parameter('xycar_speed_scale', 0.000230769)

        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('reverse_speed', -0.15)
        self.declare_parameter('forward_lookahead', 0.50)
        self.declare_parameter('reverse_lookahead', 0.40)

        self.declare_parameter('waypoint_tolerance', 0.22)
        self.declare_parameter('parking_tolerance', 0.10)
        self.declare_parameter('parking_yaw_tolerance', 0.10)
        self.declare_parameter('start_tolerance', 0.10)
        self.declare_parameter('parking_a_tolerance', 0.10)
        self.declare_parameter('parking_a_yaw_tolerance', 0.10)
        self.declare_parameter('parking_b_tolerance', 0.10)
        self.declare_parameter('parking_b_yaw_tolerance', 0.10)
        self.declare_parameter('parking_wait_sec', 3.0)
        self.declare_parameter('phase1_overshoot_dist', 0.30)
        self.declare_parameter('phase3_overshoot_dist', 0.50)
        self.declare_parameter('start_pose_x', 1.80)
        self.declare_parameter('start_pose_y', 0.90)
        self.declare_parameter('start_pose_yaw', 3.141592)
        self.declare_parameter('start_box_length', 0.70)
        self.declare_parameter('start_box_width', 0.30)

        self.declare_parameter('parking_a_center_x', 0.00)
        self.declare_parameter('parking_a_center_y', 4.20)
        self.declare_parameter('parking_a_target_yaw', 0.00)

        self.declare_parameter('parking_b_center_x', 2.10)
        self.declare_parameter('parking_b_center_y', 3.30)
        self.declare_parameter('parking_b_target_yaw', -1.5708)
        self.declare_parameter('heading_align_dist', 0.40)

        self.declare_parameter('obstacle_stop_dist', 0.30)
        self.declare_parameter('lidar_front_min_deg', -35.0)
        self.declare_parameter('lidar_front_max_deg', 35.0)
        self.declare_parameter('lidar_rear_min_deg', 145.0)
        self.declare_parameter('lidar_rear_max_deg', 215.0)

        self.declare_parameter('enable_camera_cone_slowdown', True)
        self.declare_parameter('cone_slowdown_scale', 0.50)

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_mode', 'xycar_motor')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', '/usb_cam/image_raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('loop_rate_hz', 20.0)

        # Retrieve Parameters
        self.csv_path = self.get_parameter('waypoints_csv_path').get_parameter_value().string_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.vehicle_length = self.get_parameter('vehicle_length').get_parameter_value().double_value
        self.vehicle_width = self.get_parameter('vehicle_width').get_parameter_value().double_value
        self.vehicle_height = self.get_parameter('vehicle_height').get_parameter_value().double_value
        self.max_steer_rad = self.get_parameter('max_steer_rad').get_parameter_value().double_value
        self.xycar_max_steer_command = self.get_parameter('xycar_max_steer_command').get_parameter_value().integer_value
        self.xycar_max_speed_command = self.get_parameter('xycar_max_speed_command').get_parameter_value().integer_value
        self.xycar_steer_scale = self.get_parameter('xycar_steer_scale').get_parameter_value().double_value
        self.xycar_speed_scale = self.get_parameter('xycar_speed_scale').get_parameter_value().double_value

        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.reverse_speed = self.get_parameter('reverse_speed').get_parameter_value().double_value
        self.forward_lookahead = self.get_parameter('forward_lookahead').get_parameter_value().double_value
        self.reverse_lookahead = self.get_parameter('reverse_lookahead').get_parameter_value().double_value

        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.parking_tolerance = self.get_parameter('parking_tolerance').get_parameter_value().double_value
        self.parking_yaw_tolerance = self.get_parameter('parking_yaw_tolerance').get_parameter_value().double_value
        self.start_tolerance = self.get_parameter('start_tolerance').get_parameter_value().double_value
        self.parking_a_tolerance = self.get_parameter('parking_a_tolerance').get_parameter_value().double_value
        self.parking_a_yaw_tolerance = self.get_parameter('parking_a_yaw_tolerance').get_parameter_value().double_value
        self.parking_b_tolerance = self.get_parameter('parking_b_tolerance').get_parameter_value().double_value
        self.parking_b_yaw_tolerance = self.get_parameter('parking_b_yaw_tolerance').get_parameter_value().double_value
        self.parking_wait_sec = self.get_parameter('parking_wait_sec').get_parameter_value().double_value
        self.phase1_overshoot = self.get_parameter('phase1_overshoot_dist').get_parameter_value().double_value
        self.phase3_overshoot = self.get_parameter('phase3_overshoot_dist').get_parameter_value().double_value
        
        self.start_x = self.get_parameter('start_pose_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_pose_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_pose_yaw').get_parameter_value().double_value
        self.start_box_length = self.get_parameter('start_box_length').get_parameter_value().double_value
        self.start_box_width = self.get_parameter('start_box_width').get_parameter_value().double_value

        self.parking_a_x = self.get_parameter('parking_a_center_x').get_parameter_value().double_value
        self.parking_a_y = self.get_parameter('parking_a_center_y').get_parameter_value().double_value
        self.parking_a_yaw = self.get_parameter('parking_a_target_yaw').get_parameter_value().double_value

        self.parking_b_x = self.get_parameter('parking_b_center_x').get_parameter_value().double_value
        self.parking_b_y = self.get_parameter('parking_b_center_y').get_parameter_value().double_value
        self.parking_b_yaw = self.get_parameter('parking_b_target_yaw').get_parameter_value().double_value

        self.heading_align_dist = self.get_parameter('heading_align_dist').get_parameter_value().double_value

        self.phase1_overshoot_active = False
        self.phase1_overshoot_start = None
        self.phase3_overshoot_active = False
        self.phase3_overshoot_start = None

        self.obstacle_stop_dist = self.get_parameter('obstacle_stop_dist').get_parameter_value().double_value
        self.lidar_front_min = math.radians(self.get_parameter('lidar_front_min_deg').get_parameter_value().double_value)
        self.lidar_front_max = math.radians(self.get_parameter('lidar_front_max_deg').get_parameter_value().double_value)
        self.lidar_rear_min = math.radians(self.get_parameter('lidar_rear_min_deg').get_parameter_value().double_value)
        self.lidar_rear_max = math.radians(self.get_parameter('lidar_rear_max_deg').get_parameter_value().double_value)

        self.enable_cone_slowdown = self.get_parameter('enable_camera_cone_slowdown').get_parameter_value().bool_value
        self.cone_slowdown_scale = self.get_parameter('cone_slowdown_scale').get_parameter_value().double_value

        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value

        self.motor_topic = self.get_parameter('motor_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        loop_rate = self.get_parameter('loop_rate_hz').get_parameter_value().double_value

        # -----------------------------------------------------------
        # Data & State Machine Initialization
        # -----------------------------------------------------------
        self.waypoints = {}  # index -> Waypoint
        self.ordered_indices = []
        self.load_waypoints()

        self.state = FSMState.PHASE_1_FWD
        self.state_start_time = self.get_clock().now()
        self.wait_start_time = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_received = False

        self.emergency_stop = False
        self.obstacle_distance_front = 999.0
        self.obstacle_distance_rear = 999.0
        self.cone_detected = False

        if HAS_OPENCV:
            self.cv_bridge = CvBridge()

        # -----------------------------------------------------------
        # TF2 Setup
        # -----------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -----------------------------------------------------------
        # ROS 2 Publishers & Subscribers
        # -----------------------------------------------------------
        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )

        self.path_pub = self.create_publisher(Path, '/mission_path', latched_qos)
        self.active_path_pub = self.create_publisher(Path, '/active_segment_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        if HAS_XYCAR_MSGS:
            self.motor_pub = self.create_publisher(XycarMotor, self.motor_topic, 10)

        # Subscriptions
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        if HAS_OPENCV:
            self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        # Publish global path once
        self.publish_global_path()

        # Main Control Timer
        self.timer = self.create_timer(1.0 / loop_rate, self.control_loop)
        self.get_logger().info("MissionWaypointFollower initialized. Ready to execute 6-phase mission.")

    # ===============================================================
    # Waypoints Loader & Global Path
    # ===============================================================
    def load_waypoints(self):
        path = self.csv_path
        if not os.path.isabs(path):
            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            for cand in [os.path.join(os.getcwd(), path), os.path.join(project_root, path), path]:
                if os.path.exists(cand):
                    path = cand
                    break

        if not os.path.exists(path):
            self.get_logger().error(f"Waypoints CSV file not found: {path}")
            return

        with open(path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    idx = int(row['index'])
                    x = float(row['x'])
                    y = float(row['y'])
                    yaw = float(row['yaw'])
                    speed = float(row['speed']) if 'speed' in row else 0.5
                    self.waypoints[idx] = Waypoint(idx, x, y, yaw, speed)
                    self.ordered_indices.append(idx)
                except Exception as e:
                    self.get_logger().warn(f"Failed to parse row: {row}, error: {e}")

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {path}")

    def publish_global_path(self):
        if not self.waypoints:
            return

        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for idx in sorted(self.waypoints.keys()):
            wp = self.waypoints[idx]
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            # Quaternion from yaw
            pose.pose.orientation.z = math.sin(wp.yaw / 2.0)
            pose.pose.orientation.w = math.cos(wp.yaw / 2.0)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    # ===============================================================
    # Localization / TF Updates
    # ===============================================================
    def update_pose(self):
        try:
            # Lookup TF from global_frame to base_frame
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
            self.current_x = t.transform.translation.x
            self.current_y = t.transform.translation.y

            # Extract yaw from quaternion
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.pose_received = True
        except Exception:
            pass

    def odom_callback(self, msg: Odometry):
        if not self.pose_received:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.current_yaw = 2.0 * math.atan2(qz, qw)
            self.pose_received = True

    # ===============================================================
    # Perception / Safety Callbacks
    # ===============================================================
    def scan_callback(self, msg: LaserScan):
        min_dist_front = 999.0
        min_dist_rear = 999.0

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                # Normalized angle [-pi, pi]
                norm_angle = math.atan2(math.sin(angle), math.cos(angle))

                # Front sector check
                if self.lidar_front_min <= norm_angle <= self.lidar_front_max:
                    if r < min_dist_front:
                        min_dist_front = r

                # Rear sector check
                if (norm_angle >= self.lidar_rear_min) or (norm_angle <= -self.lidar_rear_min):
                    if r < min_dist_rear:
                        min_dist_rear = r

            angle += msg.angle_increment

        self.obstacle_distance_front = min_dist_front
        self.obstacle_distance_rear = min_dist_rear

    def image_callback(self, msg: Image):
        if not self.enable_cone_slowdown or not HAS_OPENCV:
            return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            # Focus on lower half ROI (track ahead)
            roi = cv_image[int(h * 0.4):, :]

            # Convert to HSV and segment Orange Cones
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            # Orange color bounds
            lower_orange = np.array([5, 100, 100])
            upper_orange = np.array([25, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            max_area = 0.0
            for c in contours:
                area = cv2.contourArea(c)
                if area > max_area:
                    max_area = area

            # If significant orange cone area detected nearby
            self.cone_detected = (max_area > 800.0)
        except Exception as e:
            self.cone_detected = False

    # ===============================================================
    # Pure Pursuit Controllers
    # ===============================================================
    def compute_forward_pure_pursuit(self, target_slice):
        """
        Computes forward Ackermann steering angle for target slice.
        Advances along the ordered waypoints from the closest waypoint.
        """
        if not target_slice:
            return 0.0, None

        rx, ry, ryaw = self.current_x, self.current_y, self.current_yaw

        # 1. Find closest waypoint in slice
        min_idx = target_slice[0]
        min_dist = float('inf')
        for idx in target_slice:
            wp = self.waypoints.get(idx)
            if wp is not None:
                d = math.hypot(wp.x - rx, wp.y - ry)
                if d < min_dist:
                    min_dist = d
                    min_idx = idx

        # 2. Advance forward along slice from closest index to find lookahead point
        start_pos = target_slice.index(min_idx)
        lookahead_pt = None

        for idx in target_slice[start_pos:]:
            wp = self.waypoints.get(idx)
            if wp is None:
                continue
            dist = math.hypot(wp.x - rx, wp.y - ry)
            dx = wp.x - rx
            dy = wp.y - ry
            local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy

            if dist >= self.forward_lookahead and local_x > 0.0:
                lookahead_pt = (wp.x, wp.y, local_x, local_y, dist)
                break

        # Fallback to endpoint if no point >= lookahead distance
        if lookahead_pt is None:
            last_wp = self.waypoints[target_slice[-1]]
            dx = last_wp.x - rx
            dy = last_wp.y - ry
            local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            lookahead_pt = (last_wp.x, last_wp.y, local_x, local_y, math.hypot(dx, dy))

        _, _, lx, ly, ld = lookahead_pt
        ld_eff = max(ld, 0.20)

        # Alpha: angle to lookahead point in body frame
        alpha = math.atan2(ly, lx)

        # Ackermann steering: delta = atan2(2 * L * sin(alpha), Ld)
        steer = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld_eff)
        steer = np.clip(steer, -self.max_steer_rad, self.max_steer_rad)

        return steer, (lookahead_pt[0], lookahead_pt[1])

    def compute_reverse_pure_pursuit(self, target_slice, target_yaw=None, custom_final_pt=None):
        """
        Computes reverse Ackermann steering angle for target slice.
        Advances along the reverse waypoint sequence from the closest point.
        Optionally targets custom center pose and blends heading alignment.
        """
        if not target_slice:
            return 0.0, None

        rx, ry, ryaw = self.current_x, self.current_y, self.current_yaw

        # 1. Find closest waypoint in slice
        min_idx = target_slice[0]
        min_dist = float('inf')
        for idx in target_slice:
            wp = self.waypoints.get(idx)
            if wp is not None:
                d = math.hypot(wp.x - rx, wp.y - ry)
                if d < min_dist:
                    min_dist = d
                    min_idx = idx

        # 2. Advance forward in sequence towards parking end
        start_pos = target_slice.index(min_idx)
        lookahead_pt = None

        for idx in target_slice[start_pos:]:
            wp = self.waypoints.get(idx)
            if wp is None:
                continue
            dist = math.hypot(wp.x - rx, wp.y - ry)
            dx = wp.x - rx
            dy = wp.y - ry
            local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy

            if dist >= self.reverse_lookahead and local_x < 0.1:
                lookahead_pt = (wp.x, wp.y, local_x, local_y, dist)
                break

        if lookahead_pt is None:
            if custom_final_pt is not None:
                target_x, target_y = custom_final_pt
            else:
                last_wp = self.waypoints[target_slice[-1]]
                target_x, target_y = last_wp.x, last_wp.y
            dx = target_x - rx
            dy = target_y - ry
            local_x = math.cos(ryaw) * dx + math.sin(ryaw) * dy
            local_y = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
            lookahead_pt = (target_x, target_y, local_x, local_y, math.hypot(dx, dy))

        _, _, lx, ly, ld = lookahead_pt
        ld_eff = max(ld, 0.20)

        # When reversing, angle from rear (-x) axis to target
        alpha_rev = math.atan2(ly, -lx)

        # Position-based reverse Ackermann steering
        steer_pos = math.atan2(2.0 * self.wheelbase * math.sin(alpha_rev), ld_eff)

        # Heading alignment correction when nearing final parking spot
        if target_yaw is not None:
            if custom_final_pt is not None:
                final_x, final_y = custom_final_pt
            else:
                last_wp = self.waypoints[target_slice[-1]]
                final_x, final_y = last_wp.x, last_wp.y
            dist_to_final = math.hypot(self.current_x - final_x, self.current_y - final_y)

            yaw_err = math.atan2(
                math.sin(ryaw - target_yaw),
                math.cos(ryaw - target_yaw)
            )
            steer_head = math.atan2(2.5 * self.wheelbase * yaw_err, 0.35)

            if dist_to_final < self.heading_align_dist:
                w = float(np.clip((self.heading_align_dist - dist_to_final) / self.heading_align_dist, 0.0, 0.50))
                steer = (1.0 - w) * steer_pos + w * steer_head
            else:
                steer = steer_pos
        else:
            steer = steer_pos

        steer = np.clip(steer, -self.max_steer_rad, self.max_steer_rad)
        return steer, (lookahead_pt[0], lookahead_pt[1])

    def compute_alignment_control(self, target_x, target_y, target_yaw, center_x, center_y, current_yaw, dist_tol=0.10, yaw_tol=0.10):
        """
        Adaptive bidirectional micro-alignment:
        Ensures both conditions [Center Distance <= dist_tol AND Heading Error <= yaw_tol] are satisfied.
        If center is already inside tolerance but heading is not aligned, executes micro-turns
        to rotate vehicle heading without leaving the tolerance circle.
        """
        dx = target_x - center_x
        dy = target_y - center_y
        dist = math.hypot(dx, dy)
        local_x = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
        local_y = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
        yaw_err = math.atan2(
            math.sin(current_yaw - target_yaw),
            math.cos(current_yaw - target_yaw)
        )

        is_aligned = (abs(yaw_err) <= yaw_tol) and (dist <= dist_tol)
        if is_aligned:
            return 0.0, 0.0, dist, yaw_err, True, "ALIGNED"

        align_speed = 0.15  # Gentle micro-alignment speed (m/s)

        # If already close to center but heading is off, do micro turn maneuvers
        if dist <= dist_tol and abs(yaw_err) > yaw_tol:
            if local_x <= 0.02:
                cmd_speed = align_speed * 0.8
                cmd_steer = float(-np.sign(yaw_err) * self.max_steer_rad)
                direction_str = "MICRO_FWD_TURN"
            else:
                cmd_speed = -align_speed * 0.8
                cmd_steer = float(np.sign(yaw_err) * self.max_steer_rad)
                direction_str = "MICRO_REV_TURN"
        elif local_x > 0.02:
            cmd_speed = align_speed
            raw_steer = math.atan2(-2.5 * self.wheelbase * yaw_err + 1.5 * local_y, 0.35)
            cmd_steer = float(np.clip(raw_steer, -self.max_steer_rad, self.max_steer_rad))
            direction_str = "FORWARD"
        elif local_x < -0.02:
            cmd_speed = -align_speed
            raw_steer = math.atan2(2.5 * self.wheelbase * yaw_err - 1.5 * local_y, 0.35)
            cmd_steer = float(np.clip(raw_steer, -self.max_steer_rad, self.max_steer_rad))
            direction_str = "REVERSE"
        else:
            cmd_speed = align_speed * 0.8
            cmd_steer = float(-np.sign(yaw_err) * self.max_steer_rad)
            direction_str = "MICRO_TURN"

        return cmd_speed, cmd_steer, dist, yaw_err, False, direction_str

    # ===============================================================
    # Main FSM Control Loop
    # ===============================================================
    def control_loop(self):
        self.update_pose()
        if not self.pose_received:
            return

        cmd_speed = 0.0
        cmd_steer = 0.0
        target_pt = None
        status_text = ""
        active_indices = []

        now = self.get_clock().now()

        # Vehicle center point (midpoint of wheelbase)
        center_x = self.current_x + (self.wheelbase / 2.0) * math.cos(self.current_yaw)
        center_y = self.current_y + (self.wheelbase / 2.0) * math.sin(self.current_yaw)

        # -----------------------------------------------------------
        # 1. Phase 1: Forward (WP 1 -> 27) + Forward Clearance Overshoot
        # -----------------------------------------------------------
        if self.state == FSMState.PHASE_1_FWD:
            active_indices = list(range(1, 28))
            dist_to_end = math.hypot(
                self.current_x - self.waypoints[27].x,
                self.current_y - self.waypoints[27].y
            )

            # Trigger overshoot when within tolerance of WP 27
            if not self.phase1_overshoot_active and dist_to_end <= self.waypoint_tolerance:
                if self.phase1_overshoot > 0.01:
                    self.phase1_overshoot_active = True
                    self.phase1_overshoot_start = (self.current_x, self.current_y)
                    self.get_logger().info(f"[FSM] Phase 1 Reached WP 27 -> Extra forward clearance ({self.phase1_overshoot:.2f}m) for reverse turning clearance")
                else:
                    self.get_logger().info(f"[FSM] Phase 1 Completed (WP 27 reached) -> Transitioning to PHASE_2_REV_A")
                    self.state = FSMState.PHASE_2_REV_A

            if self.phase1_overshoot_active:
                traveled = math.hypot(
                    self.current_x - self.phase1_overshoot_start[0],
                    self.current_y - self.phase1_overshoot_start[1]
                )
                if traveled >= self.phase1_overshoot:
                    self.get_logger().info(f"[FSM] Phase 1 Extra Forward Clearance Complete ({traveled:.2f}m) -> Transitioning to PHASE_2_REV_A")
                    self.phase1_overshoot_active = False
                    self.state = FSMState.PHASE_2_REV_A
                else:
                    cmd_speed = self.forward_speed
                    cmd_steer = 0.0
                    target_pt = (self.waypoints[27].x, self.waypoints[27].y)
                    status_text = f"PHASE 1: FORWARD CLEARANCE ({traveled:.2f}m / {self.phase1_overshoot:.2f}m)"
            else:
                cmd_steer, target_pt = self.compute_forward_pure_pursuit(active_indices)
                cmd_speed = self.forward_speed
                status_text = f"PHASE 1: FORWARD [1-27] (Dist to WP27: {dist_to_end:.2f}m)"

        # -----------------------------------------------------------
        # 2. Phase 2: Reverse Parking Mission A (Target Center: x=0.0, y=4.2, yaw=0.0)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_2_REV_A:
            active_indices = list(range(28, 34))
            dist_to_parking_center = math.hypot(
                center_x - self.parking_a_x,
                center_y - self.parking_a_y
            )
            if dist_to_parking_center <= self.parking_a_tolerance:
                self.get_logger().info(f"[FSM] Phase 2 Vehicle Center Reached Parking Zone A ({self.parking_a_x}, {self.parking_a_y}) (dist={dist_to_parking_center:.2f}m) -> Transitioning to PHASE_2_ALIGN_A")
                self.state = FSMState.PHASE_2_ALIGN_A
            else:
                cmd_steer, target_pt = self.compute_reverse_pure_pursuit(
                    active_indices,
                    target_yaw=self.parking_a_yaw,
                    custom_final_pt=(self.parking_a_x, self.parking_a_y)
                )
                cmd_speed = self.reverse_speed
                status_text = f"PHASE 2: REVERSE PARKING A (Center Dist to ({self.parking_a_x}, {self.parking_a_y}): {dist_to_parking_center:.2f}m)"

        # -----------------------------------------------------------
        # Phase 2 Alignment: Horizontal Alignment at Parking Zone A (Target: x=0.0, y=4.2, yaw=0.0)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_2_ALIGN_A:
            active_indices = [33]
            cmd_spd, cmd_st, dist_to_parking_center, yaw_err, is_aligned, dir_str = self.compute_alignment_control(
                self.parking_a_x, self.parking_a_y, self.parking_a_yaw,
                center_x, center_y, self.current_yaw,
                dist_tol=self.parking_a_tolerance,
                yaw_tol=self.parking_a_yaw_tolerance
            )

            # Strict parking condition: MUST satisfy BOTH [Heading Aligned AND Center In Position]
            if is_aligned:
                self.get_logger().info(
                    f"[FSM] Mission A Alignment Complete (Center & Heading OK) at ({self.parking_a_x}, {self.parking_a_y}) (yaw_err={math.degrees(yaw_err):.1f}°, dist={dist_to_parking_center:.2f}m) -> Starting 3.0s Parking Wait"
                )
                self.state = FSMState.PHASE_2_WAIT
                self.wait_start_time = now
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st
                target_pt = (self.parking_a_x, self.parking_a_y)
                status_text = f"MISSION A ALIGNING [{dir_str}] (Dist: {dist_to_parking_center:.2f}m / {self.parking_a_tolerance:.2f}m, Yaw Err: {math.degrees(yaw_err):.1f}° / {math.degrees(self.parking_a_yaw_tolerance):.1f}°)"

        # -----------------------------------------------------------
        # Phase 2 Parking Wait (3.0s AFTER Alignment Complete)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_2_WAIT:
            active_indices = [33]
            elapsed_sec = (now - self.wait_start_time).nanoseconds / 1e9
            cmd_speed = 0.0
            cmd_steer = 0.0
            target_pt = (self.parking_a_x, self.parking_a_y)
            status_text = f"MISSION A PARKING WAIT ({elapsed_sec:.1f}s / {self.parking_wait_sec:.1f}s)"

            if elapsed_sec >= self.parking_wait_sec:
                self.get_logger().info("[FSM] Mission A Parking Complete (3s elapsed) -> Transitioning to PHASE_3_FWD")
                self.state = FSMState.PHASE_3_FWD

        # -----------------------------------------------------------
        # 3. Phase 3: Forward (WP 34 -> 55)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_3_FWD:
            active_indices = list(range(34, 56))
            dist_to_end = math.hypot(
                self.current_x - self.waypoints[55].x,
                self.current_y - self.waypoints[55].y
            )

            if not self.phase3_overshoot_active and dist_to_end <= self.waypoint_tolerance:
                if self.phase3_overshoot > 0.01:
                    self.phase3_overshoot_active = True
                    self.phase3_overshoot_start = (self.current_x, self.current_y)
                    self.get_logger().info(f"[FSM] Phase 3 Reached WP 55 -> Extra forward clearance ({self.phase3_overshoot:.2f}m) for reverse turning clearance")
                else:
                    self.get_logger().info(f"[FSM] Phase 3 Completed (WP 55 reached, dist={dist_to_end:.2f}m) -> Transitioning to PHASE_4_REV_B")
                    self.state = FSMState.PHASE_4_REV_B

            if self.phase3_overshoot_active:
                traveled = math.hypot(
                    self.current_x - self.phase3_overshoot_start[0],
                    self.current_y - self.phase3_overshoot_start[1]
                )
                if traveled >= self.phase3_overshoot:
                    self.get_logger().info(f"[FSM] Phase 3 Extra Forward Clearance Complete ({traveled:.2f}m) -> Transitioning to PHASE_4_REV_B")
                    self.phase3_overshoot_active = False
                    self.state = FSMState.PHASE_4_REV_B
                else:
                    cmd_speed = self.forward_speed
                    cmd_steer = 0.0
                    target_pt = (self.waypoints[55].x, self.waypoints[55].y)
                    status_text = f"PHASE 3: FORWARD CLEARANCE ({traveled:.2f}m / {self.phase3_overshoot:.2f}m)"
            else:
                cmd_steer, target_pt = self.compute_forward_pure_pursuit(active_indices)
                cmd_speed = self.forward_speed
                status_text = f"PHASE 3: FORWARD [34-55] (Dist to WP55: {dist_to_end:.2f}m)"

        # -----------------------------------------------------------
        # 4. Phase 4: Reverse Parking Mission B (Target Center: x=2.1, y=3.3, yaw=-1.57)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_4_REV_B:
            active_indices = list(range(56, 62))
            dist_to_parking_center = math.hypot(
                center_x - self.parking_b_x,
                center_y - self.parking_b_y
            )
            if dist_to_parking_center <= self.parking_b_tolerance:
                self.get_logger().info(f"[FSM] Phase 4 Vehicle Center Reached Parking Zone B ({self.parking_b_x}, {self.parking_b_y}) (dist={dist_to_parking_center:.2f}m) -> Transitioning to PHASE_4_ALIGN_B")
                self.state = FSMState.PHASE_4_ALIGN_B
            else:
                cmd_steer, target_pt = self.compute_reverse_pure_pursuit(
                    active_indices,
                    target_yaw=self.parking_b_yaw,
                    custom_final_pt=(self.parking_b_x, self.parking_b_y)
                )
                cmd_speed = self.reverse_speed
                status_text = f"PHASE 4: REVERSE PARKING B (Center Dist to ({self.parking_b_x}, {self.parking_b_y}): {dist_to_parking_center:.2f}m)"

        # -----------------------------------------------------------
        # Phase 4 Alignment: Vertical Alignment at Parking Zone B (Target: x=2.1, y=3.3, yaw=-1.57)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_4_ALIGN_B:
            active_indices = [61]
            cmd_spd, cmd_st, dist_to_parking_center, yaw_err, is_aligned, dir_str = self.compute_alignment_control(
                self.parking_b_x, self.parking_b_y, self.parking_b_yaw,
                center_x, center_y, self.current_yaw,
                dist_tol=self.parking_b_tolerance,
                yaw_tol=self.parking_b_yaw_tolerance
            )

            # Strict parking condition: MUST satisfy BOTH [Heading Aligned AND Center In Position]
            if is_aligned:
                self.get_logger().info(
                    f"[FSM] Mission B Alignment Complete (Center & Heading OK) at ({self.parking_b_x}, {self.parking_b_y}) (yaw_err={math.degrees(yaw_err):.1f}°, dist={dist_to_parking_center:.2f}m) -> Starting 3.0s Parking Wait"
                )
                self.state = FSMState.PHASE_4_WAIT
                self.wait_start_time = now
            else:
                cmd_speed = cmd_spd
                cmd_steer = cmd_st
                target_pt = (self.parking_b_x, self.parking_b_y)
                status_text = f"MISSION B ALIGNING [{dir_str}] (Dist: {dist_to_parking_center:.2f}m / {self.parking_b_tolerance:.2f}m, Yaw Err: {math.degrees(yaw_err):.1f}° / {math.degrees(self.parking_b_yaw_tolerance):.1f}°)"

        # -----------------------------------------------------------
        # Phase 4 Parking Wait (3.0s AFTER Alignment Complete)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_4_WAIT:
            active_indices = [61]
            elapsed_sec = (now - self.wait_start_time).nanoseconds / 1e9
            cmd_speed = 0.0
            cmd_steer = 0.0
            target_pt = (self.parking_b_x, self.parking_b_y)
            status_text = f"MISSION B PARKING WAIT ({elapsed_sec:.1f}s / {self.parking_wait_sec:.1f}s)"

            if elapsed_sec >= self.parking_wait_sec:
                self.get_logger().info("[FSM] Mission B Parking Complete (3s elapsed) -> Transitioning to PHASE_5_FWD")
                self.state = FSMState.PHASE_5_FWD

        # -----------------------------------------------------------
        # 5. Phase 5: Forward (WP 62 -> 79)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_5_FWD:
            active_indices = list(range(62, 80))
            dist_to_end = math.hypot(
                self.current_x - self.waypoints[79].x,
                self.current_y - self.waypoints[79].y
            )
            if dist_to_end <= self.waypoint_tolerance:
                self.get_logger().info(f"[FSM] Phase 5 Completed (WP 79 reached, dist={dist_to_end:.2f}m) -> Transitioning to PHASE_6_RETURN")
                self.state = FSMState.PHASE_6_RETURN
            else:
                cmd_steer, target_pt = self.compute_forward_pure_pursuit(active_indices)
                cmd_speed = self.forward_speed
                status_text = f"PHASE 5: FORWARD [62-79] (Dist to WP79: {dist_to_end:.2f}m)"

        # -----------------------------------------------------------
        # 6. Phase 6: Return to Start Zone (Target Center: x=1.8, y=0.9, yaw=3.14)
        # -----------------------------------------------------------
        elif self.state == FSMState.PHASE_6_RETURN:
            # Targeting start pose (1.8, 0.9)
            dist_to_start = math.hypot(
                center_x - self.start_x,
                center_y - self.start_y
            )
            if dist_to_start <= self.start_tolerance:
                self.get_logger().info(f"[FSM] Vehicle Center Returned to Start Pose ({self.start_x}, {self.start_y}) (dist={dist_to_start:.2f}m) -> MISSION COMPLETE & SHUTDOWN!")
                self.state = FSMState.MISSION_COMPLETE
            else:
                cmd_steer, target_pt = self.compute_forward_pure_pursuit([1])
                cmd_speed = self.forward_speed
                status_text = f"PHASE 6: RETURN TO START (Dist to ({self.start_x}, {self.start_y}): {dist_to_start:.2f}m)"

        # -----------------------------------------------------------
        # Mission Complete
        # -----------------------------------------------------------
        elif self.state == FSMState.MISSION_COMPLETE:
            cmd_speed = 0.0
            cmd_steer = 0.0
            target_pt = (self.waypoints[1].x, self.waypoints[1].y)
            status_text = "🎉 MISSION COMPLETE & VEHICLE SHUTDOWN 🎉"

        # -----------------------------------------------------------
        # Safety Overrides (Emergency Brake & Cone Slowdown)
        # -----------------------------------------------------------
        is_fwd = (cmd_speed > 0.0)
        is_rev = (cmd_speed < 0.0)
        e_stop_active = False

        if is_fwd and self.obstacle_distance_front < self.obstacle_stop_dist:
            cmd_speed = 0.0
            e_stop_active = True
            status_text = f"⚠️ EMERGENCY BRAKE (Front Obstacle: {self.obstacle_distance_front:.2f}m)"
            self.get_logger().warn(status_text, throttle_duration_sec=1.0)
        elif is_rev and self.obstacle_distance_rear < self.obstacle_stop_dist:
            cmd_speed = 0.0
            e_stop_active = True
            status_text = f"⚠️ EMERGENCY BRAKE (Rear Obstacle: {self.obstacle_distance_rear:.2f}m)"
            self.get_logger().warn(status_text, throttle_duration_sec=1.0)
        elif is_fwd and self.cone_detected:
            cmd_speed *= self.cone_slowdown_scale
            status_text += " [CONE DETECTED - SLOWDOWN]"

        # Publish Command
        self.publish_actuator_commands(cmd_speed, cmd_steer)

        # Publish Visualizations
        self.publish_active_path(active_indices)
        self.publish_markers(status_text, target_pt, e_stop_active)

    # ===============================================================
    # Actuator Publishing
    # ===============================================================
    def publish_actuator_commands(self, speed_mps: float, steer_rad: float):
        # 1. Publish Twist on /cmd_vel
        twist = Twist()
        twist.linear.x = float(speed_mps)
        if abs(speed_mps) > 0.01:
            twist.angular.z = float((speed_mps / self.wheelbase) * math.tan(steer_rad))
        else:
            twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

        # 2. Publish XycarMotor on /xycar_motor
        if HAS_XYCAR_MSGS:
            motor_msg = XycarMotor()
            motor_msg.header.stamp = self.get_clock().now().to_msg()
            motor_msg.header.frame_id = self.base_frame

            # Convert steering angle in radians to the calibrated servo command.
            # Xycar standard: negative angle turns left, positive right, or vice versa
            steer_unit = int(round(-steer_rad / self.xycar_steer_scale))
            steer_unit = int(np.clip(
                steer_unit,
                -self.xycar_max_steer_command,
                self.xycar_max_steer_command,
            ))

            # Convert speed in m/s to the calibrated motor command.
            speed_unit = int(round(speed_mps / self.xycar_speed_scale))
            speed_unit = int(np.clip(
                speed_unit,
                -self.xycar_max_speed_command,
                self.xycar_max_speed_command,
            ))

            motor_msg.angle = float(steer_unit)
            motor_msg.speed = float(speed_unit)
            self.motor_pub.publish(motor_msg)

    # ===============================================================
    # RViz 2 Visualizations
    # ===============================================================
    def publish_active_path(self, active_indices):
        if not active_indices:
            return
        path = Path()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for idx in active_indices:
            wp = self.waypoints.get(idx)
            if wp is not None:
                p = PoseStamped()
                p.header.frame_id = self.global_frame
                p.pose.position.x = wp.x
                p.pose.position.y = wp.y
                p.pose.position.z = 0.02
                p.pose.orientation.w = 1.0
                path.poses.append(p)

        self.active_path_pub.publish(path)

    def publish_markers(self, status_text: str, target_pt, e_stop: bool):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: Floating Status Text above vehicle
        text_marker = Marker()
        text_marker.header.frame_id = self.global_frame
        text_marker.header.stamp = now
        text_marker.ns = "mission_status"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = self.current_x
        text_marker.pose.position.y = self.current_y
        text_marker.pose.position.z = 0.70  # 70cm above ground
        text_marker.scale.z = 0.18
        text_marker.text = status_text

        if e_stop:
            text_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif self.state == FSMState.MISSION_COMPLETE:
            text_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=1.0)
        elif "WAIT" in status_text:
            text_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)
        elif "REVERSE" in status_text:
            text_marker.color = ColorRGBA(r=1.0, g=0.2, b=0.8, a=1.0)
        else:
            text_marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=1.0)

        marker_array.markers.append(text_marker)

        # Marker 2: Target Point Sphere
        if target_pt is not None:
            target_marker = Marker()
            target_marker.header.frame_id = self.global_frame
            target_marker.header.stamp = now
            target_marker.ns = "lookahead_target"
            target_marker.id = 1
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = float(target_pt[0])
            target_marker.pose.position.y = float(target_pt[1])
            target_marker.pose.position.z = 0.10
            target_marker.scale.x = 0.15
            target_marker.scale.y = 0.15
            target_marker.scale.z = 0.15
            target_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            marker_array.markers.append(target_marker)

        # Marker 3: Vehicle 3D Body Frame Marker (55cm x 30cm x 25cm)
        car_marker = Marker()
        car_marker.header.frame_id = self.base_frame
        car_marker.header.stamp = now
        car_marker.ns = "vehicle_body"
        car_marker.id = 5
        car_marker.type = Marker.CUBE
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = self.wheelbase / 2.0  # Center between axles
        car_marker.pose.position.y = 0.0
        car_marker.pose.position.z = self.vehicle_height / 2.0  # 12.5cm height
        car_marker.scale.x = self.vehicle_length
        car_marker.scale.y = self.vehicle_width
        car_marker.scale.z = self.vehicle_height
        car_marker.color = ColorRGBA(r=0.0, g=0.7, b=1.0, a=0.60)  # Translucent cyan body
        marker_array.markers.append(car_marker)

        # Marker 4: Vehicle Center Indicator (Sphere at exact center of body)
        center_marker = Marker()
        center_marker.header.frame_id = self.base_frame
        center_marker.header.stamp = now
        center_marker.ns = "vehicle_center"
        center_marker.id = 6
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.wheelbase / 2.0
        center_marker.pose.position.y = 0.0
        center_marker.pose.position.z = self.vehicle_height / 2.0
        center_marker.scale.x = 0.09
        center_marker.scale.y = 0.09
        center_marker.scale.z = 0.09
        center_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)  # Bright yellow center sphere
        marker_array.markers.append(center_marker)

        # Marker 5: Vehicle Center Ground Footprint Marker (Disc on ground)
        ground_center_marker = Marker()
        ground_center_marker.header.frame_id = self.base_frame
        ground_center_marker.header.stamp = now
        ground_center_marker.ns = "vehicle_center_ground"
        ground_center_marker.id = 7
        ground_center_marker.type = Marker.CYLINDER
        ground_center_marker.action = Marker.ADD
        ground_center_marker.pose.position.x = self.wheelbase / 2.0
        ground_center_marker.pose.position.y = 0.0
        ground_center_marker.pose.position.z = 0.015
        ground_center_marker.scale.x = 0.16
        ground_center_marker.scale.y = 0.16
        ground_center_marker.scale.z = 0.03
        ground_center_marker.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.95)  # Bright red ground disc
        marker_array.markers.append(ground_center_marker)

        # Marker 6: Vehicle Nose Heading Arrow
        nose_marker = Marker()
        nose_marker.header.frame_id = self.base_frame
        nose_marker.header.stamp = now
        nose_marker.ns = "vehicle_heading"
        nose_marker.id = 8
        nose_marker.type = Marker.ARROW
        nose_marker.action = Marker.ADD
        nose_marker.pose.position.x = self.wheelbase / 2.0
        nose_marker.pose.position.y = 0.0
        nose_marker.pose.position.z = self.vehicle_height / 2.0
        nose_marker.scale.x = 0.35  # Arrow length
        nose_marker.scale.y = 0.04
        nose_marker.scale.z = 0.04
        nose_marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.95)  # Bright green arrow pointing forward
        marker_array.markers.append(nose_marker)

        # Marker 7: Start Zone Box (0.70m x 0.30m)
        start_zone = Marker()
        start_zone.header.frame_id = self.global_frame
        start_zone.header.stamp = now
        start_zone.ns = "mission_zones"
        start_zone.id = 10
        start_zone.type = Marker.CUBE
        start_zone.action = Marker.ADD
        start_zone.pose.position.x = self.start_x
        start_zone.pose.position.y = self.start_y
        start_zone.pose.position.z = 0.01
        start_zone.scale.x = self.start_box_length  # 0.70m
        start_zone.scale.y = self.start_box_width   # 0.30m
        start_zone.scale.z = 0.02
        start_zone.color = ColorRGBA(r=0.0, g=1.0, b=0.4, a=0.50)  # Green translucent box
        marker_array.markers.append(start_zone)

        # Marker 8: Parking Zone A (horizontal vehicle footprint)
        park_a = Marker()
        park_a.header.frame_id = self.global_frame
        park_a.header.stamp = now
        park_a.ns = "mission_zones"
        park_a.id = 11
        park_a.type = Marker.CUBE
        park_a.action = Marker.ADD
        park_a.pose.position.x = self.parking_a_x  # 0.00m
        park_a.pose.position.y = self.parking_a_y  # 4.20m
        park_a.pose.position.z = 0.01
        park_a.scale.x = self.vehicle_length
        park_a.scale.y = self.vehicle_width
        park_a.scale.z = 0.02
        park_a.color = ColorRGBA(r=1.0, g=0.0, b=0.8, a=0.55)  # Magenta translucent box
        marker_array.markers.append(park_a)

        # Marker 9: Parking Zone B (vertical vehicle footprint)
        park_b = Marker()
        park_b.header.frame_id = self.global_frame
        park_b.header.stamp = now
        park_b.ns = "mission_zones"
        park_b.id = 12
        park_b.type = Marker.CUBE
        park_b.action = Marker.ADD
        park_b.pose.position.x = self.parking_b_x  # 2.10m
        park_b.pose.position.y = self.parking_b_y  # 3.30m
        park_b.pose.position.z = 0.01
        park_b.scale.x = self.vehicle_width
        park_b.scale.y = self.vehicle_length
        park_b.scale.z = 0.02
        park_b.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.55)  # Orange translucent box
        marker_array.markers.append(park_b)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionWaypointFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            if node.pose_received and rclpy.ok():
                node.publish_actuator_commands(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
