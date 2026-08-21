#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

try:
    from xycar_msgs.msg import XycarMotor
    HAS_XYCAR_MSGS = True
except ImportError:
    HAS_XYCAR_MSGS = False


class SimVehicleNode(Node):
    """
    Ackermann Kinematic Bicycle Simulator for 1/10 RC Vehicle
    Subscribes to /xycar_motor or /cmd_vel, integrates vehicle kinematics,
    and broadcasts TF (map -> base_link) & /odom for local testing in RViz 2.
    """

    def __init__(self):
        super().__init__('sim_vehicle_node')

        # Initial pose from waypoint 1 (x=1.80, y=0.90, yaw=pi)
        self.declare_parameter('init_x', 1.80)
        self.declare_parameter('init_y', 0.90)
        self.declare_parameter('init_yaw', 3.141592)
        self.declare_parameter('wheelbase', 0.315)
        self.declare_parameter('max_steer_rad', 0.610865)
        self.declare_parameter('xycar_steer_scale', 0.000939793)
        self.declare_parameter('xycar_speed_scale', 0.000230769)
        self.declare_parameter('sim_rate_hz', 50.0)

        self.x = self.get_parameter('init_x').get_parameter_value().double_value
        self.y = self.get_parameter('init_y').get_parameter_value().double_value
        self.yaw = self.get_parameter('init_yaw').get_parameter_value().double_value
        self.L = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steer_rad = self.get_parameter('max_steer_rad').get_parameter_value().double_value
        self.xycar_steer_scale = self.get_parameter('xycar_steer_scale').get_parameter_value().double_value
        self.xycar_speed_scale = self.get_parameter('xycar_speed_scale').get_parameter_value().double_value
        self.dt = 1.0 / self.get_parameter('sim_rate_hz').get_parameter_value().double_value

        self.target_speed = 0.0  # m/s
        self.target_steer = 0.0  # rad

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Control Subscribers
        if HAS_XYCAR_MSGS:
            self.create_subscription(XycarMotor, '/xycar_motor', self.xycar_motor_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Simulation update timer
        self.timer = self.create_timer(self.dt, self.update_kinematics)
        self.scan_timer = self.create_timer(0.1, self.publish_mock_scan)

        self.get_logger().info(f"SimVehicleNode initialized at ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad)")

    def xycar_motor_cb(self, msg):
        # Convert calibrated Xycar commands to SI units.
        steer_rad = -float(msg.angle) * self.xycar_steer_scale
        speed_mps = float(msg.speed) * self.xycar_speed_scale
        self.target_steer = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)
        self.target_speed = speed_mps

    def cmd_vel_cb(self, msg: Twist):
        self.target_speed = msg.linear.x
        # If angular velocity given, steer = atan(w * L / v)
        if abs(msg.linear.x) > 0.01:
            steer = math.atan(msg.angular.z * self.L / msg.linear.x)
            self.target_steer = np.clip(steer, -self.max_steer_rad, self.max_steer_rad)
        else:
            self.target_steer = 0.0

    def update_kinematics(self):
        v = self.target_speed
        delta = self.target_steer

        # Bicycle model integration:
        # dx = v * cos(yaw)
        # dy = v * sin(yaw)
        # dyaw = (v / L) * tan(delta)
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw += (v / self.L) * math.tan(delta) * self.dt

        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        now = self.get_clock().now().to_msg()

        # Quaternion from yaw
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Broadcast map -> base_link TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float((v / self.L) * math.tan(delta))
        self.odom_pub.publish(odom)

    def publish_mock_scan(self):
        # Publish a safe LaserScan with no close obstacles
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(1.0)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 10.0
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [5.0] * num_readings
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = SimVehicleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
