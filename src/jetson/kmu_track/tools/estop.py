#!/usr/bin/env python3
"""Publish one latched emergency-stop request."""

import rclpy
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node('vehicle_estop_tool')
    publisher = node.create_publisher(Bool, '/vehicle/estop', 10)
    for _ in range(3):
        publisher.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
