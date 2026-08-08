#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan


class LidarBasicNode(Node):

    def __init__(self):
        super().__init__('lidar_basic_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('LiDAR Basic Node Started')

    def scan_callback(self, msg):

        self.get_logger().info(
            f'LiDAR data received: {len(msg.ranges)} points'
        )

        # 0 rad = LiDAR 좌표계 기준 정면
        front_index = int(
            round((0.0 - msg.angle_min) / msg.angle_increment)
        )

        if not (0 <= front_index < len(msg.ranges)):
            self.get_logger().warn(
                f'Front index out of range: {front_index}'
            )
            return

        front_distance = msg.ranges[front_index]

        if math.isfinite(front_distance):

            if msg.range_min <= front_distance <= msg.range_max:
                self.get_logger().info(
                    f'Front Distance: {front_distance:.3f} m'
                )
            else:
                self.get_logger().warn(
                    f'Front distance out of sensor range: '
                    f'{front_distance:.3f} m'
                )

        else:
            self.get_logger().info(
                'Front Distance: No valid object'
            )


def main(args=None):

    rclpy.init(args=args)

    node = LidarBasicNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()