#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import yaml
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion


class MapServerNode(Node):
    """
    ROS 2 Self-contained Map Server Node
    Loads a YAML + PGM map file and publishes OccupancyGrid to /map
    with Transient Local (Latched) QoS for seamless RViz 2 visualization.
    """

    def __init__(self):
        super().__init__('map_server_node')

        # Declare parameters
        self.declare_parameter('yaml_filename', 'parking_map.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_name', '/map')
        self.declare_parameter('publish_period_sec', 2.0)

        self.yaml_filename = self.get_parameter('yaml_filename').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.publish_period = self.get_parameter('publish_period_sec').get_parameter_value().double_value

        # Configure Transient Local QoS for RViz 2
        self.map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.map_pub = self.create_publisher(OccupancyGrid, self.topic_name, self.map_qos)
        self.meta_pub = self.create_publisher(MapMetaData, '/map_metadata', self.map_qos)

        self.occupancy_grid_msg = None
        self.load_map()

        # Timer to periodically refresh / guarantee delivery to late-joining subscribers
        self.timer = self.create_timer(self.publish_period, self.publish_map)
        # Immediate first publish
        self.publish_map()
        self.get_logger().info(f"MapServerNode initialized and publishing to {self.topic_name} (frame: {self.frame_id})")

    def load_map(self):
        yaml_path = self.yaml_filename
        if not os.path.isabs(yaml_path):
            # Check current working directory or package shares
            candidates = [
                os.path.join(os.getcwd(), yaml_path),
                os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    yaml_path,
                ),
                yaml_path
            ]
            for c in candidates:
                if os.path.exists(c):
                    yaml_path = c
                    break

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Map YAML file not found: {yaml_path}")
            return

        with open(yaml_path, 'r', encoding='utf-8') as f:
            map_config = yaml.safe_load(f)

        image_filename = map_config.get('image', 'parking_map.pgm')
        if not os.path.isabs(image_filename):
            image_path = os.path.join(os.path.dirname(yaml_path), image_filename)
        else:
            image_path = image_filename

        if not os.path.exists(image_path):
            self.get_logger().error(f"Map PGM image file not found: {image_path}")
            return

        resolution = float(map_config.get('resolution', 0.05))
        origin = map_config.get('origin', [-2.15, -1.05, 0.0])
        negate = int(map_config.get('negate', 0))
        occupied_thresh = float(map_config.get('occupied_thresh', 0.65))
        free_thresh = float(map_config.get('free_thresh', 0.25))

        # Load image via OpenCV (grayscale)
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Failed to read image with OpenCV: {image_path}")
            return

        height, width = img.shape
        self.get_logger().info(f"Loaded map image: {image_path} ({width}x{height}, res={resolution}m/px)")

        # Convert image to occupancy values:
        # ROS 2 coordinate convention: index (0,0) is bottom-left.
        # Images have row 0 at top, so flip vertically.
        img_flipped = cv2.flip(img, 0)
        img_float = img_flipped.astype(np.float32) / 255.0

        if negate:
            occ_prob = img_float
        else:
            occ_prob = 1.0 - img_float

        grid_data = np.full((height, width), -1, dtype=np.int8)
        grid_data[occ_prob > occupied_thresh] = 100
        grid_data[occ_prob < free_thresh] = 0

        # Construct OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height

        origin_pose = Pose()
        origin_pose.position.x = float(origin[0])
        origin_pose.position.y = float(origin[1])
        origin_pose.position.z = float(origin[2]) if len(origin) > 2 else 0.0
        origin_pose.orientation.w = 1.0
        msg.info.origin = origin_pose

        msg.data = grid_data.flatten().tolist()
        self.occupancy_grid_msg = msg

    def publish_map(self):
        if self.occupancy_grid_msg is not None:
            self.occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.occupancy_grid_msg)
            self.meta_pub.publish(self.occupancy_grid_msg.info)


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
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
