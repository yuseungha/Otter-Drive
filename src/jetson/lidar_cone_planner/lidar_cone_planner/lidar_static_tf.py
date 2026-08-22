"""Publish the measured base-to-LiDAR transform from YAML parameters."""

from math import cos, isfinite, sin

import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class LidarStaticTf(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "cone_lidar_static_tf", parameter_overrides=parameter_overrides or []
        )
        read_only = ParameterDescriptor(read_only=True)
        self.declare_parameter("planning_frame", "base_link", read_only)
        self.declare_parameter("laser_frame", "laser", read_only)
        self.declare_parameter("lidar_x_m", 0.0, read_only)
        self.declare_parameter("lidar_y_m", 0.0, read_only)
        self.declare_parameter("lidar_z_m", 0.0, read_only)
        self.declare_parameter("lidar_roll_rad", 0.0, read_only)
        self.declare_parameter("lidar_pitch_rad", 0.0, read_only)
        self.declare_parameter("lidar_yaw_rad", 0.0, read_only)

        parent = str(self.get_parameter("planning_frame").value)
        child = str(self.get_parameter("laser_frame").value)
        if not parent or not child or parent == child:
            raise ValueError("planning_frame and laser_frame must be non-empty and distinct")
        x, y, z, roll, pitch, yaw = (
            float(self.get_parameter(name).value)
            for name in (
                "lidar_x_m",
                "lidar_y_m",
                "lidar_z_m",
                "lidar_roll_rad",
                "lidar_pitch_rad",
                "lidar_yaw_rad",
            )
        )
        if not all(isfinite(value) for value in (x, y, z, roll, pitch, yaw)):
            raise ValueError("LiDAR transform parameters must be finite")

        cr, sr = cos(0.5 * roll), sin(0.5 * roll)
        cp, sp = cos(0.5 * pitch), sin(0.5 * pitch)
        cy, sy = cos(0.5 * yaw), sin(0.5 * yaw)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
        transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
        transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
        transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(transform)
        self.get_logger().info(
            "Static TF %s -> %s: xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)"
            % (parent, child, x, y, z, roll, pitch, yaw)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarStaticTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
