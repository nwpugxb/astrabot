#!/usr/bin/env python3
"""Convert sensor_msgs/LaserScan to sensor_msgs/PointCloud2 (/scan -> /cloud)."""

import laser_geometry.laser_geometry as lg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanToCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_to_cloud")
        self._proj = lg.LaserProjection()
        self._pub = self.create_publisher(PointCloud2, "cloud", 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.get_logger().info("Publishing PointCloud2 on /cloud from /scan")

    def _on_scan(self, msg: LaserScan) -> None:
        cloud = self._proj.projectLaser(msg)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = ScanToCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
