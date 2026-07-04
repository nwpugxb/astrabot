#!/usr/bin/env python3
"""Publish wheel odometry as nav_msgs/Path for RViz (frame: odom)."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

_ODOM_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class OdomPathNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_path")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/odom/path")
        self.declare_parameter("max_points", 50000)
        self.declare_parameter("min_distance_m", 0.01)

        odom_topic = str(self.get_parameter("odom_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._min_dist = float(self.get_parameter("min_distance_m").value)

        self._path = Path()
        self._path.header.frame_id = "odom"
        self._last_x = 0.0
        self._last_y = 0.0
        self._have_last = False

        self._pub = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, _ODOM_QOS)
        self.get_logger().info(f"Publishing {path_topic} from {odom_topic} (frame=odom)")

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._have_last:
            dx = x - self._last_x
            dy = y - self._last_y
            if (dx * dx + dy * dy) ** 0.5 < self._min_dist:
                return
        self._last_x = x
        self._last_y = y
        self._have_last = True

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose
        self._path.header = pose.header
        self._path.poses.append(pose)
        if len(self._path.poses) > self._max_points:
            self._path.poses = self._path.poses[-self._max_points :]
        self._pub.publish(self._path)


def main() -> None:
    rclpy.init()
    node = OdomPathNode()
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
