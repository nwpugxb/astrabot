#!/usr/bin/env python3
"""Republish odom -> base_footprint TF from /odom (for offline bag replay)."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        topic = str(self.get_parameter("odom_topic").value)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self._on_odom, 10)
        self.get_logger().info(f"Broadcasting TF from {topic}")

    def _on_odom(self, msg: Odometry) -> None:
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
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
