#!/usr/bin/env python3
"""Republish /odom (BEST_EFFORT from micro-ROS) as odom→base_footprint TF.

slam_toolbox needs TF; ESP32 publishes /odom only (no TF).
robot_localization EKF often uses RELIABLE and will not see BEST_EFFORT /odom.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self) -> None:
        super().__init__("odom_to_tf")
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.get_logger().info("Publishing TF odom → base_footprint from /odom (BEST_EFFORT)")

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = msg.header.frame_id or "odom"
        t.child_frame_id = msg.child_frame_id or "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        # Guard against empty quaternion
        q = t.transform.rotation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-9:
            q.w = 1.0
        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomToTf()
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
