#!/usr/bin/env python3
"""Republish /odom with finite covariance so RTAB-Map trusts wheel odometry."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomCovarianceRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("odom_covariance")
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odom")
        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Odometry) -> None:
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        # Trust XY/yaw from wheel encoders; reject roll/pitch drift from vision.
        out.pose.covariance[0] = 0.02
        out.pose.covariance[7] = 0.02
        out.pose.covariance[14] = 1e6
        out.pose.covariance[21] = 1e6
        out.pose.covariance[28] = 1e6
        out.pose.covariance[35] = 0.05
        out.twist.covariance[0] = 0.02
        out.twist.covariance[35] = 0.05
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdomCovarianceRepublisher()
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
