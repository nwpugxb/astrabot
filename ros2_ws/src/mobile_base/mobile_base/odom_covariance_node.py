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
        # Pass through bag covariance when present (e.g. turn-dynamic from arduino_base).
        yaw_cov = msg.pose.covariance[35]
        lin_cov = msg.pose.covariance[0]
        if yaw_cov <= 0.0 or lin_cov <= 0.0:
            lin_cov = 0.02
            yaw_cov = 0.05
        out.pose.covariance[0] = lin_cov
        out.pose.covariance[7] = lin_cov
        out.pose.covariance[14] = 1e6
        out.pose.covariance[21] = 1e6
        out.pose.covariance[28] = 1e6
        out.pose.covariance[35] = yaw_cov
        twist_yaw = msg.twist.covariance[35]
        twist_lin = msg.twist.covariance[0]
        if twist_yaw <= 0.0:
            twist_yaw = yaw_cov
        if twist_lin <= 0.0:
            twist_lin = lin_cov
        out.twist.covariance[0] = twist_lin
        out.twist.covariance[35] = twist_yaw
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
