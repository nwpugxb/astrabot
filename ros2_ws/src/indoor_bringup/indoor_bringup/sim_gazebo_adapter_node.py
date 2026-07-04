#!/usr/bin/env python3
"""Adapt Gazebo Sim topics to the real-robot naming used by Nav2 / EKF / layers."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan


class SimGazeboAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_gazebo_adapter")
        self.declare_parameter("scan_in", "/scan_unfixed")
        self.declare_parameter("scan_out", "/scan")
        self.declare_parameter("scan_frame_id", "laser")
        # gpu_lidar scan angles already match REP-143 (+X forward) in our Fortress setup.
        self.declare_parameter("scan_angle_offset", 0.0)
        self.declare_parameter("odom_in", "/diff_drive_controller/odom")
        self.declare_parameter("odom_out", "/odom")
        self.declare_parameter("depth_in", "/gz/camera/depth/image_raw")
        self.declare_parameter("depth_out", "/camera/depth/image_raw")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        self.declare_parameter("cmd_vel_in", "/cmd_vel")
        self.declare_parameter("cmd_vel_out", "/diff_drive_controller/cmd_vel_unstamped")
        self._logged_scan_frame = False

        self._scan_frame = str(self.get_parameter("scan_frame_id").value)
        self._scan_angle_offset = float(self.get_parameter("scan_angle_offset").value)
        self._depth_frame = str(self.get_parameter("depth_frame_id").value)
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("scan_out").value), 10
        )
        self._odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_out").value), 10
        )
        self._odom_filtered_pub = self.create_publisher(
            Odometry, "/odometry/filtered", 10
        )
        self._depth_pub = self.create_publisher(
            Image, str(self.get_parameter("depth_out").value), 10
        )
        self._cmd_vel_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_out").value), cmd_qos
        )

        self.create_subscription(
            LaserScan, str(self.get_parameter("scan_in").value), self._on_scan, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_in").value), self._on_odom, 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("depth_in").value), self._on_depth, 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_in").value),
            self._on_cmd_vel,
            cmd_qos,
        )

        self.get_logger().info(
            f"scan {self.get_parameter('scan_in').value} -> "
            f"{self.get_parameter('scan_out').value} (frame={self._scan_frame}); "
            f"odom {self.get_parameter('odom_in').value} -> "
            f"{self.get_parameter('odom_out').value}; "
            f"cmd_vel -> {self.get_parameter('cmd_vel_out').value}"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        if not self._logged_scan_frame:
            self._logged_scan_frame = True
            self.get_logger().info(
                f"Gazebo scan frame '{msg.header.frame_id}' -> '{self._scan_frame}'"
            )
        msg.header.frame_id = self._scan_frame
        if abs(self._scan_angle_offset) > 1e-6:
            msg.angle_min += self._scan_angle_offset
            msg.angle_max += self._scan_angle_offset
        self._scan_pub.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"
        self._odom_pub.publish(msg)
        self._odom_filtered_pub.publish(msg)

    def _on_depth(self, msg: Image) -> None:
        msg.header.frame_id = self._depth_frame
        self._depth_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGazeboAdapterNode()
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
