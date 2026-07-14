#!/usr/bin/env python3
"""Host-time TF + scan bridge for deck SLAM.

Prefers /odom (Odometry). Falls back to /odom_pose (PoseStamped) — small enough
for default micro-ROS XRCE MTU when full Odometry is dropped.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

QOS_BE1 = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class DeckSlamSync(Node):
    def __init__(self) -> None:
        super().__init__("deck_slam_sync")
        self._br = TransformBroadcaster(self)
        self._scan_pub = self.create_publisher(LaserScan, "/scan_slam", 10)

        self._x = 0.0
        self._y = 0.0
        self._qz = 0.0
        self._qw = 1.0
        self._have_pose = False
        self._pose_src = ""
        self._scan_count = 0
        self._last_tf_stamp = None

        self.create_subscription(Odometry, "/odom", self._on_odom, QOS_BE1)
        self.create_subscription(PoseStamped, "/odom_pose", self._on_pose, QOS_BE1)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_timer(0.02, self._pub_tf)
        self.create_timer(2.0, self._status)

        self.get_logger().info(
            "deck_slam_sync: TF from /odom or /odom_pose + /scan→/scan_slam"
        )

    def _now(self):
        return self.get_clock().now().to_msg()

    def _set_pose(self, x: float, y: float, qz: float, qw: float, src: str) -> None:
        self._x, self._y = x, y
        n = abs(qz) + abs(qw)
        if n < 1e-9:
            self._qz, self._qw = 0.0, 1.0
        else:
            self._qz, self._qw = qz, qw
        if not self._have_pose:
            self._have_pose = True
            self._pose_src = src
            self.get_logger().info(f"First pose from {src}: x={x:.3f} y={y:.3f}")

    def _pub_tf(self) -> None:
        stamp = self._now()
        self._last_tf_stamp = stamp
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = self._qz
        t.transform.rotation.w = self._qw
        self._br.sendTransform(t)

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._set_pose(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(q.z),
            float(q.w),
            "/odom",
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        # Prefer full /odom when both exist; still accept pose if odom silent.
        if self._pose_src == "/odom":
            return
        q = msg.pose.orientation
        self._set_pose(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(q.z),
            float(q.w),
            "/odom_pose",
        )

    def _on_scan(self, msg: LaserScan) -> None:
        stamp = self._last_tf_stamp if self._last_tf_stamp is not None else self._now()
        out = LaserScan()
        out.header.stamp = stamp
        out.header.frame_id = "laser"
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = list(msg.ranges)
        out.intensities = list(msg.intensities)
        self._scan_pub.publish(out)
        self._scan_count += 1
        if self._scan_count == 1:
            self.get_logger().info(f"First /scan→/scan_slam ranges={len(out.ranges)}")

    def _status(self) -> None:
        if not self._have_pose:
            self.get_logger().warn(
                "No /odom or /odom_pose yet — check ESP32 micro-ROS. "
                f"scans={self._scan_count}"
            )


def main() -> None:
    rclpy.init()
    node = DeckSlamSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
