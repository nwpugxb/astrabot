#!/usr/bin/env python3
"""RViz helper: yellow = current /scan frame, gray = accumulated history (odom frame)."""

from __future__ import annotations

import math
import struct
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


def _quat_rotate(q, x: float, y: float, z: float) -> tuple[float, float, float]:
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    )


class SimSlamScanVizNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_slam_scan_viz")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("history_topic", "/sim/slam/history_cloud")
        self.declare_parameter("current_topic", "/sim/slam/current_cloud")
        self.declare_parameter("max_history_points", 120_000)
        self.declare_parameter("flatten_z", True)
        self.declare_parameter("slice_z", 0.28)

        self._frame = str(self.get_parameter("target_frame").value)
        self._max_pts = int(self.get_parameter("max_history_points").value)
        self._history: deque[tuple[float, float, float]] = deque(maxlen=self._max_pts)
        self._flatten_z = bool(self.get_parameter("flatten_z").value)
        self._slice_z = float(self.get_parameter("slice_z").value)
        self._have_tf = False

        hist_topic = str(self.get_parameter("history_topic").value)
        cur_topic = str(self.get_parameter("current_topic").value)
        self._hist_pub = self.create_publisher(PointCloud2, hist_topic, 10)
        self._cur_pub = self.create_publisher(PointCloud2, cur_topic, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        scan_topic = str(self.get_parameter("scan_topic").value)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.get_logger().info(
            f"Scan viz: {scan_topic} -> yellow {cur_topic}, gray {hist_topic} "
            f"({self._frame}, tf2 @ scan stamp)"
        )

    def _lookup_tf(self, scan: LaserScan):
        """Use scan acquisition time; 'latest' TF runs ahead during motion and smears walls."""
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = rclpy.time.Time()
        try:
            return self._tf_buffer.lookup_transform(
                self._frame,
                scan.header.frame_id,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception:
            return self._tf_buffer.lookup_transform(
                self._frame,
                scan.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )

    def _scan_to_points(self, scan: LaserScan) -> list[tuple[float, float, float]]:
        try:
            tf = self._lookup_tf(scan)
        except Exception:
            return []

        if not self._have_tf:
            self._have_tf = True
            t = tf.transform.translation
            self.get_logger().info(
                f"Scan TF ready: {scan.header.frame_id} -> {self._frame} "
                f"at ({t.x:.2f}, {t.y:.2f})"
            )

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tz = tf.transform.translation.z
        q = tf.transform.rotation

        pts: list[tuple[float, float, float]] = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max and math.isfinite(r):
                xl = r * math.cos(angle)
                yl = r * math.sin(angle)
                xr, yr, zr = _quat_rotate(q, xl, yl, 0.0)
                z = self._slice_z if self._flatten_z else tz + zr
                pts.append((tx + xr, ty + yr, z))
            angle += scan.angle_increment
        return pts

    def _make_cloud(self, header: Header, points: list[tuple[float, float, float]]) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header = header
        cloud.header.frame_id = self._frame
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        buf = b"".join(struct.pack("fff", x, y, z) for x, y, z in points)
        cloud.data = buf
        return cloud

    def _on_scan(self, scan: LaserScan) -> None:
        current = self._scan_to_points(scan)
        if not current:
            return
        if not self._history:
            self.get_logger().info(f"First scan cloud: {len(current)} points")
        self._history.extend(current)
        stamp = self.get_clock().now().to_msg()
        hdr = Header(stamp=stamp, frame_id=self._frame)
        self._cur_pub.publish(self._make_cloud(hdr, current))
        self._hist_pub.publish(self._make_cloud(hdr, list(self._history)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimSlamScanVizNode()
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
