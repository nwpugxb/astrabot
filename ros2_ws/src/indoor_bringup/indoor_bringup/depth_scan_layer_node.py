#!/usr/bin/env python3
"""Mapping layer: Astra Pro depth -> LaserScan for low obstacles in a height band.

Publishes /mapping/layers/depth_scan (base_footprint frame) for fusion or RViz debug.
Ground points are removed by z-band filtering in base_link/base_footprint.
"""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import tf2_ros

from indoor_bringup.laser_utils import bin_points, scan_from_bins


def _quat_to_matrix(qx, qy, qz, qw) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


class DepthScanLayerNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_scan_layer")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_topic", "/mapping/layers/depth_scan")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("min_z_m", 0.05)
        self.declare_parameter("max_z_m", 0.40)
        self.declare_parameter("stride", 2)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("num_bins", 720)
        self.declare_parameter("angle_min", -3.14159)
        self.declare_parameter("angle_max", 3.14159)
        self.declare_parameter("range_min", 0.08)
        self.declare_parameter("range_max", 4.0)

        self._target = str(self.get_parameter("target_frame").value)
        self._min_z = float(self.get_parameter("min_z_m").value)
        self._max_z = float(self.get_parameter("max_z_m").value)
        self._stride = max(1, int(self.get_parameter("stride").value))
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._num_bins = int(self.get_parameter("num_bins").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        out_topic = str(self.get_parameter("output_topic").value)

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(LaserScan, out_topic, 10)
        self._depth_sub = self.create_subscription(
            Image, str(self.get_parameter("depth_topic").value), self._on_depth, 10
        )
        self._info: Optional[CameraInfo] = None
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_info,
            10,
        )
        self.get_logger().info(
            f"Depth scan layer -> {out_topic}, z=[{self._min_z}, {self._max_z}] m in {self._target}"
        )

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _depth_meters(self, msg: Image) -> np.ndarray:
        enc = (msg.encoding or "").lower()
        if enc in ("32fc1", "32fc"):
            d = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            return d.astype(np.float32)
        if enc in ("16uc1", "mono16"):
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            return raw.astype(np.float32) / 1000.0
        d = self._bridge.imgmsg_to_cv2(msg)
        return d.astype(np.float32)

    def _lookup(self, source: str, stamp: Time) -> Optional[np.ndarray]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target,
                source,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        m = _quat_to_matrix(q.x, q.y, q.z, q.w)
        m[0, 3], m[1, 3], m[2, 3] = t.x, t.y, t.z
        return m

    def _on_depth(self, msg: Image) -> None:
        if self._info is None:
            return
        cam = msg.header.frame_id or self._info.header.frame_id
        if not cam:
            return
        tf_mat = self._lookup(cam, Time.from_msg(msg.header.stamp))
        if tf_mat is None:
            return

        depth = self._depth_meters(msg)
        s = self._stride
        depth = depth[::s, ::s]
        h, w = depth.shape
        fx = self._info.k[0] / s
        fy = self._info.k[4] / s
        cx = self._info.k[2] / s
        cy = self._info.k[5] / s

        vs = np.arange(h, dtype=np.float32) + 0.5
        us = np.arange(w, dtype=np.float32) + 0.5
        vv, uu = np.meshgrid(vs, us, indexing="ij")
        z = depth.astype(np.float32)
        valid = (z > 0.05) & (z <= self._max_depth) & np.isfinite(z)
        if not np.any(valid):
            return

        uu, vv, z_cam = uu[valid], vv[valid], z[valid]
        x_cam = (uu - cx) * z_cam / fx
        y_cam = (vv - cy) * z_cam / fy
        pts = np.stack([x_cam, y_cam, z_cam, np.ones_like(z_cam)], axis=1)
        base = (tf_mat @ pts.T).T[:, :3]
        band = (base[:, 2] >= self._min_z) & (base[:, 2] <= self._max_z)
        if not np.any(band):
            return
        xy = base[band, :2]
        angles = np.arctan2(xy[:, 1], xy[:, 0])
        dists = np.hypot(xy[:, 0], xy[:, 1])
        bins = bin_points(angles, dists, self._angle_min, self._angle_max, self._num_bins)
        scan = scan_from_bins(
            msg.header.stamp,
            self._target,
            self._angle_min,
            self._angle_max,
            self._num_bins,
            bins,
            self._range_min,
            self._range_max,
        )
        self._pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthScanLayerNode()
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
