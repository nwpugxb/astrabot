#!/usr/bin/env python3
"""Project depth points in a base height band onto a plane for 2D-style wall ICP."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import tf2_ros


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), 0.0],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), 0.0],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


class WallPlaneCloudNode(Node):
    """Keep points with z in [min_z, max_z] (base frame) and flatten them to plane_z."""

    def __init__(self) -> None:
        super().__init__("wall_plane_cloud")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("use_static_tf_only", True)
        self.declare_parameter("min_z_m", -0.10)
        self.declare_parameter("max_z_m", 0.50)
        self.declare_parameter("plane_z_m", 0.20)
        self.declare_parameter("stride", 2)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("output_topic", "wall_plane_cloud")

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._static_tf_only = bool(self.get_parameter("use_static_tf_only").value)
        self._min_z = float(self.get_parameter("min_z_m").value)
        self._max_z = float(self.get_parameter("max_z_m").value)
        self._plane_z = float(self.get_parameter("plane_z_m").value)
        self._stride = max(1, int(self.get_parameter("stride").value))
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        out_topic = str(self.get_parameter("output_topic").value)

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(PointCloud2, out_topic, 10)

        depth_sub = Subscriber(self, Image, "depth/image_raw")
        info_sub = Subscriber(self, CameraInfo, "depth/camera_info")
        self._sync = ApproximateTimeSynchronizer(
            [depth_sub, info_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_depth)
        self.get_logger().info(
            f"Publishing {out_topic} in {self._target_frame}: "
            f"z in [{self._min_z:.2f}, {self._max_z:.2f}] -> plane z={self._plane_z:.2f}"
        )
        self._published = 0

    def _depth_to_meters(self, depth_msg: Image) -> np.ndarray:
        enc = (depth_msg.encoding or "").lower()
        if enc in ("32fc1", "32fc"):
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            return depth.astype(np.float32)
        if enc in ("16uc1", "mono16"):
            raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            return raw.astype(np.float32) / 1000.0
        depth = self._bridge.imgmsg_to_cv2(depth_msg)
        return depth.astype(np.float32)

    def _lookup_matrix(self, target: str, source: str, stamp: Time) -> Optional[np.ndarray]:
        lookup_time = rclpy.time.Time() if self._static_tf_only else stamp
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                target,
                source,
                lookup_time,
                timeout=rclpy.duration.Duration(seconds=0.15),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f"TF {source}->{target}: {exc}", throttle_duration_sec=5.0
            )
            return None
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        mat = _quat_to_matrix(q.x, q.y, q.z, q.w)
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        return mat

    def _on_depth(self, depth_msg: Image, info_msg: CameraInfo) -> None:
        stamp = Time.from_msg(depth_msg.header.stamp)
        cam_frame = depth_msg.header.frame_id or info_msg.header.frame_id
        if not cam_frame:
            return
        tf_mat = self._lookup_matrix(self._target_frame, cam_frame, stamp)
        if tf_mat is None:
            return

        depth = self._depth_to_meters(depth_msg)
        s = self._stride
        depth = depth[::s, ::s]
        h, w = depth.shape

        fx = info_msg.k[0] / s
        fy = info_msg.k[4] / s
        cx = info_msg.k[2] / s
        cy = info_msg.k[5] / s

        vs = np.arange(h, dtype=np.float32) + 0.5
        us = np.arange(w, dtype=np.float32) + 0.5
        vv, uu = np.meshgrid(vs, us, indexing="ij")

        z = depth.astype(np.float32)
        valid = (z > 0.05) & (z <= self._max_depth) & np.isfinite(z)
        if not np.any(valid):
            return

        u_valid = uu[valid]
        v_valid = vv[valid]
        z_cam = z[valid]
        x_cam = (u_valid - cx) * z_cam / fx
        y_cam = (v_valid - cy) * z_cam / fy
        pts_cam = np.stack([x_cam, y_cam, z_cam, np.ones_like(z_cam)], axis=1)
        pts_base = (tf_mat @ pts_cam.T).T[:, :3]

        band = (pts_base[:, 2] >= self._min_z) & (pts_base[:, 2] <= self._max_z)
        if not np.any(band):
            zmin = float(pts_base[:, 2].min()) if len(pts_base) else float("nan")
            zmax = float(pts_base[:, 2].max()) if len(pts_base) else float("nan")
            self.get_logger().warn(
                f"No points in z band [{self._min_z}, {self._max_z}] "
                f"(valid={int(valid.sum())}, z=[{zmin:.2f},{zmax:.2f}])",
                throttle_duration_sec=5.0,
            )
            return
        pts = pts_base[band]
        pts[:, 2] = self._plane_z

        header = depth_msg.header
        header.frame_id = "base_footprint"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(
            header, fields, pts.astype(np.float32).tolist()
        )
        self._pub.publish(cloud)
        self._published += 1
        if self._published <= 3 or self._published % 200 == 0:
            self.get_logger().info(
                f"wall_plane_cloud #{self._published}: {len(pts)} points"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WallPlaneCloudNode()
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
