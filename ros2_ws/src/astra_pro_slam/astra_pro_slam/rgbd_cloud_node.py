#!/usr/bin/env python3
"""Fuse synchronized RGB + depth into a colored PointCloud2 for live RViz preview."""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class RgbdCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("rgbd_cloud")
        self.declare_parameter("stride", 2)
        self.declare_parameter("max_depth_m", 6.0)
        self._stride = max(1, int(self.get_parameter("stride").value))
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._bridge = CvBridge()
        self._pub = self.create_publisher(PointCloud2, "cloud_colored", 10)

        color_sub = Subscriber(self, Image, "color/image_raw")
        depth_sub = Subscriber(self, Image, "depth/image_raw")
        info_sub = Subscriber(self, CameraInfo, "color/camera_info")
        self._sync = ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_rgbd)
        self.get_logger().info(
            f"Publishing colored cloud on cloud_colored (stride={self._stride})"
        )

    def _on_rgbd(self, color_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")

        s = self._stride
        color = color[::s, ::s]
        depth = depth[::s, ::s]
        h, w = depth.shape

        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        vs = np.arange(0, h * s, s)
        us = np.arange(0, w * s, s)
        vv, uu = np.meshgrid(vs, us, indexing="ij")

        z = depth
        valid = (z > 0.0) & (z <= self._max_depth) & np.isfinite(z)
        if not valid.any():
            return

        u_valid = uu[valid].astype(np.float32)
        v_valid = vv[valid].astype(np.float32)
        z_valid = z[valid]

        x = (u_valid - cx) * z_valid / fx
        y = (v_valid - cy) * z_valid / fy

        bgr = color[valid]
        rgb = (
            bgr[:, 2].astype(np.uint32) << 16
            | bgr[:, 1].astype(np.uint32) << 8
            | bgr[:, 0].astype(np.uint32)
        )

        header = color_msg.header
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        points = zip(x.tolist(), y.tolist(), z_valid.tolist(), rgb.tolist())
        cloud = point_cloud2.create_cloud(header, fields, points)
        self._pub.publish(cloud)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RgbdCloudNode()
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
