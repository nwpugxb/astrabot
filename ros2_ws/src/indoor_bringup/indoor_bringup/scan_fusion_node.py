#!/usr/bin/env python3
"""Fuse mapping scan layers (A1 + depth + ToF) into /scan_fused for slam_toolbox."""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from indoor_bringup.laser_utils import merge_scan_bins, scan_from_bins, scan_to_bins


class ScanFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("scan_fusion")
        self.declare_parameter("use_lidar", True)
        self.declare_parameter("use_depth", True)
        self.declare_parameter("use_tof", True)
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("depth_topic", "/mapping/layers/depth_scan")
        self.declare_parameter("tof_topic", "/mapping/layers/tof_scan")
        self.declare_parameter("output_topic", "/scan_fused")
        self.declare_parameter("mirror_lidar_topic", "/mapping/layers/lidar_scan")
        self.declare_parameter("num_bins", 720)
        self.declare_parameter("angle_min", -3.14159)
        self.declare_parameter("angle_max", 3.14159)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 12.0)

        self._use_lidar = bool(self.get_parameter("use_lidar").value)
        self._use_depth = bool(self.get_parameter("use_depth").value)
        self._use_tof = bool(self.get_parameter("use_tof").value)
        self._num_bins = int(self.get_parameter("num_bins").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)

        out = str(self.get_parameter("output_topic").value)
        mirror = str(self.get_parameter("mirror_lidar_topic").value)
        self._pub = self.create_publisher(LaserScan, out, 10)
        self._pub_lidar = self.create_publisher(LaserScan, mirror, 10)

        self._bins: Dict[str, Optional[np.ndarray]] = {
            "lidar": None,
            "depth": None,
            "tof": None,
        }
        self._last_stamp = self.get_clock().now().to_msg()

        if self._use_lidar:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("lidar_topic").value),
                self._on_lidar,
                10,
            )
        if self._use_depth:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("depth_topic").value),
                lambda m: self._on_layer("depth", m),
                10,
            )
        if self._use_tof:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("tof_topic").value),
                lambda m: self._on_layer("tof", m),
                10,
            )

        self.create_timer(0.05, self._publish_fused)
        enabled = [k for k, on in [("lidar", self._use_lidar), ("depth", self._use_depth), ("tof", self._use_tof)] if on]
        self.get_logger().info(f"Scan fusion -> {out}, layers={enabled}")

    def _on_lidar(self, msg: LaserScan) -> None:
        self._bins["lidar"] = scan_to_bins(msg, self._num_bins)
        self._last_stamp = msg.header.stamp
        mirror = scan_from_bins(
            msg.header.stamp,
            msg.header.frame_id,
            self._angle_min,
            self._angle_max,
            self._num_bins,
            self._bins["lidar"],
            self._range_min,
            self._range_max,
        )
        self._pub_lidar.publish(mirror)

    def _on_layer(self, name: str, msg: LaserScan) -> None:
        self._bins[name] = scan_to_bins(msg, self._num_bins)
        self._last_stamp = msg.header.stamp

    def _publish_fused(self) -> None:
        layers = []
        for key in ("lidar", "depth", "tof"):
            b = self._bins.get(key)
            if b is not None:
                layers.append(b)
        if not layers:
            return
        fused = merge_scan_bins(layers)
        scan = scan_from_bins(
            self._last_stamp,
            "base_footprint",
            self._angle_min,
            self._angle_max,
            self._num_bins,
            fused,
            self._range_min,
            self._range_max,
        )
        self._pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanFusionNode()
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
