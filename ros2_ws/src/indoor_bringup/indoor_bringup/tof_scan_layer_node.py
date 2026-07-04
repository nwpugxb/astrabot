#!/usr/bin/env python3
"""Mapping layer: VL53L1X Range x3 -> sparse LaserScan for near-field low obstacles."""

from __future__ import annotations

import math
from typing import Dict
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
import tf2_ros

from indoor_bringup.microros_qos import MICROROS_QOS
from indoor_bringup.laser_utils import bin_points, scan_from_bins


class TofScanLayerNode(Node):
    def __init__(self) -> None:
        super().__init__("tof_scan_layer")
        self.declare_parameter("topics", ["/tof_front", "/tof_left", "/tof_right"])
        self.declare_parameter("output_topic", "/mapping/layers/tof_scan")
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("num_bins", 720)
        self.declare_parameter("angle_min", -3.14159)
        self.declare_parameter("angle_max", 3.14159)
        self.declare_parameter("range_min", 0.04)
        self.declare_parameter("range_max", 4.0)
        self.declare_parameter("default_fov_rad", 0.47)

        self._target = str(self.get_parameter("target_frame").value)
        self._num_bins = int(self.get_parameter("num_bins").value)
        self._angle_min = float(self.get_parameter("angle_min").value)
        self._angle_max = float(self.get_parameter("angle_max").value)
        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._default_fov = float(self.get_parameter("default_fov_rad").value)
        out = str(self.get_parameter("output_topic").value)

        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(LaserScan, out, 10)
        self._readings: Dict[str, Range] = {}

        topics = self.get_parameter("topics").value
        for t in topics:
            self.create_subscription(
                Range, str(t), lambda m, tp=str(t): self._on_range(m, tp), MICROROS_QOS
            )

        self.create_timer(0.1, self._publish_scan)
        self.get_logger().info(f"ToF scan layer -> {out} from {list(topics)}")

    def _on_range(self, msg: Range, topic: str) -> None:
        self._readings[topic] = msg

    def _beam_angles(self, msg: Range) -> tuple[list[float], list[float]]:
        """Ray in target frame: center bearing + spread from sensor FOV."""
        frame = msg.header.frame_id
        if not frame:
            return [], []
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target,
                frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except tf2_ros.TransformException:
            return [], []

        # Sensor +X is forward in REP-103 link frames.
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        fov = msg.field_of_view if msg.field_of_view > 0.01 else self._default_fov
        half = fov * 0.5
        r = msg.range
        if not math.isfinite(r) or r < msg.min_range or r > msg.max_range:
            return [], []

        # Mark center and edges of beam (3 samples per ToF).
        angles = [yaw - half, yaw, yaw + half]
        dists = [r, r, r]
        return angles, dists

    def _publish_scan(self) -> None:
        if not self._readings:
            return
        all_angles: list[float] = []
        all_dists: list[float] = []
        for msg in self._readings.values():
            a, d = self._beam_angles(msg)
            all_angles.extend(a)
            all_dists.extend(d)
        if not all_angles:
            return

        bins = bin_points(
            all_angles, all_dists, self._angle_min, self._angle_max, self._num_bins
        )
        stamp = self.get_clock().now().to_msg()
        scan = scan_from_bins(
            stamp,
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
    node = TofScanLayerNode()
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
