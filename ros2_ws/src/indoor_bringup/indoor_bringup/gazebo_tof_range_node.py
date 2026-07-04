#!/usr/bin/env python3
"""Convert Gazebo single-beam LaserScan topics to sensor_msgs/Range for Nav2 ToF layer."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class GazeboTofRangeNode(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_tof_range")
        self.declare_parameter("min_range_m", 0.04)
        self.declare_parameter("max_range_m", 4.0)
        self.declare_parameter("field_of_view", 0.47)
        self.declare_parameter("radiation_type", Range.INFRARED)

        self._min_r = float(self.get_parameter("min_range_m").value)
        self._max_r = float(self.get_parameter("max_range_m").value)
        self._fov = float(self.get_parameter("field_of_view").value)
        self._rad_type = int(self.get_parameter("radiation_type").value)

        specs = [
            ("front", "/tof_front/scan", "/tof_front", "tof_front_link"),
            ("left", "/tof_left/scan", "/tof_left", "tof_left_link"),
            ("right", "/tof_right/scan", "/tof_right", "tof_right_link"),
        ]
        for name, scan_topic, range_topic, frame in specs:
            pub = self.create_publisher(Range, range_topic, 10)
            self.create_subscription(
                LaserScan,
                scan_topic,
                lambda msg, p=pub, f=frame: self._on_scan(msg, p, f),
                10,
            )
            self.get_logger().info(f"ToF {name}: {scan_topic} -> {range_topic} ({frame})")

    def _on_scan(self, msg: LaserScan, pub, frame_id: str) -> None:
        dist = self._first_valid(msg)
        out = Range()
        out.header = msg.header
        out.header.frame_id = frame_id
        out.radiation_type = self._rad_type
        out.field_of_view = self._fov
        out.min_range = self._min_r
        out.max_range = self._max_r
        if dist is None:
            out.range = float("inf")
        else:
            out.range = dist
        pub.publish(out)

    @staticmethod
    def _first_valid(msg: LaserScan) -> Optional[float]:
        for r in msg.ranges:
            if math.isfinite(r) and r > 0:
                return float(r)
        return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboTofRangeNode()
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
