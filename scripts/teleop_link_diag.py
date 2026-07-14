#!/usr/bin/env python3
"""Teleop link diagnosis: measure /joy, /cmd_vel and /odom inter-arrival gaps.

Run while driving with the Xbox controller. Prints per-second stats:
  joy   — controller → PC (Bluetooth health)
  cmd   — teleop node output (should be steady ~30 Hz)
  odom  — ESP32 → PC over WiFi (proxy for WiFi health both ways)

Interpretation:
  joy max gap large (>300 ms)  → Bluetooth problem (controller side)
  joy fine, odom max gap large → WiFi congestion (lidar stream / ESP32 side)
  everything fine but robot stops → ESP32 firmware timeout / motor side

Usage:
  source /opt/ros/humble/setup.bash && python3 scripts/teleop_link_diag.py
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


class TopicStat:
    def __init__(self, name: str) -> None:
        self.name = name
        self.count = 0
        self.last_t: float | None = None
        self.max_gap = 0.0

    def tick(self) -> None:
        now = time.monotonic()
        if self.last_t is not None:
            gap = now - self.last_t
            if gap > self.max_gap:
                self.max_gap = gap
        self.last_t = now
        self.count += 1

    def report_and_reset(self) -> str:
        # Include gap since last message so full stalls are visible too.
        now = time.monotonic()
        open_gap = (now - self.last_t) if self.last_t is not None else float("inf")
        eff_gap = max(self.max_gap, open_gap if self.count == 0 else self.max_gap)
        s = (
            f"{self.name}: {self.count:3d} Hz, max gap "
            f"{int(eff_gap * 1000) if eff_gap != float('inf') else -1:5d} ms"
        )
        self.count = 0
        self.max_gap = 0.0
        return s


class LinkDiag(Node):
    def __init__(self) -> None:
        super().__init__("teleop_link_diag")
        self._joy = TopicStat("joy ")
        self._cmd = TopicStat("cmd ")
        self._odom = TopicStat("odom")

        self.create_subscription(
            Joy, "/joy", lambda _m: self._joy.tick(), qos_profile_sensor_data)
        self.create_subscription(
            Twist, "/cmd_vel", lambda _m: self._cmd.tick(), qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/odom", lambda _m: self._odom.tick(), qos_profile_sensor_data)

        self.create_timer(1.0, self._report)
        self.get_logger().info(
            "Link diag running — drive around; watch max gaps. "
            "joy=Bluetooth, odom=WiFi/ESP32, cmd=teleop node"
        )

    def _report(self) -> None:
        line = " | ".join([
            self._joy.report_and_reset(),
            self._cmd.report_and_reset(),
            self._odom.report_and_reset(),
        ])
        flags = []
        self.get_logger().info(line + ("  " + " ".join(flags) if flags else ""))


def main() -> None:
    rclpy.init()
    node = LinkDiag()
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
