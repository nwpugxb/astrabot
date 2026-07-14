#!/usr/bin/env python3
"""Xbox teleop — simple D-pad drive, no deadman.

  D-pad up/down/left/right  → forward / back / turn
  Y                         → speed +
  A                         → speed -
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Joy

WHEEL_D_M = 0.0646
WHEEL_SEP_M = 0.208
COUNTS_PER_REV = 564.0
M_PER_COUNT = math.pi * WHEEL_D_M / COUNTS_PER_REV

JOY_HARD_STALE_S = 0.45
JOY_ARM_S = 0.15
PUB_HZ = 30.0
PUB_DT = 1.0 / PUB_HZ

GEAR_COUNTS = (30, 45, 60, 80)
DEFAULT_GEAR = 1

BTN_A = 0
BTN_Y_CANDIDATES = (3, 4)  # classic=3, some BT=4
AXIS_HAT_X = 6
AXIS_HAT_Y = 7
HAT_THRESH = 0.5

CMD_VEL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _mps(counts: float) -> float:
    return float(counts) * 10.0 * M_PER_COUNT


def _btn(msg: Joy, idx: int) -> bool:
    return 0 <= idx < len(msg.buttons) and msg.buttons[idx] == 1


def _hat(msg: Joy, idx: int) -> float:
    if idx >= len(msg.axes):
        return 0.0
    v = msg.axes[idx]
    if v < -HAT_THRESH:
        return -1.0
    if v > HAT_THRESH:
        return 1.0
    return 0.0


class DeckXboxTeleop(Node):
    def __init__(self) -> None:
        super().__init__("deck_xbox_teleop")
        self._pub = self.create_publisher(Twist, "/cmd_vel", CMD_VEL_QOS)
        self.create_subscription(Joy, "/joy", self._on_joy, qos_profile_sensor_data)

        self._cmd = Twist()
        self._last_joy_time = 0.0
        self._got_joy = False
        self._gear = DEFAULT_GEAR
        self._prev_y = False
        self._prev_a = False

        self._armed = False
        self._healthy_since: float | None = None
        self._was_hard_stale = True

        self.create_timer(PUB_DT, self._on_timer)
        self._log_gear("init")
        self.get_logger().info("Xbox teleop — D-pad drive, Y=+, A=- (no deadman)")

    def _log_gear(self, why: str) -> None:
        c = GEAR_COUNTS[self._gear]
        self.get_logger().info(
            f"Speed {self._gear+1}/{len(GEAR_COUNTS)} "
            f"({c} cnt/100ms ≈ {_mps(c):.2f} m/s) [{why}]"
        )

    def _disarm(self, reason: str) -> None:
        if self._armed:
            self.get_logger().warn(f"Link DISARM — {reason}")
        self._armed = False
        self._healthy_since = None
        self._was_hard_stale = True
        self._cmd = Twist()

    def _note_joy(self, now: float) -> None:
        if self._was_hard_stale or self._healthy_since is None:
            self._healthy_since = now
            self._was_hard_stale = False
        if not self._armed and self._healthy_since is not None:
            if (now - self._healthy_since) >= JOY_ARM_S:
                self._armed = True
                self.get_logger().info("Link ARMED")

    def _update_gear(self, msg: Joy) -> None:
        y = any(_btn(msg, i) for i in BTN_Y_CANDIDATES)
        a = _btn(msg, BTN_A)
        if y and not self._prev_y:
            if self._gear < len(GEAR_COUNTS) - 1:
                self._gear += 1
                self._log_gear("Y")
            else:
                self.get_logger().info("Already max speed (Y)")
        if a and not self._prev_a:
            if self._gear > 0:
                self._gear -= 1
                self._log_gear("A")
            else:
                self.get_logger().info("Already min speed (A)")
        self._prev_y = y
        self._prev_a = a

    def _joy_to_twist(self, msg: Joy) -> Twist:
        self._update_gear(msg)
        if not self._armed:
            return Twist()

        # hat: up=-1 → forward; left=-1 → +yaw
        lin_dir = -_hat(msg, AXIS_HAT_Y)
        ang_dir = -_hat(msg, AXIS_HAT_X)

        twist = Twist()
        if abs(lin_dir) > 0.0 or abs(ang_dir) > 0.0:
            lin = _mps(GEAR_COUNTS[self._gear])
            ang = 2.0 * lin / WHEEL_SEP_M
            twist.linear.x = lin_dir * lin
            twist.angular.z = ang_dir * ang
        return twist

    def _on_joy(self, msg: Joy) -> None:
        now = time.monotonic()
        self._got_joy = True
        self._last_joy_time = now
        self._note_joy(now)
        self._cmd = self._joy_to_twist(msg)
        self._pub.publish(self._cmd)

    def _on_timer(self) -> None:
        if not self._got_joy:
            return
        now = time.monotonic()
        if now - self._last_joy_time >= JOY_HARD_STALE_S:
            self._disarm("joy timeout")
        self._pub.publish(self._cmd)


def main() -> None:
    rclpy.init()
    node = DeckXboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
