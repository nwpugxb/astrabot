#!/usr/bin/env python3
"""Indoor inspection patrol: loop through waypoints via Nav2, run action at each stop.

Requires Nav2 running (nav2.launch.py) and AMCL localized on the map.
Uses nav2_simple_commander BasicNavigator (goToPose per waypoint).

Example:
  ros2 launch indoor_bringup patrol.launch.py
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not data or "patrol" not in data:
        raise ValueError(f"Missing top-level 'patrol' key in {path}")
    return data["patrol"]


def _pose_from_xy_yaw(navigator: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_quaternion(float(yaw))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class PatrolRunner:
    """Navigate waypoints one-by-one; execute a simple action at each arrival."""

    def __init__(
        self,
        navigator: BasicNavigator,
        cfg: dict[str, Any],
        goal_timeout_s: float = 120.0,
        retry_count: int = 1,
    ) -> None:
        self._nav = navigator
        self._cfg = cfg
        self._goal_timeout = goal_timeout_s
        self._retry_count = retry_count
        self._capture_pub = navigator.create_publisher(String, "patrol/capture_request", 10)
        self._log = navigator.get_logger()

    def _waypoints(self) -> list[dict[str, Any]]:
        wps = self._cfg.get("waypoints") or []
        if not wps:
            raise ValueError("No waypoints defined in patrol config")
        return wps

    def _default_dwell(self) -> float:
        return float(self._cfg.get("default_dwell_s", 2.0))

    def _home_pose(self) -> Optional[PoseStamped]:
        home = self._cfg.get("home")
        if not home:
            return None
        return _pose_from_xy_yaw(
            self._nav,
            float(home.get("x", 0.0)),
            float(home.get("y", 0.0)),
            float(home.get("yaw", 0.0)),
        )

    def _run_action(self, wp: dict[str, Any]) -> None:
        name = str(wp.get("name", "?"))
        action = str(wp.get("action", "wait")).lower()
        dwell = float(wp.get("dwell_s", self._default_dwell()))

        if action == "capture":
            msg = String()
            msg.data = name
            self._capture_pub.publish(msg)
            self._log.info(f"[{name}] capture_request published (hook camera node here)")
        elif action == "log":
            self._log.info(f"[{name}] inspection log (no-op stub)")
        else:
            self._log.info(f"[{name}] wait {dwell:.1f}s")

        if dwell > 0 and action != "log":
            time.sleep(dwell)

    def _go_to_pose(self, pose: PoseStamped, label: str) -> bool:
        for attempt in range(self._retry_count + 1):
            if attempt > 0:
                self._log.warn(f"Retry {attempt}/{self._retry_count} -> {label}")
            pose.header.stamp = self._nav.get_clock().now().to_msg()
            self._nav.goToPose(pose)
            deadline = time.monotonic() + self._goal_timeout
            while not self._nav.isTaskComplete():
                if time.monotonic() > deadline:
                    self._log.error(f"Timeout navigating to {label}")
                    self._nav.cancelTask()
                    break
                time.sleep(0.1)
            result = self._nav.getResult()
            if result == TaskResult.SUCCEEDED:
                return True
            self._log.warn(f"Navigation to {label} failed: {result}")
        return False

    def run(self) -> int:
        loop = bool(self._cfg.get("loop", True))
        max_rounds = int(self._cfg.get("max_rounds", 0))
        wps = self._waypoints()
        home = self._home_pose()
        round_idx = 0

        self._log.info(
            f"Patrol starting: {len(wps)} waypoints, loop={loop}, "
            f"max_rounds={max_rounds or 'inf'}"
        )

        while rclpy.ok():
            round_idx += 1
            self._log.info(f"=== Patrol round {round_idx} ===")
            for i, wp in enumerate(wps):
                name = str(wp.get("name", f"wp_{i}"))
                pose = _pose_from_xy_yaw(
                    self._nav,
                    float(wp["x"]),
                    float(wp["y"]),
                    float(wp.get("yaw", 0.0)),
                )
                self._log.info(f"Going to {name} ({i + 1}/{len(wps)})")
                if not self._go_to_pose(pose, name):
                    self._log.error(f"Skipping actions at {name} after nav failure")
                    continue
                self._run_action(wp)

            if home is not None:
                self._log.info("Returning home")
                if not self._go_to_pose(home, "home"):
                    self._log.error("Failed to return home")

            if not loop:
                break
            if max_rounds > 0 and round_idx >= max_rounds:
                self._log.info(f"Completed {max_rounds} round(s), stopping")
                break

        self._log.info("Patrol finished")
        return 0


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    navigator: Optional[BasicNavigator] = None

    try:
        loader = Node("patrol_param_loader")
        loader.declare_parameter("waypoints_file", "")
        loader.declare_parameter("goal_timeout_s", 120.0)
        loader.declare_parameter("retry_count", 1)
        wp_file = str(loader.get_parameter("waypoints_file").value)
        goal_timeout = float(loader.get_parameter("goal_timeout_s").value)
        retry_count = int(loader.get_parameter("retry_count").value)
        loader.destroy_node()

        if not wp_file:
            print("Set waypoints_file to patrol_waypoints.yaml", file=sys.stderr)
            raise SystemExit(1)

        cfg = _load_yaml(wp_file)
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()

        runner = PatrolRunner(navigator, cfg, goal_timeout, retry_count)
        sys.exit(runner.run())
    except KeyboardInterrupt:
        pass
    finally:
        if navigator is not None:
            navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
