#!/usr/bin/env python3
"""Plot wheel odometry trajectory (live ROS topic or from a rosbag).

Examples:
  # Live while mapping / teleop (source ROS first)
  ./scripts/plot_odom_trajectory.py

  # From recorded bag
  ./scripts/plot_odom_trajectory.py --bag output/bags/mobile_20250623_120000

  # Save PNG on exit
  ./scripts/plot_odom_trajectory.py --bag ... --save output/odom_traj.png
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
from typing import List, Tuple

import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class LiveOdomCollector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("odom_plotter")
        self._xs: List[float] = []
        self._ys: List[float] = []
        self._lock = threading.Lock()
        self.create_subscription(Odometry, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        with self._lock:
            if self._xs:
                dx = x - self._xs[-1]
                dy = y - self._ys[-1]
                if math.hypot(dx, dy) < 0.005:
                    return
            self._xs.append(x)
            self._ys.append(y)

    def snapshot(self) -> Tuple[List[float], List[float]]:
        with self._lock:
            return list(self._xs), list(self._ys)


def load_odom_from_bag(bag_dir: str, topic: str = "/odom") -> Tuple[List[float], List[float]]:
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("Need ROS2 Python packages (source /opt/ros/humble/setup.bash)") from exc

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise RuntimeError(f"Topic {topic} not in bag. Available: {sorted(topics)}")

    msg_type = get_message(topics[topic])
    xs: List[float] = []
    ys: List[float] = []
    while reader.has_next():
        name, data, _stamp = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        xs.append(msg.pose.pose.position.x)
        ys.append(msg.pose.pose.position.y)
    return xs, ys


def draw_trajectory(xs: List[float], ys: List[float], title: str, save: str | None) -> None:
    if not xs:
        print("No odometry points.", file=sys.stderr)
        return

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(xs, ys, "-b", linewidth=1.5, label="wheel odom")
    ax.plot(xs[0], ys[0], "go", markersize=10, label="start")
    ax.plot(xs[-1], ys[-1], "rs", markersize=10, label="end")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X in odom (m)")
    ax.set_ylabel("Y in odom (m)")
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    if save:
        fig.savefig(save, dpi=150)
        print(f"Saved: {save}")
    plt.show()


def run_live(topic: str, save: str | None) -> None:
    rclpy.init()
    node = LiveOdomCollector(topic)
    spin_stop = threading.Event()

    def spin() -> None:
        while not spin_stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    threading.Thread(target=spin, daemon=True).start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    print(f"Listening {topic} — close plot window to exit.")
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            xs, ys = node.snapshot()
            ax.clear()
            if xs:
                ax.plot(xs, ys, "-b", linewidth=1.5)
                ax.plot(xs[0], ys[0], "go", markersize=8)
                ax.plot(xs[-1], ys[-1], "rs", markersize=8)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("X in odom (m)")
            ax.set.ylabel("Y in odom (m)")
            ax.set_title(f"Wheel odometry ({len(xs)} points)")
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        spin_stop.set()
        xs, ys = node.snapshot()
        if save and xs:
            draw_trajectory(xs, ys, "Wheel odometry", save)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot /odom trajectory")
    parser.add_argument("--bag", help="Read /odom from ros2 bag directory")
    parser.add_argument("--topic", default="/odom", help="Odometry topic (default: /odom)")
    parser.add_argument("--save", help="Save PNG on exit")
    args = parser.parse_args()

    if args.bag:
        xs, ys = load_odom_from_bag(args.bag, args.topic)
        draw_trajectory(xs, ys, f"Wheel odom from bag ({len(xs)} pts)", args.save)
    else:
        run_live(args.topic, args.save)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
