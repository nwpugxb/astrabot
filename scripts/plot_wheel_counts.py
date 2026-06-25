#!/usr/bin/env python3
"""Plot wheel encoder counts / speeds vs time (Arduino PID-full telemetry).

Data format (100 ms per line from firmware):
  targetR, actualR, pwmR, targetL, actualL, pwmL, speedR_mm_s, speedL_mm_s, faultR, faultL

Examples:
  # Live from USB serial (robot connected, mapping/teleop running or standalone Arduino)
  ./scripts/plot_wheel_counts.py

  # Live from ROS topic (arduino_base_node running)
  ./scripts/plot_wheel_counts.py --source ros

  # From saved log (one CSV line per telemetry row)
  ./scripts/plot_wheel_counts.py --source log --log encoder.log

  # From bag IF /arduino_feedback was recorded (older bags only have /odom)
  ./scripts/plot_wheel_counts.py --bag output/bags/mobile_xxx

  ./scripts/plot_wheel_counts.py --save output/wheel_counts.png
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import matplotlib.pyplot as plt

TELEMETRY_RE = re.compile(
    r"^\s*"
    r"(-?\d+(?:\.\d*)?),"
    r"\s*(\d+(?:\.\d*)?),"
    r"\s*(\d+(?:\.\d*)?),"
    r"\s*(-?\d+(?:\.\d*)?),"
    r"\s*(\d+(?:\.\d*)?),"
    r"\s*(\d+(?:\.\d*)?),"
    r"\s*(-?\d+(?:\.\d*)?),"
    r"\s*(-?\d+(?:\.\d*)?),"
)


@dataclass
class WheelSample:
    t: float
    target_r: float
    count_r: float
    target_l: float
    count_l: float
    speed_r: float
    speed_l: float


def parse_telemetry_line(line: str) -> Optional[WheelSample]:
    line = line.strip()
    if not line or line.startswith("Dual wheel") or line.startswith("Commands"):
        return None
    if line.startswith("targetR") or line.startswith("Speed range") or line.startswith("STALL"):
        return None
    m = TELEMETRY_RE.match(line)
    if not m:
        return None
    target_r, count_r, _pwm_r, target_l, count_l, _pwm_l, speed_r, speed_l = (
        float(m.group(i)) for i in range(1, 9)
    )
    # Encoders only count up; apply sign from target (same as arduino_base_node).
    sign_r = 0.0 if target_r == 0 else (1.0 if target_r > 0 else -1.0)
    sign_l = 0.0 if target_l == 0 else (1.0 if target_l > 0 else -1.0)
    return WheelSample(
        t=0.0,
        target_r=target_r,
        count_r=sign_r * abs(count_r),
        target_l=target_l,
        count_l=sign_l * abs(count_l),
        speed_r=speed_r,
        speed_l=speed_l,
    )


def stamp_times(samples: List[WheelSample], dt: float = 0.1) -> None:
    for i, s in enumerate(samples):
        s.t = i * dt


def load_from_log(path: str) -> List[WheelSample]:
    samples: List[WheelSample] = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            row = parse_telemetry_line(line)
            if row:
                samples.append(row)
    stamp_times(samples)
    return samples


def load_from_bag(bag_dir: str, topic: str = "/arduino_feedback") -> List[WheelSample]:
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("Source ROS first: source /opt/ros/humble/setup.bash") from exc

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_dir, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in topics:
        raise RuntimeError(
            f"{topic} not in bag. Available: {sorted(topics)}\n"
            "Older mobile bags only record /odom. Re-record with /arduino_feedback, "
            "or use --source serial / --source ros while driving."
        )

    msg_type = get_message(topics[topic])
    samples: List[WheelSample] = []
    t0_ns: Optional[int] = None
    while reader.has_next():
        name, data, stamp_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, msg_type)
        row = parse_telemetry_line(msg.data)
        if not row:
            continue
        if t0_ns is None:
            t0_ns = stamp_ns
        row.t = (stamp_ns - t0_ns) / 1e9
        samples.append(row)
    return samples


class SerialReader:
    def __init__(self, port: str, baud: int) -> None:
        import serial

        self._ser = serial.Serial(port, baud, timeout=0.05)
        time.sleep(0.5)
        self._ser.reset_input_buffer()

    def readline(self) -> str:
        raw = self._ser.readline()
        return raw.decode("ascii", errors="ignore").strip()

    def close(self) -> None:
        if self._ser.is_open:
            self._ser.close()


class RosFeedbackReader:
    def __init__(self, topic: str) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        self._rclpy = rclpy
        rclpy.init()
        self._node = Node("wheel_count_plotter")
        self._lock = threading.Lock()
        self._samples: List[WheelSample] = []
        self._t0: Optional[float] = None
        self._node.create_subscription(String, topic, self._cb, 10)
        self._spin_stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._spin_stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def _cb(self, msg) -> None:
        row = parse_telemetry_line(msg.data)
        if not row:
            return
        now = time.monotonic()
        if self._t0 is None:
            self._t0 = now
        row.t = now - self._t0
        with self._lock:
            self._samples.append(row)

    def snapshot(self) -> List[WheelSample]:
        with self._lock:
            return list(self._samples)

    def close(self) -> None:
        self._spin_stop.set()
        self._thread.join(timeout=1.0)
        self._node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


def draw_samples(samples: List[WheelSample], title: str, save: Optional[str]) -> None:
    if not samples:
        print("No wheel telemetry samples.", file=sys.stderr)
        return

    ts = [s.t for s in samples]
    cr = [s.count_r for s in samples]
    cl = [s.count_l for s in samples]
    sr = [s.speed_r for s in samples]
    sl = [s.speed_l for s in samples]
    tr = [s.target_r for s in samples]
    tl = [s.target_l for s in samples]

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    ax0 = axes[0]
    ax0.plot(ts, cr, "-", color="#e74c3c", linewidth=1.2, label="right count / 100ms")
    ax0.plot(ts, cl, "-", color="#3498db", linewidth=1.2, label="left count / 100ms")
    ax0.plot(ts, tr, "--", color="#e74c3c", alpha=0.45, linewidth=0.9, label="right target")
    ax0.plot(ts, tl, "--", color="#3498db", alpha=0.45, linewidth=0.9, label="left target")
    ax0.axhline(0.0, color="gray", linewidth=0.6)
    ax0.set_ylabel("encoder counts / 100ms")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right", fontsize=9)
    ax0.set_title(f"{title} — {len(samples)} samples")

    ax1 = axes[1]
    ax1.plot(ts, sr, "-", color="#e74c3c", linewidth=1.2, label="right speed (mm/s)")
    ax1.plot(ts, sl, "-", color="#3498db", linewidth=1.2, label="left speed (mm/s)")
    ax1.axhline(0.0, color="gray", linewidth=0.6)
    ax1.set_xlabel("time (s)")
    ax1.set_ylabel("wheel speed (mm/s)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)

    # Quick diagnosis: if L/R counts always equal while targets differ, turns become straight lines.
    diffs = [abs(a - b) for a, b in zip(cr, cl)]
    tgt_diffs = [abs(a - b) for a, b in zip(tr, tl)]
    if max(tgt_diffs, default=0) > 1.0 and max(diffs, default=0) < 0.5:
        fig.text(
            0.5,
            0.01,
            "WARNING: target L/R differ but actual counts stay equal → odom cannot turn.",
            ha="center",
            color="red",
            fontsize=10,
        )

    fig.tight_layout(rect=(0, 0.03, 1, 1))
    if save:
        fig.savefig(save, dpi=150)
        print(f"Saved: {save}")
    plt.show()


def run_live_serial(port: str, baud: int, save: Optional[str]) -> None:
    reader = SerialReader(port, baud)
    samples: List[WheelSample] = []
    t0 = time.monotonic()
    plt.ion()
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    print(f"Reading {port} @ {baud}. Close plot window or Ctrl+C to exit.")
    try:
        while plt.fignum_exists(fig.number):
            line = reader.readline()
            if line:
                row = parse_telemetry_line(line)
                if row:
                    row.t = time.monotonic() - t0
                    samples.append(row)
            if not samples:
                plt.pause(0.02)
                continue

            ts = [s.t for s in samples]
            cr = [s.count_r for s in samples]
            cl = [s.count_l for s in samples]
            sr = [s.speed_r for s in samples]
            sl = [s.speed_l for s in samples]

            for ax in axes:
                ax.clear()
                ax.grid(True, alpha=0.3)
                ax.axhline(0.0, color="gray", linewidth=0.6)

            axes[0].plot(ts, cr, "-", color="#e74c3c", label="right count")
            axes[0].plot(ts, cl, "-", color="#3498db", label="left count")
            axes[0].set_ylabel("counts / 100ms")
            axes[0].legend(loc="upper right", fontsize=9)
            axes[0].set_title(f"Live wheel telemetry ({len(samples)} samples)")

            axes[1].plot(ts, sr, "-", color="#e74c3c", label="right speed")
            axes[1].plot(ts, sl, "-", color="#3498db", label="left speed")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("mm/s")
            axes[1].legend(loc="upper right", fontsize=9)

            plt.pause(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        if save and samples:
            draw_samples(samples, "Wheel telemetry (serial)", save)


def run_live_ros(topic: str, save: Optional[str]) -> None:
    reader = RosFeedbackReader(topic)
    plt.ion()
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    print(f"Listening {topic}. Close plot window or Ctrl+C to exit.")
    try:
        while plt.fignum_exists(fig.number):
            samples = reader.snapshot()
            if not samples:
                plt.pause(0.1)
                continue

            ts = [s.t for s in samples]
            cr = [s.count_r for s in samples]
            cl = [s.count_l for s in samples]
            sr = [s.speed_r for s in samples]
            sl = [s.speed_l for s in samples]

            for ax in axes:
                ax.clear()
                ax.grid(True, alpha=0.3)
                ax.axhline(0.0, color="gray", linewidth=0.6)

            axes[0].plot(ts, cr, "-", color="#e74c3c", label="right count")
            axes[0].plot(ts, cl, "-", color="#3498db", label="left count")
            axes[0].set_ylabel("counts / 100ms")
            axes[0].legend(loc="upper right", fontsize=9)
            axes[0].set_title(f"Live wheel telemetry ({len(samples)} samples)")

            axes[1].plot(ts, sr, "-", color="#e74c3c", label="right speed")
            axes[1].plot(ts, sl, "-", color="#3498db", label="left speed")
            axes[1].set_xlabel("time (s)")
            axes[1].set_ylabel("mm/s")
            axes[1].legend(loc="upper right", fontsize=9)

            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        samples = reader.snapshot()
        reader.close()
        if save and samples:
            draw_samples(samples, "Wheel telemetry (ROS)", save)


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot Arduino wheel counts / speeds vs time")
    parser.add_argument(
        "--source",
        choices=("serial", "ros", "log", "bag"),
        default="serial",
        help="serial=USB, ros=/arduino_feedback, log=text file, bag=rosbag2",
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port (serial mode)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--topic", default="/arduino_feedback", help="ROS String topic")
    parser.add_argument("--log", help="Text log with one telemetry CSV line per row")
    parser.add_argument("--bag", help="Rosbag2 directory")
    parser.add_argument("--save", help="Save PNG on exit")
    args = parser.parse_args()

    if args.source == "serial":
        run_live_serial(args.port, args.baud, args.save)
    elif args.source == "ros":
        run_live_ros(args.topic, args.save)
    elif args.source == "log":
        if not args.log:
            parser.error("--log required for --source log")
        samples = load_from_log(args.log)
        draw_samples(samples, f"Wheel telemetry ({args.log})", args.save)
    elif args.source == "bag":
        if not args.bag:
            parser.error("--bag required for --source bag")
        samples = load_from_bag(args.bag, args.topic)
        draw_samples(samples, f"Wheel telemetry ({args.bag})", args.save)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
