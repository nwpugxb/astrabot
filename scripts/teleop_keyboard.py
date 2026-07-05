#!/usr/bin/env python3
"""Curses keyboard teleop matching PID-full Arduino command protocol.

Hold W/A/S/D to move; release to stop immediately (via Linux evdev key-up events).
Requires membership in the Linux 'input' group. Run: ./scripts/setup_teleop_input.sh

Default ROS mode publishes /arduino_teleop_cmd for arduino_base_node (Arduino UNO).
ESP32 micro-ROS deck robot uses /cmd_vel instead:

  ./scripts/deck_teleop.sh

Direct serial mode (no ROS):  ./scripts/teleop.sh --direct
"""

from __future__ import annotations

import argparse
import curses
import glob
import grp
import math
import os
import queue
import re
import sys
import threading
import time
from typing import Callable, Optional, Protocol

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String

MIN_SPEED = 30
MAX_SPEED = 80
CMD_VEL_PUBLISH_HZ = 20
CMD_VEL_PUB_INTERVAL = 1.0 / CMD_VEL_PUBLISH_HZ
CMD_VEL_QOS_DEPTH = 1
REPEAT_INTERVAL = CMD_VEL_PUB_INTERVAL
LOOP_INTERVAL = 0.02  # UI / evdev poll (~50 Hz); publish gated in CmdVelTeleopBridge
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Deck robot / ESP32 micro-ROS (config_l298n.h geometry).
WHEEL_D_M = 0.0646
WHEEL_SEP_M = 0.208
COUNTS_PER_REV = 564.0
M_PER_COUNT = math.pi * WHEEL_D_M / COUNTS_PER_REV

MOTION_PRIORITY = ("f", "b", "l", "r")

CMD_VEL_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=CMD_VEL_QOS_DEPTH,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _twist_is_stop(twist: Twist) -> bool:
    return abs(twist.linear.x) < 1e-6 and abs(twist.angular.z) < 1e-6

class HoldTracker(Protocol):
    def get_motion(self) -> str: ...
    def poll_aux(self) -> Optional[str]: ...
    def stop(self) -> None: ...
    @property
    def hint(self) -> str: ...


def _motion_cmd(motion: str, speed: int) -> str:
    return "s" if motion == "s" else f"{motion} {speed}"


def _counts_to_mps(counts_per_100ms: float) -> float:
    if M_PER_COUNT <= 0:
        return 0.0
    return counts_per_100ms * M_PER_COUNT / 0.1


def _parse_motion_cmd(cmd: str) -> tuple[str, int]:
    cmd = cmd.strip()
    if not cmd or cmd.lower() == "s":
        return "s", MIN_SPEED
    parts = cmd.split()
    motion = parts[0].lower()
    speed = int(float(parts[1])) if len(parts) > 1 else MIN_SPEED
    return motion, speed


def _motion_speed_to_twist(motion: str, speed: int) -> Twist:
    twist = Twist()
    if motion == "s":
        return twist
    mps = _counts_to_mps(float(speed))
    if motion == "f":
        twist.linear.x = mps
    elif motion == "b":
        twist.linear.x = -mps
    elif motion == "l":
        twist.angular.z = 2.0 * mps / WHEEL_SEP_M
    elif motion == "r":
        twist.angular.z = -2.0 * mps / WHEEL_SEP_M
    return twist


def _user_groups() -> set[str]:
    groups = {grp.getgrgid(gid).gr_name for gid in os.getgroups()}
    groups.add(os.getenv("USER", ""))
    return groups


def _keyboard_event_nodes() -> list[str]:
    """Find /dev/input/event* nodes that look like keyboards."""
    found: list[str] = []
    seen: set[str] = set()
    try:
        with open("/proc/bus/input/devices", encoding="utf-8") as handle:
            blocks = handle.read().split("\n\n")
    except OSError:
        blocks = []

    for block in blocks:
        name_match = re.search(r'N: Name="([^"]*)"', block)
        name = (name_match.group(1) if name_match else "").lower()
        handlers_match = re.search(r"H: Handlers=([^\n]*)", block)
        handlers = handlers_match.group(1) if handlers_match else ""
        is_keyboard = "kbd" in handlers.split() or "keyboard" in name
        if not is_keyboard:
            continue
        for event_id in re.findall(r"event(\d+)", handlers):
            path = f"/dev/input/event{event_id}"
            if path not in seen:
                seen.add(path)
                found.append(path)

    if not found:
        found = sorted(glob.glob("/dev/input/event*"))
    return found


def _open_evdev_keyboard(device_path: Optional[str] = None):
    try:
        from evdev import InputDevice, ecodes
    except ImportError as exc:
        raise RuntimeError(
            "python3-evdev is required.\nInstall: sudo apt install python3-evdev"
        ) from exc

    errors: list[str] = []
    paths = [device_path] if device_path else _keyboard_event_nodes()

    for path in paths:
        if not path or not os.path.exists(path):
            continue
        try:
            dev = InputDevice(path)
            keys = dev.capabilities().get(ecodes.EV_KEY, [])
            if ecodes.KEY_W in keys and ecodes.KEY_A in keys and ecodes.KEY_S in keys:
                return dev, ecodes
            dev.close()
            errors.append(f"{path}: not a WASD keyboard ({dev.name})")
        except PermissionError:
            errors.append(f"{path}: permission denied")
        except OSError as exc:
            errors.append(f"{path}: {exc}")

    groups = ", ".join(sorted(_user_groups()))
    detail = "\n".join(f"  - {line}" for line in errors[:8]) if errors else "  (no devices probed)"
    raise RuntimeError(
        "Cannot open a keyboard via evdev.\n"
        f"Your groups: {groups}\n"
        f"Details:\n{detail}\n\n"
        "Fix:\n"
        "  ./scripts/setup_teleop_input.sh\n"
        "  newgrp input\n"
        "  ./scripts/teleop.sh"
    )


class EvdevHoldTracker:
    """Track WASD hold/release from the physical keyboard via /dev/input."""

    def __init__(
        self,
        device_path: Optional[str] = None,
        on_motion: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._device, self._ecodes = _open_evdev_keyboard(device_path)
        self._hint = f"Keyboard: {self._device.path} ({self._device.name})"
        self._held: set[str] = set()
        self._held_order: list[str] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._aux_q: queue.Queue[str] = queue.Queue(maxsize=32)
        self._on_motion = on_motion
        self.speed = MIN_SPEED
        ecodes = self._ecodes
        self._motion_codes = {
            ecodes.KEY_W: "f",
            ecodes.KEY_A: "l",
            ecodes.KEY_S: "b",
            ecodes.KEY_D: "r",
        }
        self._aux_codes = {
            ecodes.KEY_SPACE: "space",
            ecodes.KEY_Q: "q",
            ecodes.KEY_1: "1",
            ecodes.KEY_2: "2",
            ecodes.KEY_3: "3",
            ecodes.KEY_EQUAL: "speed_up",
            ecodes.KEY_MINUS: "speed_down",
        }
        if hasattr(ecodes, "KEY_KPPLUS"):
            self._aux_codes[ecodes.KEY_KPPLUS] = "speed_up"
        if hasattr(ecodes, "KEY_KPMINUS"):
            self._aux_codes[ecodes.KEY_KPMINUS] = "speed_down"
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    @property
    def hint(self) -> str:
        return self._hint

    def _read_loop(self) -> None:
        ecodes = self._ecodes
        try:
            for event in self._device.read_loop():
                if self._stop.is_set():
                    break
                if event.type != ecodes.EV_KEY:
                    continue
                if event.code in self._motion_codes:
                    motion = self._motion_codes[event.code]
                    with self._lock:
                        if event.value == 1:
                            if motion not in self._held:
                                self._held.add(motion)
                                self._held_order.append(motion)
                        elif event.value == 0 and motion in self._held:
                            self._held.discard(motion)
                            self._held_order = [m for m in self._held_order if m in self._held]
                    if event.value in (0, 1) and self._on_motion is not None:
                        self._on_motion(self.get_motion())
                elif event.value == 1 and event.code in self._aux_codes:
                    try:
                        self._aux_q.put_nowait(self._aux_codes[event.code])
                    except queue.Full:
                        pass
        except OSError as exc:
            try:
                self._aux_q.put_nowait(f"evdev error: {exc}")
            except queue.Full:
                pass

    def get_motion(self) -> str:
        with self._lock:
            if not self._held:
                return "s"
            for motion in reversed(self._held_order):
                if motion in self._held:
                    return motion
            for motion in MOTION_PRIORITY:
                if motion in self._held:
                    return motion
            return "s"

    def poll_aux(self) -> Optional[str]:
        try:
            return self._aux_q.get_nowait()
        except queue.Empty:
            return None

    def stop(self) -> None:
        self._stop.set()
        try:
            self._device.close()
        except Exception:
            pass


def _apply_aux_key(
    aux: str,
    motion: str,
    speed: int,
    send_cmd: Callable[[str], None],
) -> tuple[str, int, Optional[str]]:
    if aux == "space":
        send_cmd("s")
        return "s", speed, "s"
    if aux == "q":
        send_cmd("q")
        return "s", speed, "q"
    if aux == "1":
        speed = 30
    elif aux == "2":
        speed = 45
    elif aux == "3":
        speed = 60
    elif aux == "speed_up":
        speed = min(MAX_SPEED, speed + 5)
    elif aux == "speed_down":
        speed = max(MIN_SPEED, speed - 5)
    else:
        return motion, speed, None

    if motion in MOTION_PRIORITY:
        cmd = _motion_cmd(motion, speed)
        send_cmd(cmd)
        return motion, speed, cmd
    return motion, speed, None


def _apply_curses_aux(
    key: int,
    motion: str,
    speed: int,
    send_cmd: Callable[[str], None],
) -> tuple[str, int, Optional[str]]:
    if key == ord(" "):
        return _apply_aux_key("space", motion, speed, send_cmd)
    if key in (ord("q"), ord("Q")):
        return _apply_aux_key("q", motion, speed, send_cmd)
    if key == ord("1"):
        return _apply_aux_key("1", motion, speed, send_cmd)
    if key == ord("2"):
        return _apply_aux_key("2", motion, speed, send_cmd)
    if key == ord("3"):
        return _apply_aux_key("3", motion, speed, send_cmd)
    if key in (curses.KEY_UP, ord("+"), ord("=")):
        return _apply_aux_key("speed_up", motion, speed, send_cmd)
    if key in (curses.KEY_DOWN, ord("-"), ord("_")):
        return _apply_aux_key("speed_down", motion, speed, send_cmd)
    return motion, speed, None


def _safe_addstr(stdscr, row: int, col: int, text: str) -> None:
    try:
        max_y, max_x = stdscr.getmaxyx()
        if row < 0 or row >= max_y or col >= max_x:
            return
        width = max(0, max_x - col - 1)
        if width == 0:
            return
        stdscr.addstr(row, col, text[:width])
    except curses.error:
        pass


def draw_screen(
    stdscr,
    motion: str,
    last_cmd: str,
    speed: int,
    last_arduino: str,
    mode: str,
    input_hint: str = "",
) -> None:
    stdscr.erase()
    max_y, max_x = stdscr.getmaxyx()
    if max_y < 12 or max_x < 40:
        _safe_addstr(stdscr, 0, 0, "Terminal too small. Resize to at least 40x12.")
        _safe_addstr(stdscr, 1, 0, f"Motion={motion} Speed={speed} CMD={last_cmd}")
        stdscr.refresh()
        return

    if max_y >= 28:
        lines = [
            (0, 0, "========================================"),
            (1, 0, " Robot Vacuum Terminal Keyboard Drive"),
            (2, 0, f" Mode: {mode}"),
            (3, 0, "========================================"),
            (5, 0, f"Serial Port : {SERIAL_PORT}"),
            (6, 0, f"Speed       : {speed} count / 100ms"),
            (7, 0, f"Motion      : {motion}"),
            (8, 0, f"Last CMD    : {last_cmd}"),
            (10, 0, "Controls:"),
            (11, 2, "W       Forward (hold)"),
            (12, 2, "S       Backward (hold)"),
            (13, 2, "A       Turn Left (hold)"),
            (14, 2, "D       Turn Right (hold)"),
            (15, 2, "Space   Stop"),
            (16, 2, "Q       Square Mode"),
            (17, 2, "1       Speed 30"),
            (18, 2, "2       Speed 45"),
            (19, 2, "3       Speed 60"),
            (20, 2, "Up/+    Speed +5"),
            (21, 2, "Down/-  Speed -5"),
            (22, 2, "ESC     Exit"),
            (24, 0, "Arduino:"),
            (25, 2, last_arduino),
            (27, 0, "Hold W/A/S/D to move. Release key to stop immediately."),
            (28, 0, input_hint),
        ]
    else:
        lines = [
            (0, 0, " Robot Vacuum Keyboard Drive"),
            (1, 0, f"Mode: {mode}"),
            (2, 0, f"Port: {SERIAL_PORT}  Speed: {speed}  Motion: {motion}"),
            (3, 0, f"Last CMD: {last_cmd}"),
            (5, 0, "Hold W/A/S/D. Release to stop. Space=stop Q=square"),
            (6, 0, "1/2/3=30/45/60  Up/Down +/-5  ESC exit"),
            (8, 0, "Arduino:"),
            (9, 2, last_arduino),
            (11, 0, input_hint or "Hold W/A/S/D. Release to stop."),
        ]

    for row, col, text in lines:
        if row < max_y and text:
            _safe_addstr(stdscr, row, col, text)
    stdscr.refresh()


def _drive_loop(
    stdscr,
    tracker: HoldTracker,
    send_cmd: Callable[[str], None],
    poll_feedback: Callable[[], Optional[str]],
    mode: str,
    should_run: Callable[[], bool],
    on_exit: Callable[[], None],
) -> None:
    speed = getattr(tracker, "speed", MIN_SPEED)
    motion = "s"
    last_cmd = "s"
    last_arduino = ""
    last_send_time = 0.0

    send_cmd("s")

    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    try:
        while should_run():
            fb = poll_feedback()
            if fb:
                last_arduino = fb

            aux = tracker.poll_aux()
            while aux:
                if aux.startswith("evdev error:"):
                    last_arduino = aux
                    aux = tracker.poll_aux()
                    continue
                motion, speed, cmd = _apply_aux_key(aux, motion, speed, send_cmd)
                if cmd:
                    last_cmd = cmd
                    last_send_time = time.time()
                aux = tracker.poll_aux()

            tracker.speed = speed

            key = stdscr.getch()
            if key == 27:
                send_cmd("s")
                break
            if key != -1:
                motion, speed, cmd = _apply_curses_aux(key, motion, speed, send_cmd)
                if cmd:
                    last_cmd = cmd
                    last_send_time = time.time()

            new_motion = tracker.get_motion()
            now = time.time()

            if new_motion != motion:
                motion = new_motion
                cmd = _motion_cmd(motion, speed)
                send_cmd(cmd)
                last_cmd = cmd
                last_send_time = now
            elif motion in MOTION_PRIORITY and now - last_send_time >= REPEAT_INTERVAL:
                cmd = _motion_cmd(motion, speed)
                send_cmd(cmd)
                last_cmd = cmd
                last_send_time = now

            tracker.speed = speed
            draw_screen(stdscr, motion, last_cmd, speed, last_arduino, mode, tracker.hint)
            time.sleep(LOOP_INTERVAL)
    finally:
        tracker.stop()
        send_cmd("s")
        on_exit()


def run_direct_serial(stdscr, tracker: HoldTracker) -> None:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    time.sleep(2.0)
    arduino_q: queue.Queue[str] = queue.Queue(maxsize=32)
    running = True

    def serial_reader() -> None:
        while running:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    try:
                        arduino_q.put_nowait(line)
                    except queue.Full:
                        pass
            except Exception as exc:
                try:
                    arduino_q.put_nowait(f"Serial read error: {exc}")
                except queue.Full:
                    pass
                time.sleep(0.2)

    def send_cmd(cmd: str) -> None:
        ser.write((cmd + "\n").encode("utf-8"))
        ser.flush()

    def poll_feedback() -> Optional[str]:
        try:
            return arduino_q.get_nowait()
        except queue.Empty:
            return None

    threading.Thread(target=serial_reader, daemon=True).start()

    def on_exit() -> None:
        nonlocal running
        running = False
        try:
            time.sleep(0.1)
            ser.close()
        except Exception:
            pass

    _drive_loop(stdscr, tracker, send_cmd, poll_feedback, "direct serial", lambda: True, on_exit)


class CmdVelTeleopBridge(Node):
    """Hold-to-drive teleop for ESP32 / micro-ROS (/cmd_vel)."""

    def __init__(self, cmd_vel_topic: str = "/cmd_vel") -> None:
        super().__init__("teleop_cmd_vel")
        self._pub = self.create_publisher(Twist, cmd_vel_topic, CMD_VEL_QOS)
        self._last_twist = Twist()
        self._last_published = Twist()
        self._last_pub_time = 0.0
        self._pub_interval = CMD_VEL_PUB_INTERVAL
        self.get_logger().info(
            f"evdev teleop -> {cmd_vel_topic} (direct pass-through, "
            f"hold repeat {CMD_VEL_PUBLISH_HZ:.0f} Hz, stop/reversal immediate)"
        )

    @staticmethod
    def _twist_equal(a: Twist, b: Twist, eps: float = 1e-6) -> bool:
        return (
            abs(a.linear.x - b.linear.x) < eps
            and abs(a.angular.z - b.angular.z) < eps
        )

    def send_cmd(self, cmd: str) -> None:
        motion, speed = _parse_motion_cmd(cmd)
        twist = _motion_speed_to_twist(motion, speed)
        self._pub.publish(twist)
        self._last_pub_time = time.time()
        self._last_published = Twist()
        self._last_published.linear.x = twist.linear.x
        self._last_published.angular.z = twist.angular.z
        self._last_twist = twist

    def drain_feedback(self) -> Optional[str]:
        t = self._last_twist
        if abs(t.linear.x) < 1e-6 and abs(t.angular.z) < 1e-6:
            return "cmd_vel stop"
        return f"cmd_vel lin={t.linear.x:.3f} ang={t.angular.z:.3f}"


class RosTeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__("teleop_curses")
        self._pub = self.create_publisher(String, "arduino_teleop_cmd", 10)
        self._feedback_q: queue.Queue[str] = queue.Queue(maxsize=32)
        self.create_subscription(String, "arduino_feedback", self._on_feedback, 10)

    def _on_feedback(self, msg: String) -> None:
        try:
            self._feedback_q.put_nowait(msg.data)
        except queue.Full:
            pass

    def send_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)

    def drain_feedback(self) -> Optional[str]:
        last = None
        while not self._feedback_q.empty():
            last = self._feedback_q.get()
        return last


def run_ros_teleop(stdscr, tracker: HoldTracker, cmd_vel: bool = False) -> None:
    rclpy.init()
    node: RosTeleopBridge | CmdVelTeleopBridge
    mode = "ROS /cmd_vel (ESP32 micro-ROS)" if cmd_vel else "ROS /arduino_teleop_cmd"
    if cmd_vel:
        node = CmdVelTeleopBridge()

        def on_motion(m: str) -> None:
            node.send_cmd(_motion_cmd(m, tracker.speed))

        tracker._on_motion = on_motion
    else:
        node = RosTeleopBridge()
    spin_stop = threading.Event()

    def spin() -> None:
        while not spin_stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    threading.Thread(target=spin, daemon=True).start()

    def on_exit() -> None:
        spin_stop.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    try:
        _drive_loop(
            stdscr,
            tracker,
            node.send_cmd,
            node.drain_feedback,
            mode,
            lambda: rclpy.ok(),
            on_exit,
        )
    except Exception:
        on_exit()
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description="Curses teleop for Arduino PID-full")
    parser.add_argument(
        "--direct",
        action="store_true",
        help="Talk to /dev/ttyACM0 directly (do not use while run_mobile_mapping.sh is running)",
    )
    parser.add_argument(
        "--cmd-vel",
        action="store_true",
        help="Publish geometry_msgs/Twist on /cmd_vel (ESP32 micro-ROS deck robot)",
    )
    parser.add_argument(
        "--input-device",
        default=None,
        help="Keyboard evdev node, e.g. /dev/input/event2",
    )
    args = parser.parse_args()

    try:
        tracker = EvdevHoldTracker(args.input_device)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1

    try:
        if args.direct:
            curses.wrapper(lambda stdscr: run_direct_serial(stdscr, tracker))
        else:
            curses.wrapper(lambda stdscr: run_ros_teleop(stdscr, tracker, args.cmd_vel))
    except KeyboardInterrupt:
        tracker.stop()
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
