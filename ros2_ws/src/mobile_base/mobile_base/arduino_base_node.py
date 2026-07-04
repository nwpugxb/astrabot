#!/usr/bin/env python3
"""Bridge Arduino PID-full firmware to ROS2 wheel odometry and motor commands."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _is_telemetry_line(line: str) -> bool:
    """Firmware CSV: targetR,actualR,pwmR,targetL,actualL,pwmL,... (targets may be negative)."""
    if "," not in line:
        return False
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 10:
        return False
    try:
        float(parts[0])
        float(parts[1])
        float(parts[3])
        float(parts[4])
        return True
    except ValueError:
        return False


class ArduinoBaseNode(Node):
    """Talk to PID-full.ino over serial and publish differential-drive odometry."""

    def __init__(self) -> None:
        super().__init__("arduino_base")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("wheel_diameter_m", 0.0646)
        self.declare_parameter("counts_per_rev", 565.0)
        self.declare_parameter("wheel_separation_m", 0.208)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("teleop_timeout_s", 0.5)
        self.declare_parameter("max_wheel_counts", 80.0)
        self.declare_parameter("wheel_deadband_counts", 10.0)
        self.declare_parameter("auto_clear_fault", True)
        self.declare_parameter("fault_clear_interval_s", 1.0)
        self.declare_parameter("default_telemetry_dt_s", 0.1)
        self.declare_parameter("min_telemetry_dt_s", 0.02)
        self.declare_parameter("max_telemetry_dt_s", 0.25)
        self.declare_parameter("odom_pose_cov_lin_straight", 0.02)
        self.declare_parameter("odom_pose_cov_yaw_straight", 0.05)
        self.declare_parameter("odom_pose_cov_lin_turn", 0.05)
        self.declare_parameter("odom_pose_cov_yaw_turn", 0.5)
        self.declare_parameter("odom_pose_cov_yaw_turn_onset", 1.5)
        self.declare_parameter("turn_onset_count_diff", 2.0)

        self._port = str(self.get_parameter("serial_port").value)
        self._baud = int(self.get_parameter("baud_rate").value)
        self._wheel_d = float(self.get_parameter("wheel_diameter_m").value)
        self._counts_per_rev = float(self.get_parameter("counts_per_rev").value)
        self._wheel_sep = float(self.get_parameter("wheel_separation_m").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self._teleop_timeout = float(self.get_parameter("teleop_timeout_s").value)
        self._max_wheel_counts = float(self.get_parameter("max_wheel_counts").value)
        self._wheel_deadband_counts = float(self.get_parameter("wheel_deadband_counts").value)
        self._auto_clear_fault = bool(self.get_parameter("auto_clear_fault").value)
        self._fault_clear_interval = float(self.get_parameter("fault_clear_interval_s").value)
        self._default_dt = float(self.get_parameter("default_telemetry_dt_s").value)
        self._min_dt = float(self.get_parameter("min_telemetry_dt_s").value)
        self._max_dt = float(self.get_parameter("max_telemetry_dt_s").value)
        self._cov_lin_straight = float(self.get_parameter("odom_pose_cov_lin_straight").value)
        self._cov_yaw_straight = float(self.get_parameter("odom_pose_cov_yaw_straight").value)
        self._cov_lin_turn = float(self.get_parameter("odom_pose_cov_lin_turn").value)
        self._cov_yaw_turn = float(self.get_parameter("odom_pose_cov_yaw_turn").value)
        self._cov_yaw_turn_onset = float(self.get_parameter("odom_pose_cov_yaw_turn_onset").value)
        self._turn_onset_count_diff = float(self.get_parameter("turn_onset_count_diff").value)

        self._m_per_count = math.pi * self._wheel_d / self._counts_per_rev
        self._last_telemetry_mono: Optional[float] = None
        self._last_cmd_time = 0.0
        self._last_twist = Twist()
        self._teleop_cmd = "s"
        self._last_teleop_time = 0.0
        self._last_fault_clear = 0.0
        self._stop = threading.Event()
        self._serial_lock = threading.Lock()

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._pub_odom = self.create_publisher(Odometry, "odom", 10)
        self._pub_feedback = self.create_publisher(String, "arduino_feedback", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "arduino_teleop_cmd", self._on_teleop_cmd, 10)

        self._serial: Optional[serial.Serial] = None
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(2.0, self._try_open_serial)
        self._try_open_serial()
        self._last_sent_cmd = ""
        self._last_serial_send_time = 0.0
        self.create_timer(0.05, self._dispatch_motor_cmd)

        self.get_logger().info(f"Arduino base node ready (port {self._port})")

    def _try_open_serial(self) -> None:
        if self._serial is not None and self._serial.is_open:
            return
        self.get_logger().info(f"Opening Arduino serial {self._port} @ {self._baud}...")
        try:
            ser = serial.Serial(self._port, self._baud, timeout=0.05)
        except serial.SerialException as exc:
            self.get_logger().error(
                f"Cannot open {self._port}: {exc}. "
                "Check USB cable and run: ls -l /dev/ttyACM*"
            )
            return
        time.sleep(2.0)
        ser.reset_input_buffer()
        with self._serial_lock:
            ser.write(b"s\n")
            ser.flush()
        self._serial = ser
        self.get_logger().info(
            f"Arduino base on {self._port}: wheel_d={self._wheel_d}m, "
            f"counts/rev={self._counts_per_rev}, track={self._wheel_sep}m"
        )

    def _send_line(self, cmd: str) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        with self._serial_lock:
            self._serial.write((cmd.strip() + "\n").encode("ascii"))
            self._serial.flush()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_twist = msg
        self._last_cmd_time = time.monotonic()

    def _on_teleop_cmd(self, msg: String) -> None:
        self._teleop_cmd = msg.data.strip()
        self._last_teleop_time = time.monotonic()

    def _mps_to_counts_per_100ms(self, v_mps: float) -> float:
        """Wheel linear speed (m/s) -> encoder counts per 100 ms (firmware unit)."""
        if self._m_per_count <= 0:
            return 0.0
        return v_mps / self._m_per_count * 0.1

    def _clamp_wheel_counts(self, counts: float) -> float:
        if abs(counts) < self._wheel_deadband_counts:
            return 0.0
        if counts > self._max_wheel_counts:
            return self._max_wheel_counts
        if counts < -self._max_wheel_counts:
            return -self._max_wheel_counts
        return counts

    def _twist_to_arduino_cmd(self, twist: Twist) -> str:
        """Differential inverse kinematics: (v, w) -> `w <right> <left>`.

        v_r = v + w*L/2, v_l = v - w*L/2 (m/s), converted to counts/100ms so the
        firmware closed-loop tracks arbitrary linear+angular velocities (arcs).
        """
        v = twist.linear.x
        w = twist.angular.z
        if abs(v) < 1e-3 and abs(w) < 1e-3:
            return "s"
        half_track = self._wheel_sep / 2.0
        v_r = v + w * half_track
        v_l = v - w * half_track
        cr = self._clamp_wheel_counts(self._mps_to_counts_per_100ms(v_r))
        cl = self._clamp_wheel_counts(self._mps_to_counts_per_100ms(v_l))
        if abs(cr) < 1e-6 and abs(cl) < 1e-6:
            return "s"
        return f"w {cr:.0f} {cl:.0f}"

    def _dispatch_motor_cmd(self) -> None:
        now = time.monotonic()
        if now - self._last_teleop_time < self._teleop_timeout:
            cmd = self._teleop_cmd or "s"
        elif now - self._last_cmd_time < self._cmd_timeout:
            cmd = self._twist_to_arduino_cmd(self._last_twist)
        else:
            cmd = "s"

        # Repeat active commands every 200ms, same as direct teleop script.
        should_send = cmd != self._last_sent_cmd or now - self._last_serial_send_time >= 0.2
        if should_send:
            self._send_line(cmd)
            self._last_sent_cmd = cmd
            self._last_serial_send_time = now

    def _telemetry_dt(self) -> float:
        now = time.monotonic()
        if self._last_telemetry_mono is None:
            dt = self._default_dt
        else:
            dt = now - self._last_telemetry_mono
        self._last_telemetry_mono = now
        return max(self._min_dt, min(self._max_dt, dt))

    def _odom_covariance(
        self,
        target_r: float,
        target_l: float,
        delta_r: float,
        delta_l: float,
        delta_yaw: float,
    ) -> tuple[float, float]:
        """Pose covariance (linear x/y, yaw). Higher yaw variance during turns."""
        cmd_spin = target_r * target_l < 0 and (
            abs(target_r) > 5.0 or abs(target_l) > 5.0
        )
        cmd_arc = (
            target_r > 0
            and target_l > 0
            and abs(target_r - target_l) >= 5.0
        )
        cmd_turn = cmd_spin or cmd_arc
        meas_turn = abs(delta_yaw) > math.radians(2.0)
        weak_diff = abs(abs(delta_r) - abs(delta_l)) < self._turn_onset_count_diff
        turn_onset = cmd_turn and weak_diff

        if turn_onset:
            return self._cov_lin_turn, self._cov_yaw_turn_onset
        if cmd_turn or meas_turn:
            return self._cov_lin_turn, self._cov_yaw_turn
        return self._cov_lin_straight, self._cov_yaw_straight

    def _integrate_odom(
        self, target_r: float, target_l: float, delta_r: float, delta_l: float, dt: float
    ) -> None:
        sign_r = 0.0 if target_r == 0 else (1.0 if target_r > 0 else -1.0)
        sign_l = 0.0 if target_l == 0 else (1.0 if target_l > 0 else -1.0)
        dist_r = sign_r * abs(delta_r) * self._m_per_count
        dist_l = sign_l * abs(delta_l) * self._m_per_count
        delta_s = (dist_l + dist_r) / 2.0
        delta_yaw = (dist_r - dist_l) / self._wheel_sep

        self._x += delta_s * math.cos(self._yaw + delta_yaw / 2.0)
        self._y += delta_s * math.sin(self._yaw + delta_yaw / 2.0)
        self._yaw += delta_yaw
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        vx = delta_s / dt if dt > 0 else 0.0
        vyaw = delta_yaw / dt if dt > 0 else 0.0

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = _yaw_to_quaternion(self._yaw)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        cov_lin, cov_yaw = self._odom_covariance(
            target_r, target_l, delta_r, delta_l, delta_yaw
        )
        odom.pose.covariance[0] = cov_lin
        odom.pose.covariance[7] = cov_lin
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = cov_yaw
        odom.twist.covariance[0] = cov_lin
        odom.twist.covariance[35] = cov_yaw
        self._pub_odom.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = _yaw_to_quaternion(self._yaw)
            self._tf_broadcaster.sendTransform(tf)

    def _maybe_clear_fault(self) -> None:
        """Recover from a latched wheel STALL fault so autonomy can continue."""
        if not self._auto_clear_fault:
            return
        now = time.monotonic()
        if now - self._last_fault_clear < self._fault_clear_interval:
            return
        self._last_fault_clear = now
        self.get_logger().warn("Wheel STALL fault detected; sending 'clear'.")
        self._send_line("clear")

    def _parse_telemetry(self, line: str) -> None:
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 10:
            return
        try:
            target_r = float(parts[0])
            actual_r = float(parts[1])
            target_l = float(parts[3])
            actual_l = float(parts[4])
        except ValueError:
            return
        if "FAULT" in parts[8] or "FAULT" in parts[9]:
            self._maybe_clear_fault()
        self._integrate_odom(
            target_r, target_l, actual_r, actual_l, self._telemetry_dt()
        )

    def _read_loop(self) -> None:
        while not self._stop.is_set():
            if self._serial is None or not self._serial.is_open:
                time.sleep(0.1)
                continue
            try:
                raw = self._serial.readline()
            except (serial.SerialException, AttributeError) as exc:
                self.get_logger().error(f"Serial error: {exc}")
                self._serial = None
                time.sleep(1.0)
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line or line.startswith("Dual wheel") or line.startswith("Commands"):
                continue
            if line.startswith("targetR") or line.startswith("Speed range"):
                continue
            if line.startswith("STALL") or line.startswith("Set targets"):
                self.get_logger().warn(line)
                fb = String()
                fb.data = line
                self._pub_feedback.publish(fb)
                continue
            if _is_telemetry_line(line):
                self._parse_telemetry(line)
                fb = String()
                fb.data = line
                self._pub_feedback.publish(fb)

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self._send_line("s")
        except Exception:
            pass
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ArduinoBaseNode()
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
