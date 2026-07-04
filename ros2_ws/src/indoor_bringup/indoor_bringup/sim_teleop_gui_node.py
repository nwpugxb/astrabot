#!/usr/bin/env python3
"""GUI teleop for Gazebo sim — mouse hold + keyboard capture zone (Wayland/Cursor safe)."""

from __future__ import annotations

import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

KEY_BINDINGS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "i": (1.0, 0.0),
    "k": (-1.0, 0.0),
    "j": (0.0, 1.0),
    "l": (0.0, -1.0),
    "up": (1.0, 0.0),
    "down": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}


class SimTeleopGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_teleop_gui")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 0.6)
        self.declare_parameter("publish_hz", 20.0)

        self._linear = 0.0
        self._angular = 0.0
        self._lin_max = float(self.get_parameter("linear_speed").value)
        self._ang_max = float(self.get_parameter("angular_speed").value)
        topic = str(self.get_parameter("cmd_vel_topic").value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Twist, topic, qos)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish)
        self.create_timer(2.0, self._health_check)
        self._health_ok = False
        self.get_logger().info(
            f"GUI teleop -> {topic}  (linear={self._lin_max}, angular={self._ang_max})"
        )

    def _health_check(self) -> None:
        names = {n for n, _ in self.get_topic_names_and_types()}
        if "/scan" in names and "/joint_states" in names:
            if not self._health_ok:
                self.get_logger().info("Sim OK: /scan + /joint_states detected.")
            self._health_ok = True
            return
        self.get_logger().error(
            "Sim NOT running! Run ./run_sim_mapping.sh first (wait 15s).",
            throttle_duration_sec=5.0,
        )

    def _publish(self) -> None:
        msg = Twist()
        msg.linear.x = self._linear
        msg.angular.z = self._angular
        self._pub.publish(msg)

    def set_motion(self, linear: float, angular: float) -> None:
        self._linear = max(-self._lin_max, min(self._lin_max, linear))
        self._angular = max(-self._ang_max, min(self._ang_max, angular))

    def stop(self) -> None:
        self.set_motion(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimTeleopGuiNode()

    root = tk.Tk()
    root.title("Indoor Sim Teleop")
    root.geometry("380x340")
    root.resizable(False, False)

    motion_status = tk.StringVar(value="停止")
    focus_hint = tk.StringVar(value="键盘：先点黄色区域，再按住 WASD")

    def update_status() -> None:
        if node._linear > 0.01:
            motion_status.set(f"前进 {node._linear:.2f} m/s")
        elif node._linear < -0.01:
            motion_status.set(f"后退 {abs(node._linear):.2f} m/s")
        elif node._angular > 0.01:
            motion_status.set(f"左转 {node._angular:.2f} rad/s")
        elif node._angular < -0.01:
            motion_status.set(f"右转 {abs(node._angular):.2f} rad/s")
        else:
            motion_status.set("停止")

    def set_motion_from_unit(lin_u: float, ang_u: float) -> None:
        node.set_motion(lin_u * node._lin_max, ang_u * node._ang_max)
        update_status()

    def stop() -> None:
        node.stop()
        update_status()

    def apply_key(key: str) -> None:
        key = key.lower()
        if key in KEY_BINDINGS:
            lin_u, ang_u = KEY_BINDINGS[key]
            set_motion_from_unit(lin_u, ang_u)

    frame = tk.Frame(root, padx=12, pady=10)
    frame.pack(fill="both", expand=True)

    tk.Label(
        frame,
        text="鼠标：按住方向按钮 = 走，松开即停\n键盘：点黄色条，再按住 WASD（终端里按键无效）",
        justify="left",
        wraplength=350,
    ).pack(anchor="w", pady=(0, 8))

    key_zone = tk.Label(
        frame,
        textvariable=focus_hint,
        bg="#fff3cd",
        fg="#333333",
        relief="solid",
        borderwidth=2,
        padx=8,
        pady=12,
        takefocus=True,
        cursor="hand2",
    )
    key_zone.pack(fill="x", pady=(0, 8))

    def on_zone_focus_in(_event) -> None:
        key_zone.config(bg="#d4edda")
        focus_hint.set("键盘已启用 — 按住 WASD / 方向键")

    def on_zone_focus_out(_event) -> None:
        key_zone.config(bg="#fff3cd")
        focus_hint.set("键盘：先点黄色区域，再按住 WASD")
        stop()

    def on_zone_click(_event) -> None:
        key_zone.focus_set()

    def on_key_press(event) -> None:
        apply_key(event.keysym or "")

    def on_key_release(event) -> None:
        if (event.keysym or "").lower() in KEY_BINDINGS:
            stop()

    key_zone.bind("<Button-1>", on_zone_click)
    key_zone.bind("<FocusIn>", on_zone_focus_in)
    key_zone.bind("<FocusOut>", on_zone_focus_out)
    key_zone.bind("<KeyPress>", on_key_press)
    key_zone.bind("<KeyRelease>", on_key_release)
    key_zone.bind("<space>", lambda _e: stop())

    tk.Label(frame, textvariable=motion_status, font=("", 11, "bold")).pack(pady=(0, 10))

    btn_frame = tk.Frame(frame)
    btn_frame.pack()

    def make_hold_button(text: str, row: int, col: int, lin_u: float, ang_u: float) -> None:
        btn = tk.Button(btn_frame, text=text, width=14)

        def press(_e) -> None:
            set_motion_from_unit(lin_u, ang_u)

        def release(_e) -> None:
            stop()

        btn.bind("<ButtonPress-1>", press)
        btn.bind("<ButtonRelease-1>", release)
        btn.bind("<Leave>", release)
        btn.grid(row=row, column=col, padx=3, pady=3)

    make_hold_button("Forward (W)", 0, 1, 1.0, 0.0)
    make_hold_button("Left (A)", 1, 0, 0.0, 1.0)
    stop_btn = tk.Button(btn_frame, text="Stop", width=14, command=stop)
    stop_btn.grid(row=1, column=1, padx=3, pady=3)
    make_hold_button("Right (D)", 1, 2, 0.0, -1.0)
    make_hold_button("Back (S)", 2, 1, -1.0, 0.0)

    def spin_ros() -> None:
        rclpy.spin_once(node, timeout_sec=0.0)
        root.after(20, spin_ros)

    def on_close() -> None:
        stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    spin_ros()
    root.after(200, key_zone.focus_set)
    root.mainloop()


if __name__ == "__main__":
    main()
