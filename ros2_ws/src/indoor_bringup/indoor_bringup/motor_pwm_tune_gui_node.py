#!/usr/bin/env python3
"""GUI teleop for ESP32 deck robot PWM feedforward tune (hold to move).

Same interaction as sim_teleop_gui_node: click yellow zone, hold WASD, release to stop.
Up/Down adjust wheel speed (counts/100ms); [ ] adjust live /motor_ff_pwm override.

Firmware (config_l298n.h): OPEN_LOOP_MOTOR=false, STALL_PROTECTION_ENABLE=true.
"""

from __future__ import annotations

import math
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from indoor_bringup.microros_qos import MICROROS_CMD_QOS, MICROROS_QOS
from std_msgs.msg import Float32

WHEEL_D_M = 0.0646
WHEEL_SEP_M = 0.208
COUNTS_PER_REV = 564.0
M_PER_COUNT = math.pi * WHEEL_D_M / COUNTS_PER_REV

MIN_SPEED = 30
MAX_SPEED = 80
MIN_PWM = 0
MAX_PWM = 255
PWM_STEP = 5
SPEED_STEP = 5

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

MOTION_KEYS = frozenset(KEY_BINDINGS)
SPEED_UP_KEYS = frozenset({"equal", "plus", "kp_add"})
SPEED_DOWN_KEYS = frozenset({"minus", "underscore", "kp_subtract"})


def counts_to_mps(counts_per_100ms: float) -> float:
    if M_PER_COUNT <= 0:
        return 0.0
    return counts_per_100ms * M_PER_COUNT / 0.1


def pwm_band(speed: int) -> str:
    c = float(speed)
    if c <= 12:
        return "PWM_FF_LE_12"
    if c <= 20:
        return "PWM_FF_LE_20"
    if c <= 30:
        return "PWM_FF_LE_30"
    if c <= 40:
        return "PWM_FF_LE_40"
    if c <= 55:
        return "PWM_FF_LE_55"
    if c <= 70:
        return "PWM_FF_LE_70"
    if c <= 80:
        return "PWM_FF_LE_80"
    return "PWM_FF_MAX"


def motion_to_twist(lin_u: float, ang_u: float, speed: int) -> Twist:
    twist = Twist()
    if lin_u == 0.0 and ang_u == 0.0:
        return twist
    c = float(speed)
    if lin_u > 0.0:
        twist.linear.x = counts_to_mps(c)
    elif lin_u < 0.0:
        twist.linear.x = -counts_to_mps(c)
    elif ang_u > 0.0:
        twist.angular.z = 2.0 * counts_to_mps(c) / WHEEL_SEP_M
    elif ang_u < 0.0:
        twist.angular.z = -2.0 * counts_to_mps(c) / WHEEL_SEP_M
    return twist


class MotorPwmTuneGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_pwm_tune_gui")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("motor_ff_pwm_topic", "/motor_ff_pwm")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("initial_speed", 30)
        self.declare_parameter("initial_pwm", 70)

        self._lin_u = 0.0
        self._ang_u = 0.0
        self._speed = int(self.get_parameter("initial_speed").value)
        self._pwm = int(self.get_parameter("initial_pwm").value)

        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        ff_topic = str(self.get_parameter("motor_ff_pwm_topic").value)
        self._pub_cmd = self.create_publisher(Twist, cmd_topic, MICROROS_CMD_QOS)
        self._pub_ff = self.create_publisher(Float32, ff_topic, MICROROS_QOS)

        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f"PWM tune GUI -> {cmd_topic}, {ff_topic}  "
            f"(speed={self._speed} counts/100ms, pwm={self._pwm})"
        )

    @property
    def speed(self) -> int:
        return self._speed

    @property
    def pwm(self) -> int:
        return self._pwm

    def _publish(self) -> None:
        moving = self._lin_u != 0.0 or self._ang_u != 0.0
        self._pub_cmd.publish(motion_to_twist(self._lin_u, self._ang_u, self._speed))
        ff = Float32()
        ff.data = float(self._pwm) if moving else 0.0
        self._pub_ff.publish(ff)

    def set_motion(self, lin_u: float, ang_u: float) -> None:
        self._lin_u = lin_u
        self._ang_u = ang_u

    def stop(self) -> None:
        self.set_motion(0.0, 0.0)

    def bump_speed(self, delta: int) -> None:
        self._speed = max(MIN_SPEED, min(MAX_SPEED, self._speed + delta))

    def bump_pwm(self, delta: int) -> None:
        self._pwm = max(MIN_PWM, min(MAX_PWM, self._pwm + delta))

    def set_speed_preset(self, value: int) -> None:
        self._speed = max(MIN_SPEED, min(MAX_SPEED, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorPwmTuneGuiNode()

    root = tk.Tk()
    root.title("Deck Robot PWM Tune")
    root.geometry("420x460")
    root.resizable(False, False)

    motion_status = tk.StringVar(value="停止")
    tune_status = tk.StringVar()
    focus_hint = tk.StringVar(value="键盘：先点窗口或黄色条，再按住 WASD / 方向键")

    def refresh_tune_status() -> None:
        tune_status.set(
            f"速度 {node.speed} counts/100ms  ->  {pwm_band(node.speed)}\n"
            f"PWM 覆盖 {node.pwm}  (/motor_ff_pwm，按住方向键时生效)"
        )

    def update_motion_status() -> None:
        if node._lin_u > 0.0:
            motion_status.set(f"前进  speed={node.speed}")
        elif node._lin_u < 0.0:
            motion_status.set(f"后退  speed={node.speed}")
        elif node._ang_u > 0.0:
            motion_status.set(f"左转  speed={node.speed}")
        elif node._ang_u < 0.0:
            motion_status.set(f"右转  speed={node.speed}")
        else:
            motion_status.set("停止")
        refresh_tune_status()

    def set_motion_from_unit(lin_u: float, ang_u: float) -> None:
        node.set_motion(lin_u, ang_u)
        update_motion_status()

    def stop() -> None:
        node.stop()
        update_motion_status()

    def apply_key(key: str) -> None:
        key = key.lower()
        if key in KEY_BINDINGS:
            lin_u, ang_u = KEY_BINDINGS[key]
            set_motion_from_unit(lin_u, ang_u)

    frame = tk.Frame(root, padx=12, pady=10)
    frame.pack(fill="both", expand=True)

    tk.Label(
        frame,
        text=(
            "ESP32 闭环 + stall 标定 feedforward PWM\n"
            "鼠标：按住方向按钮 = 走，松开即停\n"
            "键盘：先点本窗口，再按住 WASD / 方向键（+/- 调速度，[ ] 调 PWM）"
        ),
        justify="left",
        wraplength=390,
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
        focus_hint.set("键盘已启用 — WASD/方向键走；+/- 速度，[ ] PWM")

    def on_zone_focus_out(_event) -> None:
        key_zone.config(bg="#fff3cd")
        focus_hint.set("键盘：先点窗口或黄色条，再按住 WASD / 方向键")

    def focus_keyboard(_event=None) -> None:
        root.focus_set()
        key_zone.config(bg="#d4edda")
        focus_hint.set("键盘已启用 — WASD/方向键走；+/- 速度，[ ] PWM")

    def on_key_press(event) -> None:
        sym = (event.keysym or "").lower()
        if sym in SPEED_UP_KEYS:
            node.bump_speed(SPEED_STEP)
            update_motion_status()
            return
        if sym in SPEED_DOWN_KEYS:
            node.bump_speed(-SPEED_STEP)
            update_motion_status()
            return
        if sym == "bracketleft":
            node.bump_pwm(-PWM_STEP)
            update_motion_status()
            return
        if sym == "bracketright":
            node.bump_pwm(PWM_STEP)
            update_motion_status()
            return
        if sym == "1":
            node.set_speed_preset(30)
            update_motion_status()
            return
        if sym == "2":
            node.set_speed_preset(45)
            update_motion_status()
            return
        if sym == "3":
            node.set_speed_preset(60)
            update_motion_status()
            return
        apply_key(sym)

    def on_key_release(event) -> None:
        if (event.keysym or "").lower() in MOTION_KEYS:
            stop()

    key_zone.bind("<Button-1>", focus_keyboard)
    key_zone.bind("<FocusIn>", on_zone_focus_in)
    key_zone.bind("<FocusOut>", on_zone_focus_out)

    root.bind("<KeyPress>", on_key_press)
    root.bind("<KeyRelease>", on_key_release)
    root.bind("<space>", lambda _e: stop())
    root.bind("<Button-1>", focus_keyboard, add="+")

    tk.Label(frame, textvariable=motion_status, font=("", 11, "bold")).pack(pady=(0, 6))
    tk.Label(frame, textvariable=tune_status, justify="left", wraplength=390).pack(anchor="w", pady=(0, 8))

    tune_btn = tk.Frame(frame)
    tune_btn.pack(fill="x", pady=(0, 8))

    def mk_tune_row(label: str, on_dec, on_inc) -> None:
        row = tk.Frame(tune_btn)
        row.pack(fill="x", pady=2)
        tk.Label(row, text=label, width=14, anchor="w").pack(side="left")
        tk.Button(row, text="-", width=4, command=on_dec).pack(side="left", padx=2)
        tk.Button(row, text="+", width=4, command=on_inc).pack(side="left", padx=2)

    mk_tune_row(
        "Speed",
        lambda: (node.bump_speed(-SPEED_STEP), update_motion_status()),
        lambda: (node.bump_speed(SPEED_STEP), update_motion_status()),
    )
    mk_tune_row(
        "PWM",
        lambda: (node.bump_pwm(-PWM_STEP), update_motion_status()),
        lambda: (node.bump_pwm(PWM_STEP), update_motion_status()),
    )

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

    tk.Label(
        frame,
        text="标定：按住前进，用 [ 降低 PWM 直到 stall，最低值 +10 写入 config_l298n.h",
        justify="left",
        wraplength=390,
        fg="#555555",
    ).pack(anchor="w", pady=(10, 0))

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
    update_motion_status()
    spin_ros()
    root.after(200, focus_keyboard)
    root.mainloop()


if __name__ == "__main__":
    main()
