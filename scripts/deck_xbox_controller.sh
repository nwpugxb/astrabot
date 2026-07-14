#!/usr/bin/env bash
# Xbox One controller teleop for ESP32 deck robot (micro-ROS /cmd_vel).
#
# Prerequisites (same WiFi/serial agent as keyboard teleop):
#   ./scripts/run_deck_wifi_lidar.sh   OR   ./scripts/run_microros_agent_wifi.sh
#   Controller connected (USB or already paired Bluetooth)
#   Once: ./scripts/setup_teleop_input.sh   # input group for /dev/input/js*
#
# Controls:
#   D-pad     forward / back / turn
#   Y / A     speed + / -
#
# Do NOT run together with ./scripts/deck_teleop.sh (both publish /cmd_vel).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${VIRTUAL_ENV:-}" ]]; then unset VIRTUAL_ENV; fi
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash" 2>/dev/null || true

JOY_DEV="${1:-0}"

if ! id -nG "${USER}" | tr ' ' '\n' | grep -qx input; then
  cat >&2 <<EOF
Joystick access needs Linux group: input.
Current groups: $(id -nG)

Run once:
  sudo usermod -aG input "${USER}"
  newgrp input

Then:
  ./scripts/deck_xbox_controller.sh
EOF
  exit 1
fi

if [[ ! -e /dev/input/js${JOY_DEV} ]]; then
  cat >&2 <<EOF
No joystick at /dev/input/js${JOY_DEV}.

Check:
  ls -l /dev/input/js*
  # Xbox One USB should appear after plugging in.
  # Bluetooth: pair in Settings, then re-check js devices.

Usage: ./scripts/deck_xbox_controller.sh [device_id]
  default device_id=0  →  /dev/input/js0
EOF
  exit 1
fi

if command -v ros2 >/dev/null 2>&1; then
  pub_count="$(ros2 topic info /cmd_vel 2>/dev/null | sed -n 's/Publisher count: //p' | head -1 || true)"
  if [[ -n "${pub_count}" && "${pub_count}" -ge 1 ]]; then
    cat >&2 <<EOF
Warning: /cmd_vel already has ${pub_count} publisher(s).

Stop keyboard teleop / PWM tune if running:
  pkill -f teleop_keyboard.py
  pkill -f motor_pwm_tune_gui_node

Two nodes on /cmd_vel causes stutter.
EOF
  fi
fi

echo "==> Xbox teleop  /dev/input/js${JOY_DEV}"
echo "    D-pad = drive | Y = faster | A = slower"
echo "    Stop keyboard teleop if running:  pkill -f teleop_keyboard.py"
echo "    Ctrl-C to quit"

JOY_PID=""
TELEOP_PID=""

cleanup() {
  if [[ -n "$TELEOP_PID" ]] && kill -0 "$TELEOP_PID" 2>/dev/null; then
    kill "$TELEOP_PID" 2>/dev/null || true
    wait "$TELEOP_PID" 2>/dev/null || true
  fi
  if [[ -n "$JOY_PID" ]] && kill -0 "$JOY_PID" 2>/dev/null; then
    kill "$JOY_PID" 2>/dev/null || true
    wait "$JOY_PID" 2>/dev/null || true
  fi
  # Best-effort stop command if agent is still up.
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

ros2 run joy joy_node --ros-args \
  -p "device_id:=${JOY_DEV}" \
  -p deadzone:=0.08 \
  -p autorepeat_rate:=50.0 \
  -p sticky_buttons:=false \
  -p coalesce_interval_ms:=1 &
JOY_PID=$!
sleep 0.4

python3 "$ROOT/scripts/deck_xbox_teleop.py" &
TELEOP_PID=$!

wait "$TELEOP_PID"
