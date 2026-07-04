#!/usr/bin/env bash
# evdev WASD teleop for ESP32 deck robot (micro-ROS /cmd_vel).
# Same keys as teleop.sh, but publishes Twist instead of /arduino_teleop_cmd.
#
# Prerequisites (pick one transport — must match flashed firmware):
#   USB serial: scripts/run_microros_agent.sh /dev/ttyUSB0
#   WiFi:       scripts/run_microros_agent_wifi.sh
#   ./scripts/setup_teleop_input.sh  (once, for input group)
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${VIRTUAL_ENV:-}" ]]; then unset VIRTUAL_ENV; fi
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash" 2>/dev/null || true

if ! id -nG "${USER}" | tr ' ' '\n' | grep -qx input; then
  cat >&2 <<EOF
Teleop needs read access to /dev/input/event* (Linux group: input).
Current groups: $(id -nG)

Run once:
  sudo usermod -aG input "${USER}"
  newgrp input

Then run:
  ./scripts/deck_teleop.sh
EOF
  exit 1
fi

exec python3 "$ROOT/scripts/teleop_keyboard.py" --cmd-vel "$@"
