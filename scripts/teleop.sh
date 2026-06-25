#!/usr/bin/env bash
# Curses WASD teleop — hold W/A/S/D to move, release to stop (needs evdev/input group).
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
  ./scripts/teleop.sh
EOF
  exit 1
fi

exec python3 "$ROOT/scripts/teleop_keyboard.py" "$@"
