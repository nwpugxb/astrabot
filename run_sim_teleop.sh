#!/usr/bin/env bash
# Teleop GUI — prefer a standalone terminal so keyboard focus is not stolen by Cursor.
export ROS_DOMAIN_ID=77
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2_ws"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

if [ -z "${SIM_TELEOP_NO_TERMINAL:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
  echo "Opening teleop in a new gnome-terminal (keyboard works better than Cursor terminal)..."
  exec gnome-terminal --title="Indoor Sim Teleop" -- bash -lc \
    "export ROS_DOMAIN_ID=77 SIM_TELEOP_NO_TERMINAL=1; cd '${ROOT}' && ./run_sim_teleop.sh"
fi

ros2 launch indoor_bringup sim_teleop.launch.py
