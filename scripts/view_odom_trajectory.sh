#!/usr/bin/env bash
# Second RViz window: wheel odometry path only (Fixed Frame = odom).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${VIRTUAL_ENV:-}" ]]; then unset VIRTUAL_ENV; fi
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash" 2>/dev/null || true

# Path publisher (no-op if already running from mobile launch).
ros2 run mobile_base odom_path_node 2>/dev/null &
PATH_PID=$!
sleep 1

cleanup() { kill "$PATH_PID" 2>/dev/null || true; }
trap cleanup EXIT INT TERM

echo "Odometry trajectory RViz (Fixed Frame = odom)"
echo "  Topic: /odom/path (green path on XY ground plane)"
echo "Close RViz to exit."
rviz2 -d "$ROOT/ros2_ws/src/mobile_base/rviz/odom_trajectory.rviz"
