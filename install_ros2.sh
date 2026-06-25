#!/usr/bin/env bash
# Build ROS2 workspace and install Python deps for the camera driver.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

echo "==> Installing Python camera dependencies..."
if [[ -d "$ROOT/venv" ]]; then
  source "$ROOT/venv/bin/activate"
else
  python3 -m venv "$ROOT/venv"
  source "$ROOT/venv/bin/activate"
fi
pip install -q -U pip
pip install -q -r "$ROOT/requirements.txt"
# ROS2 console_scripts use system python3.
/usr/bin/pip3 install -q --user -r "$ROOT/requirements.txt"

echo "==> Building ROS2 workspace..."
source /opt/ros/humble/setup.bash
cd "$ROOT/ros2_ws"
colcon build --symlink-install

echo
echo "Build complete. Source the workspace before use:"
echo "  source $ROOT/ros2_ws/install/setup.bash"
echo
echo "Start handheld mapping:"
echo "  ros2 launch astra_pro_slam handheld_mapping.launch.py"
