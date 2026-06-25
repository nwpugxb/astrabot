#!/usr/bin/env bash
# One-click: handheld RGB-D SLAM with official Orbbec driver + RTAB-Map + RViz.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/orbbec_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

"$ROOT/scripts/stop_camera.sh" || true
mkdir -p "$ROOT/output"

echo "Starting handheld SLAM (Orbbec official driver + RTAB-Map)."
echo "Move the camera slowly. Press Ctrl-C to stop. Then run: ./view_cloud.sh"
exec ros2 launch astra_pro_slam handheld_mapping.launch.py "$@"
