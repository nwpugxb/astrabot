#!/usr/bin/env bash
# Live preview: RGB image + colored point cloud in RViz (no SLAM, no noisy map).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
"$ROOT/scripts/stop_camera.sh" || true

echo "Live preview: RViz shows colored point cloud, rqt shows RGB image."
echo "Tip: aim at textured indoor objects 0.5-4 m, avoid windows/sunlight."
exec ros2 launch astra_pro_slam live_preview.launch.py "$@"
