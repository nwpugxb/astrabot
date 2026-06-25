#!/usr/bin/env bash
# One-click: mobile robot SLAM (Arduino wheel odom + Orbbec + RTAB-Map).
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

echo "Mobile SLAM starting."
echo "  Camera pitch  : ${CAMERA_PITCH_DEG:-17} deg below horizontal (+ = down)"
echo "  Camera roll   : ${CAMERA_ROLL_DEG:-0} deg (fine tune floor plane: CAMERA_ROLL_DEG=1 ./run_mobile_mapping.sh)"
echo "  Terminal 2 (drive):  ./scripts/teleop.sh"
echo "  Stop mapping:        Ctrl-C here"
echo "  View in RViz:        ./view_mobile_map.sh"
echo "  View PLY (Open3D):   ./view_cloud.sh"

LAUNCH_ARGS=()
if [[ -n "${CAMERA_PITCH_DEG:-}" ]]; then
  LAUNCH_ARGS+=(camera_pitch_deg:="${CAMERA_PITCH_DEG}")
fi
if [[ -n "${CAMERA_ROLL_DEG:-}" ]]; then
  LAUNCH_ARGS+=(camera_roll_deg:="${CAMERA_ROLL_DEG}")
fi
exec ros2 launch mobile_base mobile_mapping.launch.py "${LAUNCH_ARGS[@]}" "$@"
