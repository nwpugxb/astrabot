#!/usr/bin/env bash
# One-click: Orbbec official OpenNI2 driver for Astra Pro + RViz + rqt image.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/orbbec_ws/install/setup.bash"

# Stop our custom astra_raw driver if running.
"$ROOT/scripts/stop_camera.sh" 2>/dev/null || true
pkill -f astra_camera 2>/dev/null || true
ros2 run astra_camera clean_shm_node 2>/dev/null || true
sleep 1

RVIZ_CFG="$ROOT/rviz/orbbec_live.rviz"

echo "Starting Orbbec official driver (OpenNI2 + UVC) for Astra Pro..."
echo "  RGB + depth ~30Hz, hardware depth registration, colored point cloud"
echo "Press Ctrl-C to stop."

ros2 launch astra_camera astra_pro.launch.xml \
  enable_colored_point_cloud:=true \
  depth_registration:=true \
  enable_point_cloud:=true \
  color_depth_synchronization:=true \
  oni_log_level:=warning \
  oni_log_to_console:=false &
CAM_PID=$!

sleep 6
if ! kill -0 "$CAM_PID" 2>/dev/null; then
  echo "Camera node failed to start. Check USB connection and run:"
  echo "  ros2 run astra_camera list_devices_node"
  exit 1
fi

ros2 run rqt_image_view rqt_image_view /camera/color/image_raw &
RQT_PID=$!

rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

cleanup() {
  kill "$CAM_PID" "$RQT_PID" "$RVIZ_PID" 2>/dev/null || true
  pkill -f astra_camera 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait "$CAM_PID"
