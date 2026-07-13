#!/usr/bin/env bash
# Host-as-server RPLIDAR: ESP32 connects to this PC (no ESP32 IP needed).
# ESP32 HOST_IP must match this machine (see esp32_base/include/config_rplidar_bridge.h).
#
# Usage:
#   ./scripts/run_rplidar_wifi.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/install/setup.bash"
fi

echo "Starting host TCP server (ESP32 → this PC:20108 → sllidar @127.0.0.1:20109)"
echo "Ensure ESP32 HOST_IP matches this PC (default 192.168.1.12)."
exec ros2 launch indoor_bringup rplidar_wifi.launch.py
