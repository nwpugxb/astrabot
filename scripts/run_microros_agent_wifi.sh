#!/usr/bin/env bash
# Start micro-ROS WiFi/UDP agent for ESP32 (indoor robot base).
# ESP32 connects to the same LAN and sends XRCE-DDS over UDP to this host.
# Requires agent built in ~/microros_ws — see docs/INDOOR_INSPECTION_ROBOT.md §7.
set -eo pipefail

PORT="${1:-8888}"

MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"
if [[ ! -f "$MICROROS_WS/install/local_setup.bash" ]]; then
  echo "micro-ROS agent workspace missing: $MICROROS_WS"
  echo "Build once (see docs/INDOOR_INSPECTION_ROBOT.md), then re-run this script."
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$MICROROS_WS/install/local_setup.bash"

echo "micro-ROS WiFi agent on UDP port $PORT (Ctrl-C to stop)"
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$PORT" -v6
