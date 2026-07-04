#!/usr/bin/env bash
# Start micro-ROS serial agent for ESP32 (indoor robot base).
# Requires agent built in ~/microros_ws — see docs/INDOOR_INSPECTION_ROBOT.md §7.
set -eo pipefail

DEV="${1:-/dev/ttyUSB0}"
BAUD="${2:-921600}"

if [[ ! -e "$DEV" ]]; then
  echo "Serial device not found: $DEV"
  echo "Try: ls /dev/tty{USB,ACM}*"
  exit 1
fi

MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"
if [[ ! -f "$MICROROS_WS/install/local_setup.bash" ]]; then
  echo "micro-ROS agent workspace missing: $MICROROS_WS"
  echo "Build once (see docs/INDOOR_INSPECTION_ROBOT.md), then re-run this script."
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$MICROROS_WS/install/local_setup.bash"

echo "micro-ROS agent on $DEV @ $BAUD (Ctrl-C to stop)"
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b "$BAUD"
