#!/usr/bin/env bash
# Deck ESP32 (main_l298n_wifi): micro-ROS control + RPLIDAR TCP bridge on one board.
#
# ESP32 must be flashed:
#   pio run -e esp32dev_l298n_wifi -t upload
# Wiring lidar (SWAP=0): A1 TX->GPIO18, A1 RX->GPIO19, GND
#
# This script starts:
#   1) micro-ROS UDP agent :8888
#   2) lidar relay :20108 + sllidar + /cloud + RViz
# Teleop (optional, other terminal): ./scripts/deck_teleop.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

MICROROS_WS="${MICROROS_WS:-$HOME/microros_ws}"
if [[ ! -f "$MICROROS_WS/install/local_setup.bash" ]]; then
  echo "micro-ROS agent workspace missing: $MICROROS_WS"
  exit 1
fi

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$MICROROS_WS/install/local_setup.bash"
if [[ -f "$ROOT/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/ros2_ws/install/setup.bash"
fi

AGENT_PORT="${1:-8888}"
AGENT_PID=""

cleanup() {
  if [[ -n "$AGENT_PID" ]] && kill -0 "$AGENT_PID" 2>/dev/null; then
    kill "$AGENT_PID" 2>/dev/null || true
    wait "$AGENT_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Free stale listeners from a previous Ctrl-C that left the agent running.
free_port() {
  local port="$1"
  local pids
  pids="$(ss -H -lntp "sport = :${port}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | sort -u || true)"
  if [[ -z "$pids" ]]; then
    return 0
  fi
  echo "==> Freeing port ${port} (pids: ${pids})"
  # shellcheck disable=SC2086
  kill $pids 2>/dev/null || true
  sleep 0.5
  pids="$(ss -H -lntp "sport = :${port}" 2>/dev/null | grep -oP 'pid=\K[0-9]+' | sort -u || true)"
  if [[ -n "$pids" ]]; then
    # shellcheck disable=SC2086
    kill -9 $pids 2>/dev/null || true
    sleep 0.3
  fi
}

free_port "$AGENT_PORT"
free_port 20108
free_port 20109
pkill -f 'micro_ros_agent udp4' 2>/dev/null || true
sleep 0.3

echo "==> micro-ROS UDP agent :${AGENT_PORT}"
ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT" -v4 &
AGENT_PID=$!
sleep 1
if ! kill -0 "$AGENT_PID" 2>/dev/null; then
  echo "ERROR: micro-ROS agent failed to start (port ${AGENT_PORT} still busy?)"
  exit 1
fi

echo "==> Lidar relay + sllidar + RViz (ESP32 → :20108)"
echo "    Teleop: $ROOT/scripts/deck_teleop.sh"
echo "    Lidar wire (SWAP=1): A1 TX->GPIO19, A1 RX->GPIO18, GND"
exec ros2 launch indoor_bringup deck_wifi_lidar.launch.py
