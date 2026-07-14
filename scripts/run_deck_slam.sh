#!/usr/bin/env bash
# Deck live 2D SLAM. Needs in OTHER terminals:
#   ./scripts/run_deck_wifi_lidar.sh     # agent + /scan
#   ./scripts/deck_teleop.sh
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${VIRTUAL_ENV:-}" ]]; then unset VIRTUAL_ENV; fi
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

mkdir -p "$HOME/maps"
PARAMS="$ROOT/ros2_ws/src/indoor_bringup/config/slam_toolbox_deck.yaml"
RVIZ="$ROOT/ros2_ws/src/indoor_bringup/rviz/indoor_slam.rviz"

echo "==> Deck SLAM"
echo "    RViz Fixed Frame starts as 'odom'. After /map appears, switch to 'map'."
echo "    Save: ros2 run nav2_map_server map_saver_cli -f \$HOME/maps/deck"
echo ""

# Hard requirement: /odom from ESP32 via micro-ROS agent
if ! ros2 topic list 2>/dev/null | grep -qx /odom; then
  echo "WARNING: /odom not in topic list. Is agent + ESP32 up?"
  echo "  Start first:  ./scripts/run_deck_wifi_lidar.sh"
  echo "  Check:        ros2 topic hz /odom"
  echo "Continuing anyway (laser-only SLAM is weaker)..."
  echo ""
fi

PIDS=()
cleanup() {
  for p in "${PIDS[@]:-}"; do
    kill "$p" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# URDF provides base_footprint→base_link→laser on /tf_static
ros2 launch indoor_bringup description.launch.py &
PIDS+=($!)
sleep 0.4

python3 "$ROOT/scripts/deck_slam_sync.py" &
PIDS+=($!)
sleep 0.5

ros2 launch indoor_bringup slam.launch.py \
  slam_params_file:="$PARAMS" \
  scan_topic:=/scan_slam &
PIDS+=($!)
sleep 1

TMP_RVIZ="$(mktemp /tmp/deck_slam_XXXX.rviz)"
sed 's/Fixed Frame: map/Fixed Frame: odom/' "$RVIZ" > "$TMP_RVIZ"
ros2 run rviz2 rviz2 -d "$TMP_RVIZ" &
PIDS+=($!)

echo "==> Expect in logs: First /scan→/scan_slam  and  First /odom"
echo "    No /odom ⇒ micro-ROS not connected (car won't move in RViz)"
wait
