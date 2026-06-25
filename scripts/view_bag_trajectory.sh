#!/usr/bin/env bash
# One-click: replay bag /odom and show wheel trajectory in RViz (Fixed Frame = odom).
#
# Usage:
#   ./scripts/view_bag_trajectory.sh
#   ./scripts/view_bag_trajectory.sh mobile_20260623_212328
#   ./scripts/view_bag_trajectory.sh output/bags/mobile_20260623_212328
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/bag_path.sh
source "$ROOT/scripts/bag_path.sh"

BAG="$(resolve_bag_path "$ROOT" "${1:-latest}")"
LOOP="${LOOP:-true}"

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

RVIZ_CONFIG="$ROOT/ros2_ws/src/mobile_base/rviz/odom_trajectory.rviz"

echo "=== Bag trajectory viewer (RViz) ==="
echo "  Bag         : $BAG"
echo "  Fixed Frame : odom"
echo "  Yellow path : /odom/path"
echo "  Ctrl+C to stop"
echo ""

cleanup() {
  echo "Stopping..."
  kill "$PATH_PID" "$PLAY_PID" "$RVIZ_PID" 2>/dev/null || true
  wait "$PATH_PID" "$PLAY_PID" "$RVIZ_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run mobile_base odom_path_node --ros-args -p use_sim_time:=true &
PATH_PID=$!

rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!

sleep 2

PLAY_ARGS=(ros2 bag play "$BAG" --clock --topics /odom)
if [[ "$LOOP" == true ]]; then
  PLAY_ARGS+=(--loop)
fi
"${PLAY_ARGS[@]}" &
PLAY_PID=$!

wait "$RVIZ_PID" 2>/dev/null || true
