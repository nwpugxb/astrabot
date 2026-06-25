#!/usr/bin/env bash
# View bag RGB-D point cloud in RViz (Fixed Frame = odom).
#
# What to check:
#   - Red X / green Y / blue Z at odom origin: X forward, Z up (REP-103).
#   - Yellow odom path should move mostly along +X when driving forward.
#   - Colored point cloud floor should lie on the gray XY grid (not vertical).
#
# If floor is perpendicular to +X, camera pitch TF is wrong. Tune without rebuild:
#   CAMERA_PITCH_DEG=30 ./scripts/view_bag_cloud.sh
#   CAMERA_PITCH_DEG=45 ./scripts/view_bag_cloud.sh
#
# Usage:
#   ./scripts/view_bag_cloud.sh
#   ./scripts/view_bag_cloud.sh mobile_20260623_212328
#   ./scripts/view_bag_cloud.sh output/bags/mobile_20260623_212328
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/bag_path.sh
source "$ROOT/scripts/bag_path.sh"

BAG="$(resolve_bag_path "$ROOT" "${1:-latest}")"
CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-17.0}"
LOOP="${LOOP:-true}"

if [[ ! -d "$BAG" ]]; then
  echo "Usage: $0 [bag_directory|bag_name|latest]" >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/orbbec_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

echo "=== Bag point cloud viewer ==="
echo "  Bag           : $BAG"
echo "  Camera pitch  : ${CAMERA_PITCH_DEG} deg below horizontal"
echo "  Fixed Frame   : odom (X forward, Z up)"
echo "  Ctrl+C to stop"
echo ""

cleanup() {
  echo "Stopping..."
  kill "$LAUNCH_PID" "$PLAY_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  wait "$PLAY_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 launch mobile_base view_bag_cloud.launch.py \
  use_sim_time:=true \
  camera_pitch_deg:="$CAMERA_PITCH_DEG" &
LAUNCH_PID=$!

echo "Waiting for RViz / TF..."
sleep 4

PLAY_ARGS=(ros2 bag play "$BAG" --clock
  --topics
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
  /odom
  --remap /odom:=/odom_bag
)
if [[ "$LOOP" == true ]]; then
  PLAY_ARGS+=(--loop)
fi

"${PLAY_ARGS[@]}" &
PLAY_PID=$!

wait "$LAUNCH_PID" 2>/dev/null || true
