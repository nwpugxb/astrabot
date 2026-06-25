#!/usr/bin/env bash
# Step 1 (handheld): record raw RGB-D + TF to a ROS 2 bag (no online SLAM).
#
# Usage:
#   ./record_handheld.sh              # default 15 Hz, bag under output/bags/
#   ./record_handheld.sh 30           # 30 Hz color/depth (larger bag)
#
# Stop recording: Ctrl+C in this terminal.
# Next step (offline SLAM): ./scripts/offline_slam.sh output/bags/handheld_YYYYMMDD_HHMMSS
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/record_common.sh
source "$ROOT/scripts/record_common.sh"

FPS="${1:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$ROOT/output/bags/handheld_${STAMP}"

record_source_ros "$ROOT"
record_stop_all "$ROOT"
mkdir -p "$ROOT/output/bags"

echo "=========================================="
echo " Handheld RAW recording (no SLAM)"
echo " Bag output: $BAG_DIR"
echo " FPS: color=${FPS} depth=${FPS}"
echo " Move camera slowly. Press Ctrl+C to stop."
echo "=========================================="

ros2 launch astra_pro_slam record_handheld.launch.py \
  color_fps:="${FPS}" depth_fps:="${FPS}" &
LAUNCH_PID=$!

cleanup() {
  echo ""
  echo "Stopping sensors..."
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  record_stop_all "$ROOT"
  if [[ -d "$BAG_DIR" ]]; then
    echo "Saved: $BAG_DIR"
    ros2 bag info "$BAG_DIR" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

record_wait_for_topics 40 "${RECORD_TOPICS_HANDHELD[@]}"

echo "Recording ros2 bag..."
ros2 bag record -o "$BAG_DIR" "${RECORD_TOPICS_HANDHELD[@]}"
