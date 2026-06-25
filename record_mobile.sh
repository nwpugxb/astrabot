#!/usr/bin/env bash
# Step 1 (mobile robot): record raw RGB-D + wheel odom + TF (no online SLAM).
#
# Usage:
#   Terminal 1: ./record_mobile.sh
#   Terminal 2: ./scripts/teleop.sh   (optional, to drive while recording)
#
# Stop recording: Ctrl+C in terminal 1.
# Next step (offline SLAM): ./scripts/offline_slam.sh output/bags/mobile_YYYYMMDD_HHMMSS --mobile
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/record_common.sh
source "$ROOT/scripts/record_common.sh"

FPS="${1:-15}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$ROOT/output/bags/mobile_${STAMP}"

record_source_ros "$ROOT"
record_stop_all "$ROOT"
mkdir -p "$ROOT/output/bags"

echo "=========================================="
echo " Mobile RAW recording (no SLAM)"
echo " Bag output: $BAG_DIR"
echo " FPS: color=${FPS} depth=${FPS}"
echo " Drive (optional): ./scripts/teleop.sh"
echo " Press Ctrl+C here to stop recording."
echo "=========================================="

ros2 launch mobile_base record_mobile.launch.py \
  color_fps:="${FPS}" depth_fps:="${FPS}" &
LAUNCH_PID=$!

echo "Waiting for camera + Arduino to start..."
sleep 4

# depth -> scan for /scan topic (same as live mapping pipeline).
ros2 run depthimage_to_laserscan depthimage_to_laserscan_node \
  --ros-args \
  -r depth:=/camera/depth/image_raw \
  -r depth_camera_info:=/camera/depth/camera_info \
  -r scan:=/scan \
  -p scan_height:=1 \
  -p range_min:=0.08 \
  -p range_max:=4.0 \
  -p output_frame:=base_link &
SCAN_PID=$!

cleanup() {
  echo ""
  echo "Stopping sensors..."
  kill "$SCAN_PID" "$LAUNCH_PID" 2>/dev/null || true
  wait "$SCAN_PID" "$LAUNCH_PID" 2>/dev/null || true
  record_stop_all "$ROOT"
  if [[ -d "$BAG_DIR" ]]; then
    echo "Saved: $BAG_DIR"
    ros2 bag info "$BAG_DIR" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

record_wait_for_topics 60 "${RECORD_WAIT_TOPICS_MOBILE[@]}"

echo "Recording ros2 bag..."
ros2 bag record -o "$BAG_DIR" "${RECORD_TOPICS_MOBILE[@]}"
