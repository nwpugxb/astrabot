#!/usr/bin/env bash
# Step 1 (mobile robot): record raw RGB-D + wheel odom (same topics as mobile_mapping / RTAB-Map).
#
# Usage:
#   Terminal 1: ./record_mobile.sh
#   Terminal 2: ./scripts/teleop.sh   (optional, to drive while recording)
#
# Same camera TF as live mapping (override pitch/roll like run_mobile_mapping.sh):
#   CAMERA_PITCH_DEG=17 CAMERA_ROLL_DEG=0 ./record_mobile.sh
#
# Stop recording: Ctrl+C in terminal 1.
# Next step (offline SLAM): ./scripts/offline_slam.sh output/bags/mobile_YYYYMMDD_HHMMSS --mobile
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/record_common.sh
source "$ROOT/scripts/record_common.sh"

FPS="${1:-30}"
PITCH="${CAMERA_PITCH_DEG:-17.0}"
ROLL="${CAMERA_ROLL_DEG:-0.0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$ROOT/output/bags/mobile_${STAMP}"

record_source_ros "$ROOT"
record_stop_all "$ROOT"
mkdir -p "$ROOT/output/bags"

echo "=========================================="
echo " Mobile RAW recording (no SLAM)"
echo " Bag output: $BAG_DIR"
echo " FPS: color=${FPS} depth=${FPS} (same default as run_mobile_mapping.sh)"
echo " Camera pitch: ${PITCH} deg  roll: ${ROLL} deg"
echo " Drive (optional): ./scripts/teleop.sh"
echo " Press Ctrl+C here to stop recording."
echo "=========================================="

LAUNCH_ARGS=(
  "color_fps:=${FPS}"
  "depth_fps:=${FPS}"
  "camera_pitch_deg:=${PITCH}"
  "camera_roll_deg:=${ROLL}"
)

ros2 launch mobile_base record_mobile.launch.py "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

echo "Waiting for camera + Arduino to start..."
sleep 4

cleanup() {
  echo ""
  echo "Stopping sensors..."
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  record_stop_all "$ROOT"
  if [[ -d "$BAG_DIR" ]]; then
    record_write_mobile_meta "$BAG_DIR" "$PITCH" "$ROLL" "$FPS"
    echo "Saved: $BAG_DIR"
    echo "Metadata: $BAG_DIR/record_meta.env"
    ros2 bag info "$BAG_DIR" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

record_wait_for_topics 60 "${RECORD_WAIT_TOPICS_MOBILE[@]}"

echo "Recording ros2 bag..."
ros2 bag record -o "$BAG_DIR" "${RECORD_TOPICS_MOBILE[@]}"
