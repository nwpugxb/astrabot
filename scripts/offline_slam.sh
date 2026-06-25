#!/usr/bin/env bash
# Step 2: play a recorded bag and run RTAB-Map offline (no live camera).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/record_common.sh
source "$ROOT/scripts/record_common.sh"

BAG="${1:-}"
MODE="handheld"
BAG_TF=false
for arg in "$@"; do
  case "$arg" in
    --mobile) MODE="mobile" ;;
    --bag-tf) BAG_TF=true ;;
  esac
done

if [[ -z "$BAG" || ! -d "$BAG" ]]; then
  echo "Usage: $0 <bag_directory> [--mobile] [--bag-tf]" >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi
source /opt/ros/humble/setup.bash
source "$ROOT/orbbec_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

STAMP="$(basename "$BAG")"
DB="$ROOT/output/offline_${STAMP}.db"
mkdir -p "$ROOT/output"

echo "Offline SLAM"
echo "  Bag: $BAG"
echo "  DB : $DB"
echo "  Mode: $MODE"
if [[ "$BAG_TF" == true ]]; then
  echo "  TF : from bag (legacy)"
else
  echo "  TF : current URDF + camera static (bag /tf not used)"
fi
echo "Press Ctrl+C to stop."

CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-17.0}"
echo "  Camera pitch: ${CAMERA_PITCH_DEG} deg below horizontal (+ = down; override with CAMERA_PITCH_DEG=...)"

# 1) TF from current URDF
if [[ "$MODE" == "mobile" && "$BAG_TF" != true ]]; then
  ros2 launch mobile_base offline_replay_tf.launch.py \
    use_sim_time:=true camera_pitch_deg:="$CAMERA_PITCH_DEG" &
  TF_PID=$!
elif [[ "$MODE" == "handheld" && "$BAG_TF" != true ]]; then
  ros2 launch astra_pro_slam offline_handheld_tf.launch.py &
  TF_PID=$!
else
  TF_PID=""
fi
sleep 2

# 2) RTAB-Map + RViz MUST be up BEFORE bag play
if [[ "$MODE" == "mobile" ]]; then
  ros2 launch mobile_base offline_mobile_slam.launch.py \
    delete_db:=true database_path:="$DB" rviz:=true use_sim_time:=true &
else
  ros2 launch astra_pro_slam offline_handheld_slam.launch.py \
    delete_db:=true database_path:="$DB" rviz:=true use_sim_time:=true &
fi
SLAM_PID=$!
echo "Waiting for RTAB-Map to subscribe..."
sleep 8

# 3) Play bag after SLAM is ready
if [[ "$BAG_TF" == true ]]; then
  ros2 bag play "$BAG" --clock &
else
  PLAY_TOPICS=(
    /camera/color/image_raw
    /camera/color/camera_info
    /camera/depth/image_raw
    /camera/depth/camera_info
  )
  if [[ "$MODE" == "mobile" ]]; then
    PLAY_TOPICS+=(/odom)
    ros2 bag play "$BAG" --clock --topics "${PLAY_TOPICS[@]}" \
      --remap /odom:=/odom_bag &
  else
    ros2 bag play "$BAG" --clock --topics "${PLAY_TOPICS[@]}" &
  fi
fi
PLAY_PID=$!

cleanup() {
  echo "Stopping..."
  kill "$PLAY_PID" 2>/dev/null || true
  wait "$PLAY_PID" 2>/dev/null || true
  echo "Flushing map (5s)..."
  sleep 5
  kill "$SLAM_PID" "$TF_PID" 2>/dev/null || true
  "$ROOT/scripts/stop_camera.sh" 2>/dev/null || true
  if [[ -f "$DB" ]]; then
    echo "Map database: $DB ($(du -h "$DB" | cut -f1))"
    echo "Export: ./scripts/export_map.sh $DB"
  else
    echo "WARNING: database not found: $DB" >&2
  fi
}
trap cleanup EXIT INT TERM

wait "$PLAY_PID" 2>/dev/null || true
echo "Bag playback finished."
