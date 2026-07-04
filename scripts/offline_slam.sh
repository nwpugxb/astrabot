#!/usr/bin/env bash
# Step 2: play a recorded bag and run RTAB-Map offline (no live camera).
#
# Usage:
#   ./scripts/offline_slam.sh output/bags/mobile_xxx --mobile
#   ./scripts/offline_slam.sh output/bags/mobile_xxx --mobile --profile s2
#   ./scripts/offline_slam.sh output/bags/mobile_xxx --mobile --profile s3 --no-rviz
#
# Profiles (--mobile only):
#   baseline  default visual + wheel odom (same as before)
#   s2        scheme 2: Reg/Strategy=2 visual+ICP
#   s3        scheme 3: rgbd_odometry + graph optimization / loop closure
#   s2s3      scheme 2+3 combined
#   icp_loop  ICP + loop closure / global BA (recommended, no VO)
#   icp_plane  height-band wall plane cloud + 2D ICP + loop BA
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/record_common.sh
source "$ROOT/scripts/record_common.sh"

BAG="${1:-}"
MODE="handheld"
BAG_TF=false
PROFILE="baseline"
RVIZ=true
for arg in "$@"; do
  case "$arg" in
    --mobile) MODE="mobile" ;;
    --bag-tf) BAG_TF=true ;;
    --no-rviz) RVIZ=false ;;
    --profile=*) PROFILE="${arg#--profile=}" ;;
    --profile)
      ;;
  esac
done
# --profile s2 (two-arg form)
for ((i = 1; i < $#; i++)); do
  if [[ "${!i}" == "--profile" ]]; then
    next=$((i + 1))
    if [[ $next -le $# ]]; then
      PROFILE="${!next}"
    fi
  fi
done

if [[ -z "$BAG" || ! -d "$BAG" ]]; then
  echo "Usage: $0 <bag_directory> [--mobile] [--profile baseline|s2|s3|s2s3|icp_loop|icp_plane] [--no-rviz] [--bag-tf]" >&2
  exit 1
fi

if [[ -f "$BAG/record_meta.env" ]]; then
  # shellcheck source=/dev/null
  source "$BAG/record_meta.env"
  echo "  Loaded bag metadata: $BAG/record_meta.env"
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi
source /opt/ros/humble/setup.bash
source "$ROOT/orbbec_ws/install/setup.bash"
source "$ROOT/ros2_ws/install/setup.bash"

STAMP="$(basename "$BAG")"
CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline.yaml"
USE_RGBD_ODOM=false
USE_WALL_PLANE_CLOUD=false
DB_SUFFIX=""
PLAY_RATE="1.0"
case "$PROFILE" in
  baseline)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline.yaml"
    DB_SUFFIX=""
    ;;
  s2)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline_s2_visicp.yaml"
    DB_SUFFIX="_s2_visicp"
    ;;
  s3)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline_s3_vo.yaml"
    DB_SUFFIX="_s3_vo"
    USE_RGBD_ODOM=true
    ;;
  s2s3)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline_s2s3_visicp_vo.yaml"
    DB_SUFFIX="_s2s3"
    USE_RGBD_ODOM=true
    ;;
  icp_loop)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline_icp_loop.yaml"
    DB_SUFFIX="_icp_loop"
    PLAY_RATE="0.7"
    ;;
  icp_plane)
    CONFIG="$ROOT/ros2_ws/install/mobile_base/share/mobile_base/config/rtabmap_mobile_offline_plane_icp.yaml"
    DB_SUFFIX="_icp_plane"
    USE_WALL_PLANE_CLOUD=true
    PLAY_RATE="0.6"
    ;;
  *)
    echo "Unknown profile: $PROFILE (use baseline, s2, s3, s2s3, icp_loop, icp_plane)" >&2
    exit 1
    ;;
esac

if [[ ! -f "$CONFIG" ]]; then
  CONFIG="${CONFIG/install/src}"
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "RTAB-Map config not found for profile $PROFILE. Run: colcon build --packages-select mobile_base" >&2
  exit 1
fi

DB="$ROOT/output/offline_${STAMP}${DB_SUFFIX}.db"
mkdir -p "$ROOT/output"

echo "Offline SLAM"
echo "  Bag    : $BAG"
echo "  DB     : $DB"
echo "  Mode   : $MODE"
echo "  Profile: $PROFILE"
echo "  Config : $CONFIG"
echo "  VO node: $USE_RGBD_ODOM"
echo "  Wall plane cloud: $USE_WALL_PLANE_CLOUD"
echo "  RViz   : $RVIZ"
if [[ "$BAG_TF" == true ]]; then
  echo "  TF     : from bag (legacy)"
else
  echo "  TF     : current URDF + camera static (bag /tf not used)"
fi
echo "Press Ctrl+C to stop."

CAMERA_PITCH_DEG="${CAMERA_PITCH_DEG:-17.0}"
CAMERA_ROLL_DEG="${CAMERA_ROLL_DEG:-0.0}"
echo "  Camera pitch: ${CAMERA_PITCH_DEG} deg  roll: ${CAMERA_ROLL_DEG} deg"

# 1) TF from current URDF
if [[ "$MODE" == "mobile" && "$BAG_TF" != true ]]; then
  ros2 launch mobile_base offline_replay_tf.launch.py \
    use_sim_time:=true \
    camera_pitch_deg:="$CAMERA_PITCH_DEG" \
    camera_roll_deg:="$CAMERA_ROLL_DEG" &
  TF_PID=$!
elif [[ "$MODE" == "handheld" && "$BAG_TF" != true ]]; then
  ros2 launch astra_pro_slam offline_handheld_tf.launch.py &
  TF_PID=$!
else
  TF_PID=""
fi
sleep 2

# 2) RTAB-Map + RViz MUST be up BEFORE bag play
RVIZ_ARG="rviz:=true"
if [[ "$RVIZ" == false ]]; then
  RVIZ_ARG="rviz:=false"
fi

if [[ "$MODE" == "mobile" ]]; then
  ros2 launch mobile_base offline_mobile_slam.launch.py \
    delete_db:=true \
    database_path:="$DB" \
    "$RVIZ_ARG" \
    use_sim_time:=true \
    rtabmap_config:="$CONFIG" \
    use_rgbd_odometry:="$USE_RGBD_ODOM" \
    use_wall_plane_cloud:="$USE_WALL_PLANE_CLOUD" &
else
  ros2 launch astra_pro_slam offline_handheld_slam.launch.py \
    delete_db:=true database_path:="$DB" "$RVIZ_ARG" use_sim_time:=true &
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
    ros2 bag play "$BAG" --clock --rate "$PLAY_RATE" --topics "${PLAY_TOPICS[@]}" \
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
    echo "Export PLY:   ./scripts/export_map.sh $DB output/offline_${STAMP}${DB_SUFFIX}.ply"
    echo "View RViz:    ./view_mobile_map.sh  # then set database_path manually or use rtabmap_viz"
  else
    echo "WARNING: database not found: $DB" >&2
  fi
}
trap cleanup EXIT INT TERM

wait "$PLAY_PID" 2>/dev/null || true
echo "Bag playback finished."
echo "Waiting for RTAB-Map to drain queue (30s)..."
sleep 30
