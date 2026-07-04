#!/usr/bin/env bash
# Shared helpers for ros2 bag recording.
set -eo pipefail

record_source_ros() {
  local root="$1"
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
    unset VIRTUAL_ENV PYTHONHOME
  fi
  source /opt/ros/humble/setup.bash
  source "$root/orbbec_ws/install/setup.bash"
  source "$root/ros2_ws/install/setup.bash"
}

record_wait_for_topics() {
  local timeout_s="${1:-30}"
  shift
  local topics=("$@")
  local elapsed=0
  echo "Waiting for topics (timeout ${timeout_s}s)..."
  while (( elapsed < timeout_s )); do
    local missing=()
    for topic in "${topics[@]}"; do
      if ! ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        missing+=("$topic")
      fi
    done
    if ((${#missing[@]} == 0)); then
      echo "All topics ready."
      return 0
    fi
    if (( elapsed % 5 == 0 )); then
      echo "  still waiting (${elapsed}s): ${missing[*]}"
    fi
    sleep 1
    ((elapsed++))
  done
  echo "ERROR: Topics not ready after ${timeout_s}s:" >&2
  echo "  missing: ${missing[*]}" >&2
  echo "  If /odom is missing: check Arduino USB (/dev/ttyACM0) and permissions." >&2
  ros2 topic list >&2 || true
  return 1
}

# Raw sensor topics consumed by mobile_mapping / RTAB-Map (see mobile_mapping.launch.py remappings).
# Do NOT record /tf or /tf_static — offline_slam.sh rebuilds TF from URDF + camera_static_tf.
RECORD_TOPICS_HANDHELD=(
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
  /camera/depth/camera_info
)

# Topics required before recording starts (do not block on optional extras).
RECORD_WAIT_TOPICS_MOBILE=(
  "${RECORD_TOPICS_HANDHELD[@]}"
  /odom
)

# RTAB-Map inputs + encoder telemetry for offline diagnostics.
RECORD_TOPICS_MOBILE=(
  "${RECORD_TOPICS_HANDHELD[@]}"
  /odom
  /arduino_feedback
)

# Sidecar metadata for offline replay (camera pitch/roll, fps).
record_write_mobile_meta() {
  local bag_dir="$1"
  local pitch="$2"
  local roll="$3"
  local fps="$4"
  cat > "${bag_dir}/record_meta.env" <<EOF
# Sensor settings used during ./record_mobile.sh (matches run_mobile_mapping.sh).
camera_pitch_deg=${pitch}
camera_roll_deg=${roll}
color_fps=${fps}
depth_fps=${fps}
recorded_at=$(date -Iseconds)
EOF
}

record_stop_all() {
  local root="$1"
  "$root/scripts/stop_camera.sh" 2>/dev/null || true
}
