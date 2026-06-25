#!/usr/bin/env bash
# Stop camera / SLAM processes that hold the Orbbec device.
pkill -f "astra_pro_slam/camera_node" 2>/dev/null || true
pkill -f "astra_camera_node" 2>/dev/null || true
pkill -f "astra_camera astra_pro" 2>/dev/null || true
pkill -f "mobile_mapping.launch.py" 2>/dev/null || true
pkill -f "arduino_base" 2>/dev/null || true
pkill -f "live_preview.launch.py" 2>/dev/null || true
ros2 run astra_camera clean_shm_node 2>/dev/null || true
sleep 0.5
if fuser /dev/video2 >/dev/null 2>&1; then
  echo "Warning: /dev/video2 still in use. Try unplug/replug the camera." >&2
  fuser -v /dev/video2 2>&1 || true
else
  echo "Camera device is free."
fi
