#!/usr/bin/env bash
# Install and build Orbbec ros2_astra_camera for Astra Pro (OpenNI2 + UVC).
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ORBBEC_WS="$ROOT/orbbec_ws"

source /opt/ros/humble/setup.bash

echo "==> Installing apt dependencies..."
sudo apt-get update -qq
sudo apt-get install -y \
  libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-humble-image-geometry ros-humble-camera-info-manager \
  ros-humble-image-transport ros-humble-image-publisher \
  ros-humble-cv-bridge ros-humble-message-filters \
  git cmake build-essential pkg-config

if ! pkg-config --exists libuvc; then
  echo "==> Building libuvc..."
  TMPUVc="$(mktemp -d)"
  git clone --depth 1 https://github.com/libuvc/libuvc.git "$TMPUVc/libuvc"
  cmake -S "$TMPUVc/libuvc" -B "$TMPUVc/libuvc/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$TMPUVc/libuvc/build" -j"$(nproc)"
  sudo cmake --install "$TMPUVc/libuvc/build"
  sudo ldconfig
  rm -rf "$TMPUVc"
fi

mkdir -p "$ORBBEC_WS/src"
if [[ ! -d "$ORBBEC_WS/src/ros2_astra_camera" ]]; then
  echo "==> Cloning ros2_astra_camera..."
  git clone --depth 1 https://github.com/orbbec/ros2_astra_camera.git "$ORBBEC_WS/src/ros2_astra_camera"
fi

echo "==> Installing Orbbec udev rules..."
sudo bash "$ORBBEC_WS/src/ros2_astra_camera/astra_camera/scripts/install.sh"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "==> Building orbbec workspace..."
cd "$ORBBEC_WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1

echo ""
echo "Build complete. Source with:"
echo "  source $ORBBEC_WS/install/setup.bash"
echo "Launch Astra Pro:"
echo "  ros2 launch astra_camera astra_pro.launch.xml"
