#!/usr/bin/env bash
# Clean start for Gazebo SLAM sim (mapping mode).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2_ws"

"${ROOT}/stop_sim.sh"

# ROS setup.bash references optional vars — do not use 'set -u' here.
set +u
source /opt/ros/humble/setup.bash
cd "${WS}"
colcon build --packages-select indoor_bringup
source install/setup.bash
set -u

export ROS_DOMAIN_ID=77
echo ""
echo "=============================================="
echo " Terminal 1: simulation (this script)"
echo " Terminal 2: after ~15s run:"
echo "   cd ${ROOT} && ./run_sim_teleop.sh"
echo " Click teleop window, use buttons/WASD — not this terminal."
echo " Health check: ./check_sim.sh"
echo "=============================================="
echo ""
ros2 launch indoor_bringup simulation.launch.py stack:=mapping headless:=true
