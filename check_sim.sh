#!/usr/bin/env bash
# Quick health check — run while simulation.launch.py is up (~20s after start).
export ROS_DOMAIN_ID=77
source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/ros2_ws" && pwd)/install/setup.bash" 2>/dev/null || {
  echo "ERROR: ros2_ws not built. Run: cd ros2_ws && colcon build --packages-select indoor_bringup"
  exit 1
}

ok=0
fail=0
check() {
  local name="$1" cmd="$2"
  if eval "$cmd" >/dev/null 2>&1; then
    echo "  OK   $name"
    ok=$((ok + 1))
  else
    echo "  FAIL $name"
    fail=$((fail + 1))
  fi
}

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "Nodes:"
ros2 node list 2>/dev/null | grep -E 'sim_gazebo_adapter|controller_manager|slam_toolbox|indoor_robot_rsp|sim_slam_scan_viz' || echo "  (no sim nodes — is simulation.launch.py running?)"
echo ""
echo "Checks (wait up to 5s each; need ~20s after launch for controllers):"
check "sim_gazebo_adapter running" "ros2 node list | grep -q '^/sim_gazebo_adapter$'"
check "/scan publishing" "timeout 5 ros2 topic hz /scan --window 10 | grep -q average"
check "sim_gt_odom running" "ros2 node list | grep -q '^/sim_gt_odom$'"
check "/sim/gt_odom publishing" "timeout 5 ros2 topic hz /sim/gt_odom --window 10 | grep -q average"
check "TF odom->base_footprint" "timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -q Translation"
check "scan viz clouds" "timeout 5 ros2 topic hz /sim/slam/current_cloud --window 10 | grep -q average"
check "cmd_vel chain" "ros2 topic info /diff_drive_controller/cmd_vel_unstamped | grep -q 'Subscription count: 1'"
check "scan frame=laser" "timeout 3 ros2 topic echo /scan --once 2>&1 | grep -q 'frame_id: laser'"

echo ""
if [ "$fail" -eq 0 ]; then
  echo "All checks passed. Teleop:"
  echo "  cd $(dirname "${BASH_SOURCE[0]}") && ./run_sim_teleop.sh"
else
  echo "$fail check(s) failed."
  echo "Fix: ./stop_sim.sh && ./run_sim_mapping.sh  (wait 20s, run this script again)"
fi
exit "$fail"
