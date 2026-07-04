#!/usr/bin/env bash
# Stop all indoor Gazebo / bringup processes (run before starting sim).
echo "Stopping indoor simulation processes..."

_kill() {
  pkill -TERM -f "$1" 2>/dev/null || true
}

_kill9() {
  pkill -KILL -f "$1" 2>/dev/null || true
}

# Graceful stop first.
_kill 'indoor_sim.sdf'
_kill 'ign gazebo'
_kill 'ros2 launch indoor_bringup'
_kill 'parameter_bridge'
_kill 'image_bridge'
_kill 'sim_gazebo_adapter'
_kill 'sim_gt_odom'
_kill 'sim_slam_scan_viz'
_kill 'sim_world_viz'
_kill 'sim_camera_info'
_kill 'gazebo_tof_range'
_kill 'slam_toolbox'
_kill 'async_slam_toolbox'
_kill 'robot_state_publisher.*indoor_robot'
_kill 'sim_teleop_gui'
_kill 'rviz2'
sleep 2

# Force kill stubborn Gazebo server processes.
_kill9 'indoor_sim.sdf'
_kill9 'ign gazebo'
_kill9 'gz sim'
_kill9 'ruby /usr/bin/ign gazebo'
_kill9 'ros2 launch indoor_bringup'
_kill9 'sim_gazebo_adapter'
_kill9 'sim_gt_odom'
_kill9 'sim_slam_scan_viz'
sleep 1

rm -rf /dev/shm/fastrtps_* 2>/dev/null || true

echo "Done. Remaining Gazebo/sim processes:"
remaining=$(pgrep -af 'indoor_sim\.sdf|ros2 launch indoor_bringup|sim_gazebo_adapter|sim_gt_odom' 2>/dev/null || true)
if [ -n "$remaining" ]; then
  echo "$remaining"
  echo ""
  echo "Force kill: kill -9 \$(pgrep -f 'indoor_sim.sdf')"
else
  echo "  (none)"
fi
