./scripts/flash_deck_wifi.sh
./scripts/run_deck_wifi_lidar.sh
./scripts/deck_teleop.sh

pkill -f deck_xbox_teleop.py; pkill -f joy_node
./scripts/deck_xbox_controller.sh

source /opt/ros/humble/setup.bash
ros2 topic echo /cmd_vel --qos-reliability best_effort

************************

pkill -f 'micro_ros_agent udp4' || true
pkill -f 'micro_ros_agent/micro_ros_agent' || true
sleep 0.5
pgrep -af micro_ros_agent || echo "agent stopped"

----->
ss -lun | grep 8888 || echo "port 8888 free — OK to start"  -> free!

0. 固件（若雷达之前关掉过）
确认 LIDAR_BRIDGE_ENABLE=1，建议建图时用开环开车：OPEN_LOOP_MOTOR=true（已帮你改回）。然后：

./scripts/flash_deck_wifi.sh
1. 底盘 + 雷达
./scripts/run_deck_wifi_lidar.sh
等 /scan 有数据（可另开窗口 ros2 topic hz /scan）。

./scripts/check_deck_odom.sh

2. SLAM
./scripts/run_deck_slam.sh
会起：URDF TF、/odom→base_footprint、slam_toolbox、RViz。

3. 遥控
./scripts/deck_teleop.sh
慢速走一圈，多转几个弯，让回环有机会闭合。RViz 里应看到 /map 逐渐长出来。

4. 存图
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/deck
得到 ~/maps/deck.pgm + ~/maps/deck.yaml。




********************


# 终端1（雷达 RViz 可关，避免两个窗口）
source /opt/ros/humble/setup.bash
source ~/Documents/orbbec_astro_pro/ros2_ws/install/setup.bash
ros2 launch indoor_bringup deck_wifi_lidar.launch.py use_rviz:=false
# 若 agent 也要一起开，仍用：
# ./scripts/run_deck_wifi_lidar.sh
# 然后在 launch 里不好关时，关掉多余的 rviz 即可
# 终端2（先 Ctrl-C 掉旧的 run_deck_slam）
./scripts/run_deck_slam.sh
# 终端3
./scripts/deck_teleop.sh
