cd ~/Documents/orbbec_astro_pro
./stop_sim.sh

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select indoor_bringup
source install/setup.bash
cd ..

./run_sim_mapping.sh



cd ~/Documents/orbbec_astro_pro
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=77

实验 1 — 建图（SLAM）
# 终端 1
./stop_sim.sh
./run_sim_mapping.sh          # 等 ~15s，RViz 打开
# 终端 2
./run_sim_teleop.sh           # 用按钮按住走，绕房间一圈
# 终端 3
./check_sim.sh                # 确认 /scan、TF 正常

ros2 run nav2_map_server map_saver_cli -f ~/maps/sim_slam   #存图
ls -lh ~/maps/sim_slam.*
