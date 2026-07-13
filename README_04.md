1. 先停 agent（跑 agent 的那个终端里 Ctrl+C，或执行）：
pkill -f "micro_ros_agent serial --dev /dev/ttyUSB0"
或：
pkill -f micro_ros_agent
pkill -f "pio device monitor"

2. 确认没人占串口：
fuser /dev/ttyUSB0
 无输出 = 空闲
 

# 串口通信
1. 刷机
~/Documents/orbbec_astro_pro/scripts/flash_deck_serial.sh
OR
cd ~/Documents/orbbec_astro_pro/esp32_base && pio run -e esp32dev_l298n -t upload
2. 启动
source /opt/ros/humble/setup.bash
source ~/Documents/orbbec_astro_pro/ros2_ws/install/setup.bash 
cd ~/Documents/orbbec_astro_pro && ./scripts/run_microros_agent.sh /dev/ttyUSB0

# wifi通信（底盘 + 雷达透传，同一固件 main_l298n）
1. 刷机
~/Documents/orbbec_astro_pro/scripts/flash_deck_wifi.sh
OR
cd ~/Documents/orbbec_astro_pro/esp32_base && pio run -e esp32dev_l298n_wifi -t upload
# 雷达接线 (SWAP=1): A1 TX->GPIO19, A1 RX->GPIO18, GND；A1 用外部 5V，勿插电脑 USB
2. 启动（micro-ROS agent + 雷达 relay/sllidar/RViz）
~/Documents/orbbec_astro_pro/scripts/run_deck_wifi_lidar.sh
3. 遥控（另一终端）
~/Documents/orbbec_astro_pro/scripts/deck_teleop.sh

# 如果build接收端代码
cd ~/Documents/orbbec_astro_pro/ros2_ws
colcon build --packages-select indoor_bringup mobile_base sllidar_ros2
source install/setup.bash

#检查话题
ros2 topic info /odom -v      # Reliability: BEST_EFFORT, Depth: 1
ros2 topic info /imu/data_raw -v
ros2 topic info /cmd_vel -v     # Reliability: RELIABLE
ros2 topic hz /scan
ros2 topic hz /cloud

#控制小车
source /opt/ros/humble/setup.bash
source ~/Documents/orbbec_astro_pro/ros2_ws/install/setup.bash  
./scripts/deck_teleop.sh

# 测试代码
// 持续前进（约 speed=30 档），10Hz
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.11}, angular: {z: 0.0}}"
// 另一个终端：持续覆盖前馈 PWM=80
ros2 topic pub -r 10 /motor_ff_pwm std_msgs/msg/Float32 "{data: 80.0}"

# ============================================================
# 仅雷达调试（独立固件 esp32dev_rplidar_bridge，可选）
# ============================================================
# 合并后请优先用上面的 flash_deck_wifi + run_deck_wifi_lidar。
# 独立桥接仍可用: scripts/flash_rplidar_bridge.sh / scripts/run_rplidar_wifi.sh

