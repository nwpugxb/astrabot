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

# wifi通信
1. 刷机
cd ~/Documents/orbbec_astro_pro/scripts/flash_deck_wifi.sh
OR
cd ~/Documents/orbbec_astro_pro/esp32_base
pio run -e esp32dev_l298n_wifi -t upload
2. 启动
source /opt/ros/humble/setup.bash
source ~/Documents/orbbec_astro_pro/ros2_ws/install/setup.bash 
cd ~/Documents/orbbec_astro_pro && ./scripts/run_microros_agent_wifi.sh

# 如果build接收端代码
cd ~/Documents/orbbec_astro_pro/ros2_ws
colcon build --packages-select indoor_bringup mobile_base
source install/setup.bash

#检查话题
ros2 topic info /odom -v      # Reliability: BEST_EFFORT, Depth: 1
ros2 topic info /imu/data_raw -v
ros2 topic info /cmd_vel -v     # Reliability: RELIABLE

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

