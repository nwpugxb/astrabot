1. 先停 agent（跑 agent 的那个终端里 Ctrl+C，或执行）：
pkill -f "micro_ros_agent serial --dev /dev/ttyUSB0"

2. 确认没人占串口：
fuser /dev/ttyUSB0
 无输出 = 空闲



确认烧的是 L298N 固件
cd ~/Documents/orbbec_astro_pro/esp32_base
# 先停 agent（Ctrl+C），再烧录
pio run -e esp32dev_l298n -t upload

*****************

# 若还在跑，先 Ctrl+C
~/Documents/orbbec_astro_pro/scripts/run_microros_agent.sh /dev/ttyUSB0
agent 已经在跑的情况下，按 EN（或拔插 USB 后再启动 agent）。

*****************

source /opt/ros/humble/setup.bash
ros2 topic list
sleep 10
ros2 topic list

ros2 topic echo /imu/data_raw --once

*****************

# 调试低速试转（轮子架空或手能挡住）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 10
# 几秒后 Ctrl+C 停

# 键盘 teleop（和原来 teleop.sh 一样的 WASD 按住走）
./scripts/deck_teleop.sh

# ToF 
source /opt/ros/humble/setup.bash
ros2 topic echo /tof_front
ros2 topic echo /tof_left
ros2 topic echo /tof_right
