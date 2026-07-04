#!/usr/bin/env bash
# Deck robot PWM feedforward tune GUI — same pattern as run_sim_teleop.sh.
# Hold WASD / mouse buttons to move; release to stop. Up/Down = speed, [ ] = PWM.
#
# Prerequisites (USB serial firmware — default esp32dev_l298n env):
#   Terminal 1: scripts/run_microros_agent.sh /dev/ttyUSB0
#   Flash:      cd esp32_base && pio run -e esp32dev_l298n -t upload
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2_ws"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source "${WS}/install/setup.bash"
else
  echo "ROS workspace not built. Run:"
  echo "  cd ${WS} && colcon build --packages-select indoor_bringup"
  exit 1
fi
set -u

if [ -z "${MOTOR_PWM_TUNE_NO_TERMINAL:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
  echo "Opening PWM tune GUI in a new gnome-terminal (keyboard works better than Cursor)..."
  exec gnome-terminal --title="Deck Robot PWM Tune" -- bash -lc \
    "export MOTOR_PWM_TUNE_NO_TERMINAL=1; cd '${ROOT}' && ./run_motor_pwm_tune.sh"
fi

exec ros2 launch indoor_bringup motor_pwm_tune.launch.py
