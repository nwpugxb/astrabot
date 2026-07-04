#!/usr/bin/env bash
# Flash deck robot firmware — USB serial micro-ROS (default for PWM tune / teleop).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/esp32_base"
echo "==> Flashing esp32dev_l298n (USB serial transport)"
pio run -e esp32dev_l298n -t upload
echo ""
echo "Done. Next:"
echo "  Terminal 1: $ROOT/scripts/run_microros_agent.sh /dev/ttyUSB0   # 921600 baud"
echo "  Terminal 2: $ROOT/scripts/deck_teleop.sh   OR   $ROOT/run_motor_pwm_tune.sh"
