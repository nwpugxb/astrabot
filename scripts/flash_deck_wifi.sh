#!/usr/bin/env bash
# Flash deck robot firmware — WiFi UDP micro-ROS (no USB data link needed after flash).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT/esp32_base"
echo "==> Flashing esp32dev_l298n_wifi (WiFi transport)"
pio run -e esp32dev_l298n_wifi -t upload
echo ""
echo "Done. USB can be unplugged for power-only. Next:"
echo "  Control + lidar: $ROOT/scripts/run_deck_wifi_lidar.sh"
echo "  Teleop:          $ROOT/scripts/deck_teleop.sh"
echo "  (agent-only:     $ROOT/scripts/run_microros_agent_wifi.sh)"
echo "Lidar wiring (SWAP=1): A1 TX->GPIO19, A1 RX->GPIO18, GND"
