#!/usr/bin/env bash
# Flash ESP32 RPLIDAR A1 UART ↔ WiFi TCP bridge.
#
# Usage:
#   ./scripts/flash_rplidar_bridge.sh              # auto port (risky if A1 also plugged)
#   ./scripts/flash_rplidar_bridge.sh /dev/ttyUSB1 # explicit ESP32 port
#
# Tip: unplug the RPLIDAR USB adapter while flashing so only the ESP32
# appears under /dev/ttyUSB* or /dev/ttyACM*.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-}"

cd "$ROOT/esp32_base"
echo "==> Flashing esp32dev_rplidar_bridge"

echo "Current serial devices:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none)"
if [[ -d /dev/serial/by-id ]]; then
  echo "by-id:"
  ls -la /dev/serial/by-id/ 2>/dev/null || true
fi

if [[ -n "$PORT" ]]; then
  if [[ ! -e "$PORT" ]]; then
    echo "ERROR: port $PORT not found"
    exit 1
  fi
  echo "Using upload_port=$PORT"
  pio run -e esp32dev_rplidar_bridge -t upload --upload-port "$PORT"
else
  echo ""
  echo "WARNING: No port given. PlatformIO will auto-pick the first ttyUSB/ACM."
  echo "If RPLIDAR is also plugged in, that is often /dev/ttyUSB0 and flash will fail."
  echo "Prefer: unplug A1, or pass the ESP32 port explicitly."
  echo ""
  pio run -e esp32dev_rplidar_bridge -t upload
fi

echo ""
echo "Done. ESP32 will connect to HOST_IP:20108 (see config_rplidar_bridge.h)."
echo "Start the PC server (no ESP32 IP needed):"
echo "  $ROOT/scripts/run_rplidar_wifi.sh"
echo "Optional serial log:"
echo "  pio device monitor -e esp32dev_rplidar_bridge"