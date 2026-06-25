#!/usr/bin/env bash
# One-time setup: allow non-root access to Orbbec Astra Pro USB devices.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$SCRIPT_DIR/99-orbbec-astra.rules" /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
echo "udev rules installed. Unplug and replug the camera if depth/IR capture still fails."
