#!/usr/bin/env bash
# One-time setup: allow teleop to read keyboard release events via evdev.
set -euo pipefail
if ! getent group input >/dev/null; then
  echo "input group not found on this system." >&2
  exit 1
fi
if id -nG "${USER}" | tr ' ' '\n' | grep -qx input; then
  echo "User ${USER} is already in group input."
  exit 0
fi
echo "Adding ${USER} to group input (requires sudo)..."
sudo usermod -aG input "${USER}"
cat <<EOF

Done. Activate the new group in THIS shell:
  newgrp input

Or log out and log back in, then run:
  ./scripts/teleop.sh
EOF
