#!/usr/bin/env bash
# One-click: check whether the scene the camera sees has enough depth to map.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$ROOT/scripts/stop_camera.sh" >/dev/null 2>&1 || true
exec "$ROOT/venv/bin/python3" "$ROOT/scripts/check_depth.py"
