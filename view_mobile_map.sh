#!/usr/bin/env bash
# Open a saved mobile SLAM database in RViz (/cloud_map). No camera or robot needed.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB="${1:-$ROOT/output/mobile_map.db}"

if [[ ! -f "$DB" ]]; then
  echo "Map database not found: $DB" >&2
  echo "Build a map first: ./run_mobile_mapping.sh" >&2
  exit 1
fi

if [[ -n "${VIRTUAL_ENV:-}" ]]; then
  PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v "^${VIRTUAL_ENV}/bin$" | paste -sd:)"
  unset VIRTUAL_ENV PYTHONHOME
fi

source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"

echo "Loading map into RViz: $DB ($(du -h "$DB" | cut -f1))"
exec ros2 launch mobile_base view_mobile_map.launch.py "database_path:=$DB" "$@"
