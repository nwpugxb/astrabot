#!/usr/bin/env bash
# One-click: export the latest map (if needed) and open the point cloud viewer.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PY="$ROOT/venv/bin/python3"
OUT_PLY="$ROOT/output/map_cloud.ply"

# Pick the newest database we can find.
DB=""
for candidate in \
  "$ROOT"/output/offline_*.db \
  "$ROOT/output/mobile_map.db" \
  "$ROOT/output/handheld_map.db" \
  "$HOME/.ros/rtabmap.db"
do
  # shellcheck disable=SC2086
  for f in $candidate; do
    if [[ -f "$f" ]]; then
      if [[ -z "$DB" || "$f" -nt "$DB" ]]; then
        DB="$f"
      fi
    fi
  done
done

# (Re)export when the database is newer than the exported cloud.
if [[ -n "$DB" ]]; then
  if [[ ! -f "$OUT_PLY" || "$DB" -nt "$OUT_PLY" ]]; then
    echo "Exporting point cloud from: $DB"
    "$ROOT/scripts/export_map.sh" "$DB" "$OUT_PLY"
  fi
fi

echo "Opening point cloud viewer..."
# Open3D's GLFW/GLEW viewer fails under native Wayland; force X11 (XWayland).
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11
exec "$VENV_PY" "$ROOT/scripts/view_cloud.py" "$@"
