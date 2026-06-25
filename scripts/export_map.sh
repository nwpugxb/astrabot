#!/usr/bin/env bash
# Export RTAB-Map database to a PLY point cloud.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DB="${1:-}"
OUT="${2:-$ROOT/output/map_cloud.ply}"

if [[ -z "$DB" ]]; then
  for candidate in \
    "$ROOT/output/mobile_map.db" \
    "$ROOT/output/handheld_map.db" \
    "$HOME/.ros/astra_pro_handheld.db" \
    "$HOME/.ros/rtabmap.db"
  do
    if [[ -f "$candidate" ]]; then
      DB="$candidate"
      break
    fi
  done
fi

if [[ -z "$DB" || ! -f "$DB" ]]; then
  echo "No database found. Build a map first, or pass:" >&2
  echo "  $0 /path/to/handheld_map.db" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
OUT_DIR="$(dirname "$OUT")"
OUT_NAME="$(basename "${OUT%.ply}")"
mkdir -p "$OUT_DIR"

# rtabmap-export takes --output (name) and --output_dir, not a path arg.
# It writes "<name>.ply" (sometimes "<name>_cloud.ply") into the output dir.
rtabmap-export --cloud --output "$OUT_NAME" --output_dir "$OUT_DIR" "$DB" || {
  echo "ERROR: rtabmap-export failed for $DB" >&2
  exit 1
}

# Find whichever PLY was just produced and normalize it to "$OUT".
produced="$(ls -t "$OUT_DIR"/${OUT_NAME}*.ply 2>/dev/null | head -1)"
if [[ -n "$produced" && "$produced" != "$OUT" ]]; then
  mv -f "$produced" "$OUT"
fi

if [[ -f "$OUT" ]]; then
  echo "Exported point cloud: $OUT ($(du -h "$OUT" | cut -f1))"
else
  echo "Export finished but PLY not found in $OUT_DIR" >&2
fi
echo "Source database: $DB ($(du -h "$DB" | cut -f1))"
