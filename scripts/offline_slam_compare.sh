#!/usr/bin/env bash
# Run all three SLAM tuning profiles on one bag (no RViz), save separate databases.
#
# Usage:
#   ./scripts/offline_slam_compare.sh output/bags/mobile_20260625_194102
#
# Outputs:
#   output/offline_<bag>_s2_visicp.db
#   output/offline_<bag>_s3_vo.db
#   output/offline_<bag>_s2s3.db
#
# Export point clouds after:
#   ./scripts/export_map.sh output/offline_<bag>_s2_visicp.db output/compare_<bag>_s2.ply
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-}"

if [[ -z "$BAG" || ! -d "$BAG" ]]; then
  echo "Usage: $0 <bag_directory>" >&2
  echo "Example: $0 output/bags/mobile_20260625_194102" >&2
  exit 1
fi

STAMP="$(basename "$BAG")"
SLAM="$ROOT/scripts/offline_slam.sh"

run_profile() {
  local profile="$1"
  local label="$2"
  echo ""
  echo "============================================================"
  echo " Profile: $profile ($label)"
  echo "============================================================"
  "$SLAM" "$BAG" --mobile --profile "$profile" --no-rviz
}

echo "Compare SLAM profiles on: $BAG"
echo "Results will be written under: $ROOT/output/offline_${STAMP}_*.db"

run_profile s2 "scheme 2: visual + ICP"
run_profile s3 "scheme 3: rgbd_odometry + graph opt"
run_profile s2s3 "scheme 2+3 combined"

echo ""
echo "Done. Databases:"
for suffix in _s2_visicp _s3_vo _s2s3; do
  db="$ROOT/output/offline_${STAMP}${suffix}.db"
  if [[ -f "$db" ]]; then
    echo "  $db ($(du -h "$db" | cut -f1))"
  else
    echo "  MISSING: $db" >&2
  fi
done
echo ""
echo "Export for comparison:"
echo "  ./scripts/export_map.sh output/offline_${STAMP}_s2_visicp.db output/compare_${STAMP}_s2.ply"
echo "  ./scripts/export_map.sh output/offline_${STAMP}_s3_vo.db output/compare_${STAMP}_s3.ply"
echo "  ./scripts/export_map.sh output/offline_${STAMP}_s2s3.db output/compare_${STAMP}_s2s3.ply"
