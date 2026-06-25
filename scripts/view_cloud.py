#!/usr/bin/env python3
"""Open a PLY point cloud in an interactive Open3D window.

Usage:
    view_cloud.py [cloud.ply]

If no path is given, the newest .ply under output/ (or ~/.ros) is used.
Mouse: left-drag rotate, scroll zoom, right-drag pan. Press Q to quit.
"""
from __future__ import annotations

import glob
import os
import sys

import open3d as o3d

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def find_latest_ply() -> str | None:
    candidates: list[str] = []
    for pattern in (
        os.path.join(ROOT, "output", "*.ply"),
        os.path.expanduser("~/.ros/*.ply"),
    ):
        candidates.extend(glob.glob(pattern))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def main() -> int:
    path = sys.argv[1] if len(sys.argv) > 1 else find_latest_ply()
    if not path or not os.path.isfile(path):
        print("No point cloud (.ply) found. Build/export a map first.", file=sys.stderr)
        return 1

    print(f"Loading {path} ...")
    pcd = o3d.io.read_point_cloud(path)
    n = len(pcd.points)
    if n == 0:
        print("Point cloud is empty.", file=sys.stderr)
        return 1
    print(f"{n} points")

    # Light cleanup so the view is less noisy.
    if n > 5000:
        pcd = pcd.voxel_down_sample(voxel_size=0.01)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    o3d.visualization.draw_geometries(
        [pcd],
        window_name=os.path.basename(path),
        width=1280,
        height=800,
        point_show_normal=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
