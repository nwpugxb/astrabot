#!/usr/bin/env python3
"""Quick depth-quality check for the Astra Pro before mapping.

Captures a few frames, reports how much usable depth the current scene gives,
and saves a color|depth preview to output/depth_check.png.

A good mapping scene should show >25% usable depth. The structured-light
sensor fails on sunlit surfaces, windows, glass and very dark/black objects,
so aim at a textured indoor scene with objects roughly 0.5-4 m away.
"""
from __future__ import annotations

import os
import numpy as np
import cv2
from astra_raw import AstraIRCamera
from astra_raw.ir import ir_to_depth_mm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MIN_M, MAX_M = 0.4, 4.0


def main() -> int:
    cam = AstraIRCamera(color_index=2)
    cam.open()
    for _ in range(5):
        cam.read_ir(timeout=5)
        cam.read_color()

    fracs = []
    color = depth = None
    for _ in range(10):
        ir = cam.read_ir(timeout=5)
        color = cam.read_color()
        if ir is None:
            continue
        depth = ir_to_depth_mm(ir) / 1000.0
        usable = (depth >= MIN_M) & (depth <= MAX_M)
        fracs.append(100.0 * usable.sum() / depth.size)
    cam.close()

    if not fracs or depth is None:
        print("No frames captured. Check the camera connection.")
        return 1

    pct = float(np.median(fracs))
    print(f"Usable depth ({MIN_M}-{MAX_M} m): {pct:.0f}% of the image")
    if pct >= 25:
        print("GOOD scene for mapping.")
    elif pct >= 10:
        print("MARGINAL - add texture / move closer / avoid bright light.")
    else:
        print("POOR - too much sunlight/glass/black or too far. Re-aim the camera.")

    usable = (depth >= MIN_M) & (depth <= MAX_M)
    norm = np.zeros_like(depth)
    if usable.any():
        norm[usable] = np.clip((depth[usable] - MIN_M) / (MAX_M - MIN_M), 0, 1)
    dcol = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    dcol[~usable] = 0
    out_path = os.path.join(ROOT, "output", "depth_check.png")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    cv2.imwrite(out_path, np.hstack([color, dcol]) if color is not None else dcol)
    print(f"Preview (color | depth) saved to: {out_path}")
    try:
        import subprocess
        subprocess.Popen(["xdg-open", out_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
