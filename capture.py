#!/usr/bin/env python3
"""Capture images from Orbbec Astra Pro and save to ./output/."""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

OUTPUT_DIR = Path(__file__).resolve().parent / "output"
COLOR_DEVICE = "/dev/video2"  # Astra Pro HD Camera (2bc5:0501)


def find_color_index() -> int:
    """Map /dev/videoN to OpenCV index."""
    dev = Path(COLOR_DEVICE)
    if dev.exists():
        return int(dev.name.replace("video", ""))
    return 2


def save_color(path: Path) -> bool:
    index = find_color_index()
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        cap = cv2.VideoCapture(COLOR_DEVICE)
    if not cap.isOpened():
        print(f"[color] failed to open {COLOR_DEVICE}", file=sys.stderr)
        return False

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Warm up a few frames for auto exposure.
    for _ in range(10):
        cap.read()
        time.sleep(0.05)

    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        print("[color] failed to read frame", file=sys.stderr)
        return False

    cv2.imwrite(str(path), frame)
    print(f"[color] saved {path} ({frame.shape[1]}x{frame.shape[0]})")
    return True


def save_depth_and_ir(color_index: int, depth_path: Path, ir_path: Path) -> bool:
    try:
        from astra_raw import AstraIRCamera
    except ImportError:
        print("[depth] orbbec-astra-raw not installed", file=sys.stderr)
        return False

    try:
        with AstraIRCamera(color_index=color_index) as cam:
            for _ in range(5):
                cam.read_ir(timeout=1.0)

            depth = cam.read_depth_mm(timeout=5.0)
            ir = cam.read_ir(timeout=5.0)
    except Exception as exc:
        print(f"[depth] skipped ({exc})", file=sys.stderr)
        print("[depth] run: ./setup_udev.sh  (needs sudo once)", file=sys.stderr)
        return False

    saved = False
    if depth is not None:
        depth_vis = np.clip(depth, 0, 4000).astype(np.float32)
        depth_vis = (depth_vis / 4000.0 * 255.0).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        cv2.imwrite(str(depth_path), depth_vis)
        print(f"[depth] saved {depth_path}")
        saved = True

    if ir is not None:
        ir_norm = cv2.normalize(ir, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imwrite(str(ir_path), ir_norm)
        print(f"[ir] saved {ir_path}")
        saved = True

    return saved


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    color_index = find_color_index()

    ok_color = save_color(OUTPUT_DIR / "color.jpg")
    ok_depth = save_depth_and_ir(
        color_index,
        OUTPUT_DIR / "depth.png",
        OUTPUT_DIR / "ir.png",
    )

    if ok_color or ok_depth:
        print(f"Done. Images in {OUTPUT_DIR}")
        return 0

    print("No images captured.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
