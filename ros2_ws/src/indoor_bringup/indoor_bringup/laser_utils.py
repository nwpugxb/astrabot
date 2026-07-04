"""Shared LaserScan binning helpers for mapping layer fusion."""

from __future__ import annotations

import math
from typing import Iterable, Optional

import numpy as np
from sensor_msgs.msg import LaserScan


def empty_ranges(num_bins: int, invalid: float = float("inf")) -> np.ndarray:
    return np.full(num_bins, invalid, dtype=np.float64)


def bin_points(
    angles: Iterable[float],
    ranges: Iterable[float],
    angle_min: float,
    angle_max: float,
    num_bins: int,
    invalid: float = float("inf"),
) -> np.ndarray:
    """Per-bin minimum range (closest hit). Angles in radians, base_link frame (+X forward)."""
    out = empty_ranges(num_bins, invalid)
    if num_bins <= 0:
        return out
    inc = (angle_max - angle_min) / float(num_bins)
    if inc <= 0:
        return out
    for ang, dist in zip(angles, ranges):
        if not math.isfinite(dist) or dist <= 0:
            continue
        if ang < angle_min or ang > angle_max:
            continue
        idx = int((ang - angle_min) / inc)
        idx = max(0, min(num_bins - 1, idx))
        if dist < out[idx]:
            out[idx] = dist
    return out


def merge_scan_bins(
    layers: list[np.ndarray],
    invalid: float = float("inf"),
) -> np.ndarray:
    """Merge layers by per-bin minimum range."""
    if not layers:
        return empty_ranges(1, invalid)
    out = layers[0].copy()
    for layer in layers[1:]:
        if len(layer) != len(out):
            continue
        valid = np.isfinite(layer) & (layer > 0)
        if not np.any(valid):
            continue
        mask = valid & ((~np.isfinite(out)) | (layer < out))
        out[mask] = layer[mask]
    return out


def scan_from_bins(
    stamp,
    frame_id: str,
    angle_min: float,
    angle_max: float,
    num_bins: int,
    ranges: np.ndarray,
    range_min: float = 0.05,
    range_max: float = 12.0,
    scan_time: float = 0.0,
) -> LaserScan:
    msg = LaserScan()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = (angle_max - angle_min) / float(num_bins) if num_bins else 0.0
    msg.time_increment = 0.0
    msg.scan_time = scan_time
    msg.range_min = range_min
    msg.range_max = range_max
    msg.ranges = [float(r) if math.isfinite(r) and r > 0 else float("inf") for r in ranges]
    return msg


def scan_to_bins(scan: LaserScan, num_bins: int, invalid: float = float("inf")) -> np.ndarray:
    """Resample an incoming LaserScan into fixed bins (min range per bin)."""
    out = empty_ranges(num_bins, invalid)
    if not scan.ranges or num_bins <= 0:
        return out
    angle_min = scan.angle_min
    inc_out = (scan.angle_max - scan.angle_min) / float(num_bins)
    if inc_out <= 0:
        return out
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        ang = scan.angle_min + i * scan.angle_increment
        idx = int((ang - angle_min) / inc_out)
        idx = max(0, min(num_bins - 1, idx))
        if r < out[idx]:
            out[idx] = r
    return out
