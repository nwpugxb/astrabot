"""Camera depth unprojection and bbox foot-point helpers (Astra Pro)."""

from __future__ import annotations

from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy), 0],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx), 0],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy), 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float64,
    )


def lookup_transform_matrix(
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    source_frame: str,
    stamp: Time,
    logger=None,
) -> Optional[np.ndarray]:
    try:
        tf = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            stamp,
            timeout=rclpy.duration.Duration(seconds=0.15),
        )
    except tf2_ros.TransformException as exc:
        if logger is not None:
            logger.warn(str(exc), throttle_duration_sec=5.0)
        return None
    t = tf.transform.translation
    q = tf.transform.rotation
    mat = quat_to_matrix(q.x, q.y, q.z, q.w)
    mat[0, 3], mat[1, 3], mat[2, 3] = t.x, t.y, t.z
    return mat


def depth_meters(bridge: CvBridge, msg: Image) -> np.ndarray:
    enc = (msg.encoding or "").lower()
    if enc in ("32fc1", "32fc"):
        return bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1").astype(np.float32)
    if enc in ("16uc1", "mono16"):
        raw = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        return raw.astype(np.float32) / 1000.0
    return bridge.imgmsg_to_cv2(msg).astype(np.float32)


def pixel_to_camera(u: float, v: float, depth_m: float, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array(
        [(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m, 1.0],
        dtype=np.float64,
    )


def transform_point(mat: np.ndarray, pt_h: np.ndarray) -> np.ndarray:
    out = mat @ pt_h
    return out[:3]


def sample_depth(depth: np.ndarray, u: int, v: int, max_depth_m: float) -> Optional[float]:
    h, w = depth.shape[:2]
    u = int(max(0, min(w - 1, u)))
    v = int(max(0, min(h - 1, v)))
    z = float(depth[v, u])
    if not np.isfinite(z) or z <= 0.05 or z > max_depth_m:
        return None
    return z


def bbox_foot_samples(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    foot_strip_ratio: float,
) -> Sequence[Tuple[float, float]]:
    """Return (u, v) samples along the bottom edge of a pixel bbox."""
    width = max(1.0, x_max - x_min)
    height = max(1.0, y_max - y_min)
    v_row = y_max - max(1.0, height * foot_strip_ratio * 0.5)
    us = (
        x_min + 0.15 * width,
        0.5 * (x_min + x_max),
        x_max - 0.15 * width,
    )
    return [(u, v_row) for u in us]


def bbox_to_ground_xy(
    depth: np.ndarray,
    info: CameraInfo,
    tf_mat: np.ndarray,
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    foot_strip_ratio: float,
    max_depth_m: float,
    ground_z_max: float = 0.25,
) -> Optional[Tuple[float, float, float]]:
    """Project bbox foot samples to target frame; return (x, y, radius_m)."""
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    pts_xy: list[np.ndarray] = []
    for u, v in bbox_foot_samples(x_min, y_min, x_max, y_max, foot_strip_ratio):
        z = sample_depth(depth, int(u), int(v), max_depth_m)
        if z is None:
            continue
        cam = pixel_to_camera(u, v, z, fx, fy, cx, cy)
        base = transform_point(tf_mat, cam)
        if abs(base[2]) > ground_z_max:
            continue
        pts_xy.append(base[:2])

    if not pts_xy:
        return None

    arr = np.stack(pts_xy, axis=0)
    center = np.median(arr, axis=0)
    spread = float(np.max(np.linalg.norm(arr - center, axis=1))) if len(arr) > 1 else 0.15
    width_m = max(0.25, 2.0 * spread)
    return float(center[0]), float(center[1]), width_m


def arc_bins_for_disc(
    cx: float,
    cy: float,
    radius: float,
    angle_min: float,
    angle_max: float,
    num_bins: int,
) -> Iterable[Tuple[int, float]]:
    """Yield (bin_index, range) for a disc obstacle in the robot plane (+X forward)."""
    if radius <= 0 or num_bins <= 0:
        return
    inc = (angle_max - angle_min) / float(num_bins)
    if inc <= 0:
        return
    for i in range(num_bins):
        ang = angle_min + (i + 0.5) * inc
        dx = np.cos(ang)
        dy = np.sin(ang)
        # Closest range from origin to circle boundary along ray.
        b = 2.0 * (cx * dx + cy * dy)
        c = cx * cx + cy * cy - radius * radius
        disc = b * b - 4.0 * c
        if disc < 0:
            continue
        r = 0.5 * (-b - np.sqrt(disc))
        if r > 0:
            yield i, float(r)
