#!/usr/bin/env python3
"""ROS2 driver for Orbbec Astra Pro RGB-D camera."""

from __future__ import annotations

import math
import os
import subprocess
import threading
from typing import Optional

# Force V4L2 before OpenCV loads the Orbbec obsensor backend.
os.environ.setdefault("OPENCV_VIDEOIO_PRIORITY_LIST", "V4L2,FFMPEG,GSTREAMER")

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


def _autodetect_astra_device() -> Optional[str]:
    """Return /dev/videoN for the Astra Pro color camera."""
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.STDOUT)
    except (subprocess.CalledProcessError, FileNotFoundError):
        return None

    lines = out.splitlines()
    for i, line in enumerate(lines):
        lower = line.lower()
        if "astra" in lower or "orbbec" in lower:
            for follow in lines[i + 1 : i + 5]:
                dev = follow.strip()
                if dev.startswith("/dev/video"):
                    return dev
    return None


def _make_camera_info(
    width: int,
    height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    frame_id: str,
) -> CameraInfo:
    info = CameraInfo()
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.header.frame_id = frame_id
    return info


def _optical_rotation_quaternion() -> tuple[float, float, float, float]:
    half = -math.pi / 4.0
    return (math.sin(half), -math.sin(half), math.sin(half), math.cos(half))


class AstraProCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("astra_pro_camera")

        self.declare_parameter("color_index", 2)
        self.declare_parameter("color_device", "auto")
        self.declare_parameter("pair_rate_hz", 9.0)
        self.declare_parameter("preview_rate_hz", 15.0)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_optical_frame", "camera_color_optical_frame")
        # Color intrinsics (HFOV 63.1 deg -> fx ~ 521 for 640px). RTAB-Map uses these.
        self.declare_parameter("fx", 521.0)
        self.declare_parameter("fy", 521.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        # Depth-sensor intrinsics (HFOV 58.4 deg -> fx ~ 570; matches astra_raw DISP_K).
        self.declare_parameter("depth_fx", 570.0)
        self.declare_parameter("depth_fy", 570.0)
        self.declare_parameter("depth_cx", 320.0)
        self.declare_parameter("depth_cy", 240.0)
        # Depth conditioning / registration.
        self.declare_parameter("min_depth_m", 0.35)
        self.declare_parameter("max_depth_m", 6.0)
        self.declare_parameter("register_depth", True)
        self.declare_parameter("depth_color_baseline_m", 0.025)

        self._frame_id = self.get_parameter("frame_id").value
        self._color_optical = self.get_parameter("color_optical_frame").value
        self._depth_optical = self.get_parameter("depth_optical_frame").value
        fx = float(self.get_parameter("fx").value)
        fy = float(self.get_parameter("fy").value)
        cx = float(self.get_parameter("cx").value)
        cy = float(self.get_parameter("cy").value)
        self._dfx = float(self.get_parameter("depth_fx").value)
        self._dfy = float(self.get_parameter("depth_fy").value)
        self._dcx = float(self.get_parameter("depth_cx").value)
        self._dcy = float(self.get_parameter("depth_cy").value)
        self._min_depth = float(self.get_parameter("min_depth_m").value)
        self._max_depth = float(self.get_parameter("max_depth_m").value)
        self._register = bool(self.get_parameter("register_depth").value)
        self._baseline = float(self.get_parameter("depth_color_baseline_m").value)

        # Color intrinsics for both topics: after registration depth lives in the
        # color optical frame, so RGB and depth share the color intrinsics.
        self._rgb_fx, self._rgb_fy, self._rgb_cx, self._rgb_cy = fx, fy, cx, cy
        self._color_info = _make_camera_info(640, 480, fx, fy, cx, cy, self._color_optical)
        self._depth_info = _make_camera_info(640, 480, fx, fy, cx, cy, self._depth_optical)
        # Precompute depth->3D backprojection grids.
        uu, vv = np.meshgrid(np.arange(640, dtype=np.float32), np.arange(480, dtype=np.float32))
        self._bp_x = (uu - self._dcx) / self._dfx
        self._bp_y = (vv - self._dcy) / self._dfy

        self._bridge = CvBridge()
        # Reliable QoS (depth 10) so RViz and RTAB-Map (both default to RELIABLE)
        # can receive the images. Best-effort publishers are silently dropped by
        # reliable subscribers, which was why no image showed up.
        image_qos = 10
        self._pub_color = self.create_publisher(Image, "color/image_raw", image_qos)
        self._pub_depth = self.create_publisher(Image, "depth/image_raw", image_qos)
        self._pub_preview = self.create_publisher(Image, "color/preview", image_qos)
        self._pub_color_info = self.create_publisher(CameraInfo, "color/camera_info", 10)
        self._pub_depth_info = self.create_publisher(CameraInfo, "depth/camera_info", 10)

        self._static_tf = StaticTransformBroadcaster(self)
        self._publish_static_transforms()

        color_device = str(self.get_parameter("color_device").value)
        color_index = int(self.get_parameter("color_index").value)
        opened_device = self._open_color_capture(color_device, color_index)
        self.get_logger().info(f"Color camera ready on {opened_device}, depth=USB")

        from astra_raw import AstraIRCamera

        self._depth_camera = AstraIRCamera(color_index=None)
        self._depth_camera.open()

        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_color: Optional[np.ndarray] = None
        # A temporally-matched (color, depth_m) pair, both captured ~simultaneously.
        self._latest_pair: Optional[tuple[np.ndarray, np.ndarray]] = None
        self._pair_count = 0
        self._pair_miss = 0

        self._color_thread = threading.Thread(target=self._color_capture_loop, daemon=True)
        self._depth_thread = threading.Thread(target=self._depth_capture_loop, daemon=True)
        self._color_thread.start()
        self._depth_thread.start()

        pair_rate = float(self.get_parameter("pair_rate_hz").value)
        preview_rate = float(self.get_parameter("preview_rate_hz").value)
        self.create_timer(1.0 / max(pair_rate, 1.0), self._publish_rgbd_pair)
        self.create_timer(1.0 / max(preview_rate, 1.0), self._publish_preview)
        self.create_timer(5.0, self._log_rates)

    def _open_color_capture(self, color_device: str, color_index: int) -> str:
        candidates: list[str] = []
        if color_device and color_device not in ("auto", ""):
            candidates.append(color_device)
        detected = _autodetect_astra_device()
        if detected:
            candidates.append(detected)
        candidates.append(f"/dev/video{color_index}")

        seen: set[str] = set()
        errors: list[str] = []
        for dev in candidates:
            if dev in seen:
                continue
            seen.add(dev)
            try:
                cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if not cap.isOpened():
                    errors.append(f"{dev}: not opened")
                    continue
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
                ok, frame = cap.read()
                if not ok or frame is None:
                    cap.release()
                    errors.append(f"{dev}: opened but read failed (device busy?)")
                    continue
                self._color_cap = cap
                return dev
            except Exception as exc:
                errors.append(f"{dev}: {exc}")

        hint = (
            "Kill stale camera processes: pkill -f astra_pro_slam/camera_node ; "
            "then unplug/replug the camera."
        )
        raise RuntimeError(
            f"Failed to open Astra color camera. Tried: {list(seen)}. "
            f"Details: {'; '.join(errors)}. {hint}"
        )

    def _read_color_once(self) -> Optional[np.ndarray]:
        ok, frame = self._color_cap.read()
        if not ok or frame is None:
            return None
        if frame.shape[:2] != (480, 640):
            frame = cv2.resize(frame, (640, 480))
        return frame

    def _color_capture_loop(self) -> None:
        while not self._stop.is_set():
            frame = self._read_color_once()
            if frame is not None:
                with self._lock:
                    self._latest_color = frame
            else:
                self._stop.wait(0.02)

    def _depth_capture_loop(self) -> None:
        while not self._stop.is_set():
            depth_mm = self._depth_camera.read_depth_mm(timeout=0.5)
            if depth_mm is None:
                self._stop.wait(0.02)
                continue
            # Snapshot the freshest color frame the moment depth arrives so the
            # RGB-D pair is captured within one color frame (~33 ms) of each other.
            with self._lock:
                color = None if self._latest_color is None else self._latest_color
            if color is None:
                continue
            depth_m = self._condition_depth(depth_mm)
            with self._lock:
                self._latest_pair = (color, depth_m)

    def _condition_depth(self, depth_mm: np.ndarray) -> np.ndarray:
        """Clamp to a sane range and (optionally) register into the color frame."""
        depth_m = depth_mm.astype(np.float32) / 1000.0
        # Disparities near zero produce absurd far depths (tens of metres); drop them.
        invalid = (depth_m < self._min_depth) | (depth_m > self._max_depth) | ~np.isfinite(depth_m)
        depth_m[invalid] = 0.0
        if self._register and self._baseline != 0.0:
            depth_m = self._register_depth_to_color(depth_m)
        return depth_m

    def _register_depth_to_color(self, depth_m: np.ndarray) -> np.ndarray:
        """Warp depth from the depth-sensor frame into the color camera frame.

        The Astra Pro's depth and color are separate sensors offset by a small
        horizontal baseline, so raw depth is misaligned with color by a
        depth-dependent disparity. We back-project, shift by the baseline, and
        reproject with the color intrinsics so each color pixel gets correct depth.
        """
        valid = depth_m > 0.0
        if not valid.any():
            return depth_m
        z = depth_m[valid]
        x = self._bp_x[valid] * z - self._baseline
        y = self._bp_y[valid] * z
        u = np.round(self._rgb_fx * x / z + self._rgb_cx).astype(np.int32)
        v = np.round(self._rgb_fy * y / z + self._rgb_cy).astype(np.int32)
        inb = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
        u, v, z = u[inb], v[inb], z[inb]
        out = np.zeros_like(depth_m)
        # Z-buffer: write far points first so nearer points win on collision.
        order = np.argsort(-z)
        out[v[order], u[order]] = z[order]
        return out

    def _publish_static_transforms(self) -> None:
        qx, qy, qz, qw = _optical_rotation_quaternion()
        color_tf = TransformStamped()
        color_tf.header.stamp = self.get_clock().now().to_msg()
        color_tf.header.frame_id = self._frame_id
        color_tf.child_frame_id = self._color_optical
        color_tf.transform.rotation.x = qx
        color_tf.transform.rotation.y = qy
        color_tf.transform.rotation.z = qz
        color_tf.transform.rotation.w = qw
        self._static_tf.sendTransform([color_tf])

    def _publish_rgbd_pair(self) -> None:
        with self._lock:
            pair = self._latest_pair
        if pair is None:
            self._pair_miss += 1
            return
        color, depth_m = pair

        stamp = self.get_clock().now().to_msg()

        color_msg = self._bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self._color_optical
        self._pub_color.publish(color_msg)

        depth_msg = self._bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self._depth_optical
        self._pub_depth.publish(depth_msg)

        self._color_info.header.stamp = stamp
        self._depth_info.header.stamp = stamp
        self._pub_color_info.publish(self._color_info)
        self._pub_depth_info.publish(self._depth_info)

        self._pair_count += 1

    def _publish_preview(self) -> None:
        with self._lock:
            if self._latest_color is None:
                return
            color = self._latest_color.copy()
        stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(color, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self._color_optical
        self._pub_preview.publish(msg)

    def _log_rates(self) -> None:
        pair_count = self._pair_count
        pair_miss = self._pair_miss
        self._pair_count = 0
        self._pair_miss = 0
        self.get_logger().info(
            f"rgbd pairs={pair_count / 5.0:.1f} Hz, missed={pair_miss / 5.0:.1f}/s"
        )

    def destroy_node(self) -> bool:
        self._stop.set()
        for thread in (getattr(self, "_color_thread", None), getattr(self, "_depth_thread", None)):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
        if hasattr(self, "_depth_camera") and self._depth_camera is not None:
            self._depth_camera.close()
        if hasattr(self, "_color_cap") and self._color_cap is not None:
            self._color_cap.release()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = AstraProCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
