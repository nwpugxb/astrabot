#!/usr/bin/env python3
"""Lightweight person detection, ground projection, Kalman tracking, Nav2 scan.

Detection modes (parameter ``detection_mode``):
  internal  - Ultralytics YOLO on ``color_topic`` (pip install ultralytics)
  external  - ``vision_msgs/Detection2DArray`` on ``detections_topic`` (ros2_yolo etc.)

Outputs:
  /perception/person_markers   - visualization_msgs/MarkerArray (tracks + velocity)
  /perception/person_scan      - sensor_msgs/LaserScan for Nav2 obstacle_layer (optional)
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from indoor_bringup.camera_projection import (
    arc_bins_for_disc,
    bbox_to_ground_xy,
    depth_meters,
    lookup_transform_matrix,
)
from indoor_bringup.laser_utils import empty_ranges, scan_from_bins
from indoor_bringup.person_kalman import PersonTrackerManager, Track

try:
    from vision_msgs.msg import Detection2DArray
except ImportError:  # pragma: no cover
    Detection2DArray = None  # type: ignore

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    YOLO = None  # type: ignore


def _bbox_from_detection(det) -> Optional[Tuple[float, float, float, float]]:
    bb = det.bbox
    cx = bb.center.position.x
    cy = bb.center.position.y
    sx = bb.size_x
    sy = bb.size_y
    if sx <= 0 or sy <= 0:
        return None
    return cx - sx / 2.0, cy - sy / 2.0, cx + sx / 2.0, cy + sy / 2.0


def _is_person_detection(det, class_names: Sequence[str]) -> bool:
    if not det.results:
        return "person" in class_names
    for hyp in det.results:
        name = (hyp.hypothesis.class_id or "").lower()
        if name in class_names:
            return True
    return False


class PersonTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("person_tracker")
        self._declare_params()
        self._load_params()

        self._bridge = CvBridge()
        self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tracker = PersonTrackerManager(
            max_association_dist_m=self._max_assoc_dist,
            max_misses=self._max_misses,
            dt=1.0 / max(1.0, self._detection_rate),
            process_noise=self._process_noise,
            meas_noise=self._meas_noise,
        )

        self._marker_pub = self.create_publisher(MarkerArray, self._markers_topic, 10)
        self._scan_pub = (
            self.create_publisher(LaserScan, self._scan_topic, 10) if self._publish_scan else None
        )

        self._info: Optional[CameraInfo] = None
        self._depth_msg: Optional[Image] = None
        self._color_msg: Optional[Image] = None
        self._last_detect_s = 0.0
        self._yolo = None

        if self._detection_mode == "internal":
            if YOLO is None:
                self.get_logger().error(
                    "detection_mode=internal but ultralytics is not installed. "
                    "Run: pip install ultralytics  OR  set detection_mode:=external"
                )
            else:
                self._yolo = YOLO(self._model_path)
                self.get_logger().info(f"YOLO loaded: {self._model_path}")
            self.create_subscription(
                Image,
                self._color_topic,
                self._on_color,
                qos_profile_sensor_data,
            )
        elif self._detection_mode == "external":
            if Detection2DArray is None:
                self.get_logger().error(
                    "vision_msgs not available; install ros-humble-vision-msgs"
                )
            else:
                self.create_subscription(
                    Detection2DArray,
                    self._detections_topic,
                    self._on_detections,
                    10,
                )
        else:
            self.get_logger().error(f"Unknown detection_mode: {self._detection_mode}")

        self.create_subscription(
            Image,
            self._depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self._camera_info_topic,
            self._on_info,
            qos_profile_sensor_data,
        )

        period = 1.0 / max(1.0, self._publish_rate)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Person tracker mode={self._detection_mode} frame={self._target_frame} "
            f"scan={'on' if self._publish_scan else 'off'}"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("detection_mode", "internal")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter(
            "person_class_names",
            ["person"],
        )
        self.declare_parameter("target_frame", "base_footprint")
        self.declare_parameter("markers_topic", "/perception/person_markers")
        self.declare_parameter("scan_topic", "/perception/person_scan")
        self.declare_parameter("publish_scan", True)
        self.declare_parameter("foot_strip_ratio", 0.15)
        self.declare_parameter("min_person_radius_m", 0.25)
        self.declare_parameter("predict_horizon_s", 0.5)
        self.declare_parameter("max_association_dist_m", 1.5)
        self.declare_parameter("max_misses", 8)
        self.declare_parameter("detection_rate_hz", 5.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("process_noise", 0.8)
        self.declare_parameter("measurement_noise", 0.2)
        self.declare_parameter("scan_num_bins", 720)
        self.declare_parameter("scan_angle_min", -3.14159)
        self.declare_parameter("scan_angle_max", 3.14159)
        self.declare_parameter("scan_range_min", 0.08)
        self.declare_parameter("scan_range_max", 4.0)

    def _load_params(self) -> None:
        g = self.get_parameter
        self._color_topic = str(g("color_topic").value)
        self._depth_topic = str(g("depth_topic").value)
        self._camera_info_topic = str(g("camera_info_topic").value)
        self._detections_topic = str(g("detections_topic").value)
        self._detection_mode = str(g("detection_mode").value).lower()
        self._model_path = str(g("model_path").value)
        self._conf_thresh = float(g("confidence_threshold").value)
        names = g("person_class_names").value
        self._person_classes = {str(n).lower() for n in names}
        self._target_frame = str(g("target_frame").value)
        self._markers_topic = str(g("markers_topic").value)
        self._scan_topic = str(g("scan_topic").value)
        self._publish_scan = bool(g("publish_scan").value)
        self._foot_strip = float(g("foot_strip_ratio").value)
        self._min_radius = float(g("min_person_radius_m").value)
        self._predict_horizon = float(g("predict_horizon_s").value)
        self._max_assoc_dist = float(g("max_association_dist_m").value)
        self._max_misses = int(g("max_misses").value)
        self._detection_rate = float(g("detection_rate_hz").value)
        self._publish_rate = float(g("publish_rate_hz").value)
        self._max_depth = float(g("max_depth_m").value)
        self._process_noise = float(g("process_noise").value)
        self._meas_noise = float(g("measurement_noise").value)
        self._num_bins = int(g("scan_num_bins").value)
        self._angle_min = float(g("scan_angle_min").value)
        self._angle_max = float(g("scan_angle_max").value)
        self._range_min = float(g("scan_range_min").value)
        self._range_max = float(g("scan_range_max").value)

    def _on_info(self, msg: CameraInfo) -> None:
        self._info = msg

    def _on_depth(self, msg: Image) -> None:
        self._depth_msg = msg

    def _on_color(self, msg: Image) -> None:
        self._color_msg = msg
        now = time.monotonic()
        if now - self._last_detect_s < 1.0 / max(1.0, self._detection_rate):
            return
        if self._yolo is None or self._depth_msg is None or self._info is None:
            return
        self._last_detect_s = now
        bboxes = self._detect_yolo(msg)
        self._process_bboxes(bboxes, Time.from_msg(msg.header.stamp))

    def _on_detections(self, msg: Detection2DArray) -> None:
        if self._depth_msg is None or self._info is None:
            return
        bboxes: List[Tuple[float, float, float, float]] = []
        for det in msg.detections:
            if not _is_person_detection(det, self._person_classes):
                continue
            bb = _bbox_from_detection(det)
            if bb is not None:
                bboxes.append(bb)
        stamp = Time.from_msg(msg.header.stamp)
        self._process_bboxes(bboxes, stamp)

    def _detect_yolo(self, color_msg: Image) -> List[Tuple[float, float, float, float]]:
        assert self._yolo is not None
        cv_img = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        results = self._yolo.predict(
            cv_img,
            conf=self._conf_thresh,
            classes=[0],
            verbose=False,
        )
        bboxes: List[Tuple[float, float, float, float]] = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                bboxes.append((float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])))
        return bboxes

    def _process_bboxes(
        self,
        bboxes: Sequence[Tuple[float, float, float, float]],
        stamp: Time,
    ) -> None:
        if self._depth_msg is None or self._info is None:
            return
        depth = depth_meters(self._bridge, self._depth_msg)
        cam_frame = self._depth_msg.header.frame_id or self._info.header.frame_id
        if not cam_frame:
            return
        tf_mat = lookup_transform_matrix(
            self._tf_buffer, self._target_frame, cam_frame, stamp, self.get_logger()
        )
        if tf_mat is None:
            return

        measurements: List[Tuple[float, float, float]] = []
        for x0, y0, x1, y1 in bboxes:
            foot = bbox_to_ground_xy(
                depth,
                self._info,
                tf_mat,
                x0,
                y0,
                x1,
                y1,
                self._foot_strip,
                self._max_depth,
            )
            if foot is None:
                continue
            x, y, radius = foot
            measurements.append((x, y, max(self._min_radius, radius)))

        stamp_s = stamp.nanoseconds * 1e-9
        self._tracker.update(measurements, stamp_s)
        self._publish_outputs(stamp)

    def _on_timer(self) -> None:
        stamp = self.get_clock().now()
        if self._tracker.tracks:
            for track in self._tracker.tracks.values():
                track.kf.predict()
        self._publish_outputs(stamp)

    def _publish_outputs(self, stamp: Time) -> None:
        self._publish_markers(stamp)
        if self._publish_scan and self._scan_pub is not None:
            self._publish_person_scan(stamp)

    def _publish_markers(self, stamp: Time) -> None:
        arr = MarkerArray()
        header = Header(stamp=stamp.to_msg(), frame_id=self._target_frame)
        for track in self._tracker.tracks.values():
            arr.markers.extend(self._markers_for_track(track, header))
        delete = Marker()
        delete.action = Marker.DELETEALL
        arr.markers.insert(0, delete)
        self._marker_pub.publish(arr)

    def _markers_for_track(self, track: Track, header: Header) -> List[Marker]:
        tid = track.track_id
        x, y = track.kf.position
        vx, vy = track.kf.velocity
        px, py = track.kf.predict_horizon(self._predict_horizon)
        speed = math.hypot(vx, vy)

        cyl = Marker()
        cyl.header = header
        cyl.ns = "person_tracks"
        cyl.id = tid
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = x
        cyl.pose.position.y = y
        cyl.pose.position.z = 0.5
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = max(self._min_radius * 2.0, 0.4)
        cyl.scale.y = max(self._min_radius * 2.0, 0.4)
        cyl.scale.z = 1.0
        cyl.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.45)

        pred = Marker()
        pred.header = header
        pred.ns = "person_predicted"
        pred.id = tid
        pred.type = Marker.CYLINDER
        pred.action = Marker.ADD
        pred.pose.position.x = px
        pred.pose.position.y = py
        pred.pose.position.z = 0.05
        pred.pose.orientation.w = 1.0
        pred.scale.x = cyl.scale.x * 1.1
        pred.scale.y = cyl.scale.y * 1.1
        pred.scale.z = 0.08
        pred.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.35)

        text = Marker()
        text.header = header
        text.ns = "person_labels"
        text.id = tid
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 1.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.18
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        text.text = f"id{tid} v={speed:.2f}m/s"

        markers = [cyl, pred, text]
        if speed > 0.05:
            arrow = Marker()
            arrow.header = header
            arrow.ns = "person_velocity"
            arrow.id = tid
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                self._mk_point(x, y, 0.3),
                self._mk_point(x + vx * 0.5, y + vy * 0.5, 0.3),
            ]
            arrow.scale.shaft_diameter = 0.05
            arrow.scale.head_diameter = 0.12
            arrow.scale.head_length = 0.12
            arrow.color = ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.9)
            markers.append(arrow)
        return markers

    @staticmethod
    def _mk_point(x: float, y: float, z: float):
        from geometry_msgs.msg import Point

        p = Point()
        p.x, p.y, p.z = x, y, z
        return p

    def _publish_person_scan(self, stamp: Time) -> None:
        bins = empty_ranges(self._num_bins)
        for track in self._tracker.tracks.values():
            px, py = track.kf.predict_horizon(self._predict_horizon)
            radius = max(self._min_radius, 0.5 * max(track.kf.P[0, 0], track.kf.P[1, 1]) ** 0.5)
            for idx, r in arc_bins_for_disc(
                px, py, radius, self._angle_min, self._angle_max, self._num_bins
            ):
                if r < bins[idx]:
                    bins[idx] = r
        scan = scan_from_bins(
            stamp.to_msg(),
            self._target_frame,
            self._angle_min,
            self._angle_max,
            self._num_bins,
            bins,
            self._range_min,
            self._range_max,
        )
        self._scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonTrackerNode()
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
