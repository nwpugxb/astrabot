#!/usr/bin/env python3
"""Publish fixed CameraInfo for Gazebo RGB-D (matches rgbd_camera in indoor_robot.gazebo.xacro)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class SimCameraInfoNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_camera_info")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("horizontal_fov", 1.0471975512)
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")

        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fov = float(self.get_parameter("horizontal_fov").value)
        fx = w / (2.0 * __import__("math").tan(fov / 2.0))
        fy = fx
        cx = w / 2.0
        cy = h / 2.0

        def make_info(frame: str) -> CameraInfo:
            msg = CameraInfo()
            msg.width = w
            msg.height = h
            msg.distortion_model = "plumb_bob"
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            msg.header.frame_id = frame
            return msg

        self._color_info = make_info(str(self.get_parameter("frame_id").value))
        self._depth_info = make_info(str(self.get_parameter("depth_frame_id").value))
        self._color_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("color_info_topic").value), 10
        )
        self._depth_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("depth_info_topic").value), 10
        )
        self.create_timer(1.0, self._publish)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._color_info.header.stamp = stamp
        self._depth_info.header.stamp = stamp
        self._color_pub.publish(self._color_info)
        self._depth_pub.publish(self._depth_info)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimCameraInfoNode()
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
