#!/usr/bin/env python3
"""Ground-truth odom from Gazebo model pose (spawn-relative odom frame).

Uses /world/.../dynamic_pose/info (Pose_V -> TFMessage) for drift-free pose.
Wheel odom fallback only if dynamic pose is unavailable.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def _yaw_from_quat(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class SimGtOdomNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_gt_odom")
        self.declare_parameter(
            "dynamic_pose_topic",
            "/world/indoor_sim/dynamic_pose/info",
        )
        self.declare_parameter("model_name", "indoor_robot")
        self.declare_parameter("wheel_odom_topic", "/diff_drive_controller/odom")
        self.declare_parameter("odom_topic", "/sim/gt_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("spawn_x", -2.0)
        self.declare_parameter("spawn_y", -2.0)
        self.declare_parameter("pose_timeout_s", 3.0)

        self._spawn_x = float(self.get_parameter("spawn_x").value)
        self._spawn_y = float(self.get_parameter("spawn_y").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        self._pose_mode = False
        self._have_odom = False
        self._logged_pose = False
        self._last_pose_rx = self.get_clock().now()
        self._warned_fallback = False

        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._prev_yaw: float | None = None
        self._prev_time = None
        self._last_ox = 0.0
        self._last_oy = 0.0
        self._last_q: Quaternion | None = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        pose_topic = str(self.get_parameter("dynamic_pose_topic").value)
        self.create_subscription(TFMessage, pose_topic, self._on_dynamic_pose, 10)
        wheel_topic = str(self.get_parameter("wheel_odom_topic").value)
        self.create_subscription(
            Odometry,
            wheel_topic,
            self._on_wheel_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(1.0, self._check_pose_timeout)
        self.create_timer(0.2, self._seed_tf_until_odom)
        self.create_timer(0.02, self._hold_tf)
        self.get_logger().info(
            f"GT odom (spawn-relative): dynamic_pose={pose_topic} "
            f"model={self._model_name} -> {odom_topic}, "
            f"spawn=({self._spawn_x}, {self._spawn_y})"
        )

    def _check_pose_timeout(self) -> None:
        if self._pose_mode:
            return
        age = (self.get_clock().now() - self._last_pose_rx).nanoseconds * 1e-9
        if age > self._pose_timeout and not self._warned_fallback:
            self._warned_fallback = True
            self.get_logger().warn(
                f"No Gazebo dynamic_pose for {age:.1f}s — wheel odom fallback "
                "(turning will smear scans; check pose bridge)."
            )

    def _publish_odom_tf(
        self, stamp, ox, oy, oz, q: Quaternion, vx, vy, vyaw, *, real: bool = True
    ) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.position.z = oz
        odom.pose.pose.orientation = q
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=vyaw)
        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = ox
        tf_msg.transform.translation.y = oy
        tf_msg.transform.translation.z = oz
        tf_msg.transform.rotation = q
        self._tf_broadcaster.sendTransform(tf_msg)
        if real:
            self._have_odom = True

    def _seed_tf_until_odom(self) -> None:
        if self._have_odom:
            return
        q = Quaternion(w=1.0)
        self._publish_odom_tf(
            self.get_clock().now().to_msg(), 0.0, 0.0, 0.0, q, 0.0, 0.0, 0.0, real=False
        )

    def _on_dynamic_pose(self, msg: TFMessage) -> None:
        model_tf = None
        for tf in msg.transforms:
            if tf.child_frame_id == self._model_name:
                model_tf = tf
                break
        if model_tf is None:
            return

        self._pose_mode = True
        self._last_pose_rx = self.get_clock().now()

        wx = model_tf.transform.translation.x
        wy = model_tf.transform.translation.y
        q = model_tf.transform.rotation
        ox = wx - self._spawn_x
        oy = wy - self._spawn_y
        yaw = _yaw_from_quat(q)

        if not self._logged_pose:
            self._logged_pose = True
            self.get_logger().info(
                f"GT odom active: world=({wx:.3f}, {wy:.3f}) "
                f"-> odom=({ox:.3f}, {oy:.3f})"
            )

        now = self.get_clock().now()
        vx = vy = vyaw = 0.0
        if self._prev_time is not None and self._prev_x is not None:
            dt = (now - self._prev_time).nanoseconds * 1e-9
            if dt > 1e-4:
                vx = (ox - self._prev_x) / dt
                vy = (oy - self._prev_y) / dt
                dyaw = math.atan2(
                    math.sin(yaw - (self._prev_yaw or 0.0)),
                    math.cos(yaw - (self._prev_yaw or 0.0)),
                )
                vyaw = dyaw / dt
        self._prev_x, self._prev_y, self._prev_yaw, self._prev_time = ox, oy, yaw, now
        self._last_ox, self._last_oy, self._last_q = ox, oy, _quat_from_yaw(yaw)

        stamp = now.to_msg()
        self._publish_odom_tf(stamp, ox, oy, 0.0, self._last_q, vx, vy, vyaw)

    def _hold_tf(self) -> None:
        if not self._have_odom or self._last_q is None:
            return
        stamp = self.get_clock().now().to_msg()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = self._last_ox
        tf_msg.transform.translation.y = self._last_oy
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self._last_q
        self._tf_broadcaster.sendTransform(tf_msg)

    def _on_wheel_odom(self, msg: Odometry) -> None:
        if self._pose_mode:
            return
        self._publish_odom_tf(
            msg.header.stamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGtOdomNode()
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
