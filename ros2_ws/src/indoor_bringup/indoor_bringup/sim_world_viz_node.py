"""Publish Gazebo sim room geometry as RViz markers (walls + obstacles).

Matches worlds/indoor_sim.sdf. odom origin = robot spawn (must match gazebo.launch spawn_x/y).
"""

from __future__ import annotations

from typing import Sequence

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Must match gazebo.launch.py spawn_x / spawn_y defaults.
SPAWN_X = -2.0
SPAWN_Y = -2.0

_WALLS_WORLD: Sequence[tuple] = (
    ("wall_north", 0.0, 3.95, 1.0, 8.0, 0.10, 2.0, 0.22, 0.28, 0.34, 0.95),
    ("wall_south", 0.0, -3.95, 1.0, 8.0, 0.10, 2.0, 0.22, 0.28, 0.34, 0.95),
    ("wall_east", 3.95, 0.0, 1.0, 0.10, 8.0, 2.0, 0.22, 0.28, 0.34, 0.95),
    ("wall_west", -3.95, 0.0, 1.0, 0.10, 8.0, 2.0, 0.22, 0.28, 0.34, 0.95),
)

_OBSTACLE_WORLD = (
    "obstacle_center",
    0.0,
    0.0,
    0.5,
    2.0,
    2.0,
    1.0,
    0.85,
    0.55,
    0.15,
    0.92,
)

_LABELS_WORLD: Sequence[tuple] = (
    (10, 0.0, 3.2, 1.6, "北墙 (8m)"),
    (11, 0.0, -3.2, 1.6, "南墙"),
    (12, 3.2, 0.0, 1.6, "东墙"),
    (13, -3.2, 0.0, 1.6, "西墙"),
    (14, 0.0, 0.0, 1.35, "中心障碍 2×2m"),
    (15, SPAWN_X, SPAWN_Y, 0.35, "起点 (小车出生)"),
)


class SimWorldVizNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_world_viz")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("topic", "/sim/world_markers")
        self.declare_parameter("publish_hz", 5.0)

        self._frame = str(self.get_parameter("frame_id").value)
        self._robot_frame = str(self.get_parameter("robot_frame").value)
        topic = str(self.get_parameter("topic").value)
        self._pub = self.create_publisher(MarkerArray, topic, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        hz = float(self.get_parameter("publish_hz").value)
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f"Sim world -> {topic} frame={self._frame} "
            f"(spawn=({SPAWN_X}, {SPAWN_Y}), odom=world-spawn)"
        )

    def _world_to_odom(self, wx: float, wy: float, wz: float) -> tuple[float, float, float]:
        return wx - SPAWN_X, wy - SPAWN_Y, wz

    def _box(self, marker_id: int, spec: tuple) -> Marker:
        _, wx, wy, wz, sx, sy, sz, r, g, b, a = spec
        ox, oy, oz = self._world_to_odom(wx, wy, wz)
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sim_world"
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = ox
        m.pose.position.y = oy
        m.pose.position.z = oz
        m.pose.orientation.w = 1.0
        m.scale = Vector3(x=float(sx), y=float(sy), z=float(sz))
        m.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
        m.lifetime.sec = 0
        return m

    def _text(self, marker_id: int, wx: float, wy: float, wz: float, text: str) -> Marker:
        ox, oy, oz = self._world_to_odom(wx, wy, wz)
        m = Marker()
        m.header.frame_id = self._frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "sim_labels"
        m.id = marker_id
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = ox
        m.pose.position.y = oy
        m.pose.position.z = oz
        m.pose.orientation.w = 1.0
        m.scale.z = 0.32
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        m.text = text
        m.lifetime.sec = 0
        return m

    def _robot_markers(self, base_id: int) -> list[Marker]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame, self._robot_frame, rclpy.time.Time()
            )
        except Exception:
            return []

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        markers: list[Marker] = []

        body = Marker()
        body.header.frame_id = self._frame
        body.header.stamp = self.get_clock().now().to_msg()
        body.ns = "sim_robot"
        body.id = base_id
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.12
        body.pose.orientation = q
        body.scale = Vector3(x=0.42, y=0.42, z=0.22)
        body.color = ColorRGBA(r=0.15, g=0.78, b=0.35, a=0.95)
        body.lifetime.sec = 0
        markers.append(body)

        arrow = Marker()
        arrow.header.frame_id = self._frame
        arrow.header.stamp = body.header.stamp
        arrow.ns = "sim_robot"
        arrow.id = base_id + 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.35
        arrow.pose.orientation = q
        arrow.scale = Vector3(x=0.55, y=0.08, z=0.08)
        arrow.color = ColorRGBA(r=0.1, g=0.95, b=0.25, a=1.0)
        arrow.lifetime.sec = 0
        markers.append(arrow)

        label = Marker()
        label.header.frame_id = self._frame
        label.header.stamp = body.header.stamp
        label.ns = "sim_robot"
        label.id = base_id + 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 0.55
        label.pose.orientation.w = 1.0
        label.scale.z = 0.28
        label.color = ColorRGBA(r=0.9, g=1.0, b=0.9, a=1.0)
        label.text = "机器人"
        label.lifetime.sec = 0
        markers.append(label)
        return markers

    def _publish(self) -> None:
        arr = MarkerArray()
        mid = 0
        for spec in _WALLS_WORLD:
            arr.markers.append(self._box(mid, spec))
            mid += 1
        arr.markers.append(self._box(mid, _OBSTACLE_WORLD))
        mid += 1
        for label_id, wx, wy, wz, text in _LABELS_WORLD:
            arr.markers.append(self._text(label_id, wx, wy, wz, text))
        arr.markers.extend(self._robot_markers(100))
        self._pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimWorldVizNode()
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
