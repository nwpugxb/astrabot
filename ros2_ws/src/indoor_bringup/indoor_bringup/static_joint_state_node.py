"""Publish placeholder /joint_states for wheel joints (no apt joint_state_publisher needed)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEEL_JOINTS = ("left_wheel_joint", "right_wheel_joint")


class StaticJointStateNode(Node):
    def __init__(self) -> None:
        super().__init__("static_joint_state")
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0, self._publish)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(WHEEL_JOINTS)
        msg.position = [0.0] * len(WHEEL_JOINTS)
        self._pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaticJointStateNode()
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
