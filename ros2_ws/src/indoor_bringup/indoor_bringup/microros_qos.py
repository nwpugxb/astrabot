"""QoS profiles for ESP32 micro-ROS topics (match firmware microros_qos_depth1)."""

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# KEEP_LAST depth=1, RELIABLE — matches rclc default reliability on ESP32.
MICROROS_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
