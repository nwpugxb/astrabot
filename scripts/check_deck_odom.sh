#!/usr/bin/env bash
# Diagnose ESP32 micro-ROS /odom (vs lidar TCP which is independent).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash" 2>/dev/null || true

echo "==> Agent UDP :8888"
if ss -H -lunu 2>/dev/null | grep -q ':8888'; then
  echo "  listening OK"
else
  echo "  NOT listening — start ./scripts/run_deck_wifi_lidar.sh"
fi

echo ""
echo "==> ROS topics"
ros2 topic list 2>/dev/null | grep -E 'odom|imu|cmd_vel|scan' || echo "  (none)"

echo ""
echo "==> /odom graph"
ros2 topic info /odom -v 2>&1 | sed -n '1,40p' || true

echo ""
echo "==> Count /odom and /odom_pose for 5s..."
python3 - <<'PY'
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

qos = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

class H(Node):
    def __init__(self):
        super().__init__('odom_hz_check')
        self.n_odom = 0
        self.n_pose = 0
        self.create_subscription(Odometry, '/odom', lambda _m: setattr(self, 'n_odom', self.n_odom + 1), qos)
        self.create_subscription(PoseStamped, '/odom_pose', lambda _m: setattr(self, 'n_pose', self.n_pose + 1), qos)

rclpy.init()
node = H()
t0 = time.monotonic()
while time.monotonic() - t0 < 5.0:
    rclpy.spin_once(node, timeout_sec=0.05)
elapsed = max(time.monotonic() - t0, 1e-6)
print(f"  /odom:      {node.n_odom} msgs ({node.n_odom/elapsed:.1f} Hz)")
print(f"  /odom_pose: {node.n_pose} msgs ({node.n_pose/elapsed:.1f} Hz)")
if node.n_odom == 0 and node.n_pose == 0:
    print("  → No pose from ESP32. Power-cycle board; reflash if needed.")
elif node.n_odom == 0 and node.n_pose > 0:
    print("  → /odom_pose OK (use this for SLAM). Full /odom still blocked by XRCE size;")
    print("    flash with microros_deck.meta (larger MTU) to restore /odom.")
elif node.n_odom > 0:
    print("  → /odom flowing — OK.")
node.destroy_node()
rclpy.shutdown()
PY

echo ""
echo "==> /imu/data_raw 3s..."
python3 - <<'PY'
import time, rclpy
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
class H(Node):
    def __init__(self):
        super().__init__('imu_hz'); self.n=0
        self.create_subscription(Imu, '/imu/data_raw', lambda _m: setattr(self,'n',self.n+1), qos)
rclpy.init(); n=H(); t0=time.monotonic()
while time.monotonic()-t0<3: rclpy.spin_once(n, timeout_sec=0.05)
print(f"  /imu/data_raw: {n.n} msgs in 3s")
n.destroy_node(); rclpy.shutdown()
PY

echo ""
echo "Note: /imu working + /odom silent ⇒ Odometry msg too big for XRCE (common)."
echo "      Fix: flash new firmware (/odom_pose + larger MTU meta)."
