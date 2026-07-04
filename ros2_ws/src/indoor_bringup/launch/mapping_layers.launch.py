"""Layered mapping scan pipeline: optional depth / ToF layers + fusion.

Each layer can be toggled independently for debugging in RViz:
  /scan                      - raw A1
  /mapping/layers/lidar_scan - A1 resampled (mirror)
  /mapping/layers/depth_scan - Astra low-obstacle band
  /mapping/layers/tof_scan   - VL53L1X merged
  /scan_fused                - min-range fusion (feeds slam_toolbox when use_fusion:=true)

Examples:
  # All layers + fusion (default)
  ros2 launch indoor_bringup mapping_layers.launch.py

  # A1 only — disable depth/tof/fusion; pair with mapping.launch.py use_fusion:=false
  ros2 launch indoor_bringup mapping_layers.launch.py \\
    use_layer_depth:=false use_layer_tof:=false use_fusion:=false

  # Depth layer only (RViz: /mapping/layers/depth_scan; disable fusion & tof)
  ros2 launch indoor_bringup mapping_layers.launch.py \\
    use_layer_tof:=false use_fusion:=false use_layer_depth:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    layer_cfg = os.path.join(share, "config", "mapping_layers.yaml")

    use_layer_depth = LaunchConfiguration("use_layer_depth")
    use_layer_tof = LaunchConfiguration("use_layer_tof")
    use_fusion = LaunchConfiguration("use_fusion")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_layer_depth",
                default_value="true",
                description="Astra depth -> /mapping/layers/depth_scan.",
            ),
            DeclareLaunchArgument(
                "use_layer_tof",
                default_value="true",
                description="VL53L1X -> /mapping/layers/tof_scan (needs ESP32 /tof_*).",
            ),
            DeclareLaunchArgument(
                "use_fusion",
                default_value="true",
                description="Run scan_fusion_node -> /scan_fused.",
            ),
            LogInfo(
                msg=[
                    "Mapping layers: depth=", use_layer_depth,
                    " tof=", use_layer_tof,
                    " fusion=", use_fusion,
                ]
            ),
            Node(
                package="indoor_bringup",
                executable="depth_scan_layer_node",
                name="depth_scan_layer",
                output="screen",
                parameters=[layer_cfg],
                condition=IfCondition(use_layer_depth),
            ),
            Node(
                package="indoor_bringup",
                executable="tof_scan_layer_node",
                name="tof_scan_layer",
                output="screen",
                parameters=[layer_cfg],
                condition=IfCondition(use_layer_tof),
            ),
            Node(
                package="indoor_bringup",
                executable="scan_fusion_node",
                name="scan_fusion",
                output="screen",
                parameters=[layer_cfg],
                condition=IfCondition(use_fusion),
            ),
        ]
    )
