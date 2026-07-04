"""Person detection + Kalman tracking (optional Nav2 person_scan layer).

Examples:
  # Standalone (RViz markers; needs Astra + TF)
  ros2 launch indoor_bringup person_tracker.launch.py

  # External detector (ros2_yolo publishes vision_msgs on /perception/detections)
  ros2 launch indoor_bringup person_tracker.launch.py detection_mode:=external

  # Enabled from navigation stack
  ros2 launch indoor_bringup navigation.launch.py use_nav_person:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    cfg = os.path.join(share, "config", "person_tracker.yaml")

    detection_mode = LaunchConfiguration("detection_mode")
    publish_scan = LaunchConfiguration("publish_scan")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "detection_mode",
                default_value="internal",
                description="internal (Ultralytics YOLO) or external (vision_msgs).",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="yolov8n.pt",
                description="YOLO weights when detection_mode:=internal.",
            ),
            DeclareLaunchArgument(
                "publish_scan",
                default_value="true",
                description="Publish /perception/person_scan for Nav2 obstacle_layer.",
            ),
            DeclareLaunchArgument(
                "confidence_threshold",
                default_value="0.45",
            ),
            LogInfo(
                msg=[
                    "Person tracker: mode=", detection_mode,
                    " publish_scan=", publish_scan,
                ]
            ),
            Node(
                package="indoor_bringup",
                executable="person_tracker_node",
                name="person_tracker",
                output="screen",
                parameters=[
                    cfg,
                    {
                        "detection_mode": detection_mode,
                        "model_path": LaunchConfiguration("model_path"),
                        "publish_scan": publish_scan,
                        "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                    },
                ],
            ),
        ]
    )
