#!/usr/bin/env python3
"""Deck ESP32 WiFi: lidar TCP relay + sllidar /scan+/cloud + RViz.

Requires separately (or via scripts/run_deck_wifi_lidar.sh):
  micro-ROS UDP agent on :8888  — ESP32 /cmd_vel /odom /imu
ESP32 also TCP-connects to :20108 for RPLIDAR passthrough.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    scan_mode = LaunchConfiguration("scan_mode")
    frame_id = LaunchConfiguration("frame_id")

    rviz_config = os.path.join(
        get_package_share_directory("indoor_bringup"),
        "rviz",
        "rplidar_wifi.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("scan_mode", default_value="Sensitivity"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("device_port", default_value="20108"),
        DeclareLaunchArgument("sllidar_port", default_value="20109"),

        Node(
            package="indoor_bringup",
            executable="rplidar_tcp_relay_node",
            name="rplidar_tcp_relay",
            parameters=[{
                "device_port": LaunchConfiguration("device_port"),
                "sllidar_port": LaunchConfiguration("sllidar_port"),
                "bind_address": "0.0.0.0",
            }],
            output="screen",
        ),

        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            parameters=[{
                "channel_type": "tcp",
                "tcp_ip": "127.0.0.1",
                "tcp_port": LaunchConfiguration("sllidar_port"),
                "frame_id": frame_id,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": scan_mode,
            }],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package="indoor_bringup",
            executable="scan_to_cloud_node",
            name="scan_to_cloud",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
