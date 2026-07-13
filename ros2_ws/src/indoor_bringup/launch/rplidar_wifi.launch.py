#!/usr/bin/env python3
"""Host-as-server RPLIDAR path: ESP32 TCP client → relay → sllidar → /scan+/cloud+RViz.

ESP32 firmware connects to this PC (HOST_IP in config_rplidar_bridge.h).
You do not need the ESP32 IP.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_port = LaunchConfiguration("device_port")
    sllidar_port = LaunchConfiguration("sllidar_port")
    frame_id = LaunchConfiguration("frame_id")
    scan_mode = LaunchConfiguration("scan_mode")

    rviz_config = os.path.join(
        get_package_share_directory("indoor_bringup"),
        "rviz",
        "rplidar_wifi.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "device_port",
            default_value="20108",
            description="TCP port ESP32 connects to on this PC",
        ),
        DeclareLaunchArgument(
            "sllidar_port",
            default_value="20109",
            description="Local TCP port for sllidar_ros2 (127.0.0.1)",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser",
            description="frame_id for LaserScan / PointCloud2",
        ),
        DeclareLaunchArgument(
            "scan_mode",
            default_value="Sensitivity",
            description="RPLIDAR A1 scan mode",
        ),

        Node(
            package="indoor_bringup",
            executable="rplidar_tcp_relay_node",
            name="rplidar_tcp_relay",
            parameters=[{
                "device_port": device_port,
                "sllidar_port": sllidar_port,
                "bind_address": "0.0.0.0",
            }],
            output="screen",
        ),

        # Respawn until relay opens :sllidar_port (after ESP32 connects).
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            parameters=[{
                "channel_type": "tcp",
                "tcp_ip": "127.0.0.1",
                "tcp_port": sllidar_port,
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
        ),
    ])
