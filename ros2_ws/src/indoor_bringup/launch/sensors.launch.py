"""Sensors: RPLIDAR A1 (/scan) + Orbbec Astra Pro (color/depth).

A1 driver: this uses sllidar_ros2's `sllidar_node`. Install it first (see README):
  - source build: github.com/Slamtec/sllidar_ros2
  - or apt alternative: ros-humble-rplidar-ros (node name differs: rplidar_node).
Astra Pro driver reused from orbbec_ws (astra_camera), publish_tf disabled because
the URDF already provides camera_*_frame / *_optical_frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory("astra_camera")

    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baud = LaunchConfiguration("lidar_baudrate")
    enable_camera = LaunchConfiguration("enable_camera")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            # RPLIDAR A1 = 115200. (A2=256000, S2/S2L=1000000.)
            DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            Node(
                package="sllidar_ros2",
                executable="sllidar_node",
                name="sllidar",
                output="screen",
                parameters=[
                    {
                        "channel_type": "serial",
                        "serial_port": lidar_port,
                        "serial_baudrate": lidar_baud,
                        "frame_id": "laser",
                        "inverted": False,
                        "angle_compensate": True,
                        "scan_mode": "Standard",
                    }
                ],
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(astra_share, "launch", "astra_pro.launch.xml")
                ),
                launch_arguments={
                    "depth_registration": "true",
                    "color_depth_synchronization": "true",
                    "enable_point_cloud": "false",
                    "enable_colored_point_cloud": "false",
                    "enable_d2c_viewer": "false",
                    "oni_log_level": "warning",
                    "oni_log_to_console": "false",
                    "publish_tf": "false",
                }.items(),
                condition=IfCondition(enable_camera),
            ),
        ]
    )
