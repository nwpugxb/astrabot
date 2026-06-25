"""Record-only: Orbbec Astra Pro RGB-D (no SLAM, no RViz)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_share = get_package_share_directory("astra_camera")

    return LaunchDescription(
        [
            DeclareLaunchArgument("color_fps", default_value="15"),
            DeclareLaunchArgument("depth_fps", default_value="15"),
            LogInfo(msg="Record mode: camera only (RGB + depth + TF). No SLAM."),
            Node(
                package="astra_camera",
                executable="clean_shm_node",
                name="clean_shm",
                output="screen",
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
                    "color_fps": LaunchConfiguration("color_fps"),
                    "depth_fps": LaunchConfiguration("depth_fps"),
                    "oni_log_level": "warning",
                    "oni_log_to_console": "false",
                }.items(),
            ),
        ]
    )
