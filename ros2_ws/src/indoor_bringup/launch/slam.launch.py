"""2D SLAM with slam_toolbox (async mapping) consuming a LaserScan topic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    default_params = os.path.join(share, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params_file")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            DeclareLaunchArgument(
                "scan_topic",
                default_value="/scan",
                description="LaserScan input: /scan (A1 only) or /scan_fused (layered).",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params,
                    {"use_sim_time": use_sim_time, "scan_topic": scan_topic},
                ],
            ),
        ]
    )
