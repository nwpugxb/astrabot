"""Live RGB + colored point cloud preview (no SLAM)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("astra_pro_slam")
    rviz_config = os.path.join(pkg_share, "rviz", "live.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("color_device", default_value="auto"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("image_view", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "camera.launch.py")
                ),
                launch_arguments={
                    "color_device": LaunchConfiguration("color_device"),
                }.items(),
            ),
            Node(
                package="astra_pro_slam",
                executable="rgbd_cloud_node",
                name="rgbd_cloud",
                output="screen",
                namespace="camera",
                parameters=[{"stride": 2, "max_depth_m": 6.0}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_image_view",
                output="screen",
                arguments=["/camera/color/image_raw"],
                condition=IfCondition(LaunchConfiguration("image_view")),
            ),
        ]
    )
