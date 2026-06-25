"""Offline handheld SLAM from a recorded bag (no live camera)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("astra_pro_slam")
    rtabmap_params = os.path.join(pkg_share, "config", "rtabmap_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "mapping.rviz")
    default_db = os.path.join(
        os.path.expanduser("~/Documents/orbbec_astro_pro/output/offline_handheld.db")
    )

    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("delete_db", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            LogInfo(msg="Offline handheld SLAM — subscribe to bag topics only."),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_params, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_params,
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=remappings,
                arguments=["-d"],
                condition=IfCondition(LaunchConfiguration("delete_db")),
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_params,
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    },
                ],
                remappings=remappings,
                condition=UnlessCondition(LaunchConfiguration("delete_db")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
