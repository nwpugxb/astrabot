"""Offline mobile SLAM from a recorded bag (no live camera / Arduino)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    default_rtabmap_params = os.path.join(mobile_share, "config", "rtabmap_mobile_offline.yaml")
    vo_params = os.path.join(mobile_share, "config", "rgbd_odometry_mobile.yaml")
    rviz_config = os.path.join(mobile_share, "rviz", "mobile_mapping.rviz")
    default_db = os.path.join(
        os.path.expanduser("~/Documents/orbbec_astro_pro/output/offline_mobile.db")
    )

    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/odom"),
        ("scan_cloud", "/wall_plane_cloud"),
    ]
    vo_remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/rgbd_odom"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    rtabmap_params = LaunchConfiguration("rtabmap_config")
    use_rgbd_odometry = LaunchConfiguration("use_rgbd_odometry")
    use_wall_plane_cloud = LaunchConfiguration("use_wall_plane_cloud")

    return LaunchDescription(
        [
            DeclareLaunchArgument("delete_db", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("rtabmap_config", default_value=default_rtabmap_params),
            DeclareLaunchArgument("use_rgbd_odometry", default_value="false"),
            DeclareLaunchArgument("use_wall_plane_cloud", default_value="false"),
            LogInfo(msg=["Offline mobile SLAM — config: ", rtabmap_params]),
            Node(
                package="mobile_base",
                executable="wall_plane_cloud_node",
                name="wall_plane_cloud",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("depth/image_raw", "/camera/depth/image_raw"),
                    ("depth/camera_info", "/camera/color/camera_info"),
                ],
                condition=IfCondition(use_wall_plane_cloud),
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=[vo_params, {"use_sim_time": use_sim_time}],
                remappings=vo_remappings,
                condition=IfCondition(use_rgbd_odometry),
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_params,
                    {
                        "database_path": LaunchConfiguration("database_path"),
                        "use_sim_time": use_sim_time,
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
                        "use_sim_time": use_sim_time,
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
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
