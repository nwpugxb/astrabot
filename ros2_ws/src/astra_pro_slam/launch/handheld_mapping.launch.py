"""Handheld RGB-D SLAM with Orbbec Astra Pro (official OpenNI2 driver) + RTAB-Map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("astra_pro_slam")
    astra_share = get_package_share_directory("astra_camera")
    rtabmap_params = os.path.join(pkg_share, "config", "rtabmap_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "mapping.rviz")
    default_db = os.path.join(
        os.path.expanduser("~/Documents/orbbec_astro_pro/output/handheld_map.db")
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
            DeclareLaunchArgument("image_view", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            LogInfo(msg=f"RTAB-Map database will be saved to: {default_db}"),
            # Clean stale OpenNI shared-memory locks from prior runs.
            Node(
                package="astra_camera",
                executable="clean_shm_node",
                name="clean_shm",
                output="screen",
            ),
            # Official Orbbec OpenNI2 + UVC driver (hardware depth-to-color registration).
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
                }.items(),
            ),
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=[rtabmap_params],
                remappings=remappings,
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_params,
                    {"database_path": LaunchConfiguration("database_path")},
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
                    {"database_path": LaunchConfiguration("database_path")},
                ],
                remappings=remappings,
                condition=UnlessCondition(LaunchConfiguration("delete_db")),
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[rtabmap_params],
                remappings=remappings,
                condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
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
