"""Load a saved mobile_map.db and show /cloud_map in RViz (no camera / Arduino)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    rtabmap_params = os.path.join(mobile_share, "config", "rtabmap_mobile.yaml")
    rviz_config = os.path.join(mobile_share, "rviz", "mobile_mapping.rviz")
    default_db = os.path.join(
        os.path.expanduser("~/Documents/orbbec_astro_pro/output/mobile_map.db")
    )

    database_path = LaunchConfiguration("database_path")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("rviz", default_value="true"),
            LogInfo(msg=["View saved map: ", database_path]),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_params,
                    {
                        "database_path": database_path,
                        "subscribe_depth": False,
                        "subscribe_odom": False,
                        "subscribe_scan": False,
                        "publish_tf": True,
                        "Mem/IncrementalMemory": "false",
                        "Mem/InitWMWithAllNodes": "true",
                        "RGBD/ProximityBySpace": "false",
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
