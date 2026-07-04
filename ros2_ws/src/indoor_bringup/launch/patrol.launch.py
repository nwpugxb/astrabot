"""Start the inspection patrol node (Nav2 must already be running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    default_waypoints = os.path.join(share, "config", "patrol_waypoints.yaml")

    waypoints_file = LaunchConfiguration("waypoints_file")
    goal_timeout_s = LaunchConfiguration("goal_timeout_s")
    retry_count = LaunchConfiguration("retry_count")

    return LaunchDescription(
        [
            DeclareLaunchArgument("waypoints_file", default_value=default_waypoints),
            DeclareLaunchArgument("goal_timeout_s", default_value="120.0"),
            DeclareLaunchArgument("retry_count", default_value="1"),
            Node(
                package="indoor_bringup",
                executable="patrol_node",
                name="patrol_node",
                output="screen",
                parameters=[
                    {
                        "waypoints_file": waypoints_file,
                        "goal_timeout_s": goal_timeout_s,
                        "retry_count": retry_count,
                    }
                ],
            ),
        ]
    )
