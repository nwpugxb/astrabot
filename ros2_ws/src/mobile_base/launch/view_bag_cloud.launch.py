"""Replay a mobile bag and show RGB-D point cloud in RViz (odom frame).

Use this to verify camera TF: in Fixed Frame=odom, ground should lie on the XY
grid (normal = +Z). Robot motion should follow yellow path along +X.
Tune camera_pitch_deg if the floor appears vertical (perpendicular to +X).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    rviz_config = os.path.join(mobile_share, "rviz", "bag_cloud.rviz")

    pitch = LaunchConfiguration("camera_pitch_deg")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_pitch_deg", default_value="17.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            LogInfo(msg=["Bag cloud viewer — pitch=", pitch, " deg below horizontal"]),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mobile_share, "launch", "offline_replay_tf.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "camera_pitch_deg": pitch,
                }.items(),
            ),
            Node(
                package="mobile_base",
                executable="odom_path_node",
                name="odom_path",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="astra_pro_slam",
                executable="rgbd_cloud_node",
                name="rgbd_cloud",
                output="screen",
                remappings=[
                    ("color/image_raw", "/camera/color/image_raw"),
                    ("depth/image_raw", "/camera/depth/image_raw"),
                    ("color/camera_info", "/camera/color/camera_info"),
                ],
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "stride": 2,
                        "max_depth_m": 4.0,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
