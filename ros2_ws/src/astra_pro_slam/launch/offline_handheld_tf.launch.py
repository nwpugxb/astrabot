"""TF for offline handheld bag replay (camera static frames only)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    return LaunchDescription(
        [
            LogInfo(msg="Offline handheld replay: camera static TF only."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mobile_share, "launch", "camera_static_tf.launch.py")
                ),
            ),
        ]
    )
