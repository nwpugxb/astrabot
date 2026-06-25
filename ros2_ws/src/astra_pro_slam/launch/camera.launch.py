from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("astra_pro_slam")
    camera_config = os.path.join(pkg_share, "config", "camera.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("color_device", default_value="auto"),
            Node(
                package="astra_pro_slam",
                executable="camera_node",
                name="astra_pro_camera",
                output="screen",
                parameters=[
                    camera_config,
                    {"color_device": LaunchConfiguration("color_device")},
                ],
                namespace="camera",
            ),
        ]
    )
