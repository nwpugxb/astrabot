"""Record-only: Orbbec camera + Arduino wheel odometry + robot TF (no SLAM)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    astra_share = get_package_share_directory("astra_camera")
    urdf_path = os.path.join(mobile_share, "urdf", "deck_robot.urdf")
    base_params = os.path.join(mobile_share, "config", "base.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("color_fps", default_value="15"),
            DeclareLaunchArgument("depth_fps", default_value="15"),
            LogInfo(msg="Record mode: camera + wheel odom + TF. No SLAM."),
            Node(
                package="astra_camera",
                executable="clean_shm_node",
                name="clean_shm",
                output="screen",
            ),
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
                    "color_fps": LaunchConfiguration("color_fps"),
                    "depth_fps": LaunchConfiguration("depth_fps"),
                    "oni_log_level": "warning",
                    "oni_log_to_console": "false",
                }.items(),
            ),
            Node(
                package="mobile_base",
                executable="arduino_base_node",
                name="arduino_base",
                output="screen",
                parameters=[base_params],
            ),
            Node(
                package="mobile_base",
                executable="odom_path_node",
                name="odom_path",
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["cat ", urdf_path]), value_type=str
                        )
                    }
                ],
            ),
        ]
    )
