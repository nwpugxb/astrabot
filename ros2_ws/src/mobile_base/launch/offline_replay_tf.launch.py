"""TF for offline bag replay: current URDF + camera static frames + odom TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    urdf_path = os.path.join(mobile_share, "urdf", "deck_robot.urdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("camera_pitch_deg", default_value="17.0"),
            DeclareLaunchArgument("camera_roll_deg", default_value="0.0"),
            LogInfo(msg="Offline replay TF from current URDF (ignores bag /tf_static)."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "robot_description": ParameterValue(
                            Command(["cat ", urdf_path]), value_type=str
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mobile_share, "launch", "camera_static_tf.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "camera_pitch_deg": LaunchConfiguration("camera_pitch_deg"),
                    "camera_roll_deg": LaunchConfiguration("camera_roll_deg"),
                }.items(),
            ),
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",
                name="depth_to_scan",
                output="screen",
                remappings=[
                    ("depth", "/camera/depth/image_raw"),
                    ("depth_camera_info", "/camera/depth/camera_info"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "scan_height": 1,
                        "range_min": 0.08,
                        "range_max": 4.0,
                        "output_frame": "base_link",
                    }
                ],
            ),
            Node(
                package="mobile_base",
                executable="odom_covariance",
                name="odom_covariance",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "input_topic": "/odom_bag",
                        "output_topic": "/odom",
                    }
                ],
            ),
            Node(
                package="mobile_base",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
