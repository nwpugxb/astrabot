"""Mobile robot SLAM: Orbbec Astra Pro + Arduino wheel odometry + RTAB-Map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mobile_share = get_package_share_directory("mobile_base")
    astra_share = get_package_share_directory("astra_camera")
    urdf_path = os.path.join(mobile_share, "urdf", "deck_robot.urdf")
    rtabmap_params = os.path.join(mobile_share, "config", "rtabmap_mobile.yaml")
    base_params = os.path.join(mobile_share, "config", "base.yaml")
    rviz_config = os.path.join(mobile_share, "rviz", "mobile_mapping.rviz")
    default_db = os.path.join(
        os.path.expanduser("~/Documents/orbbec_astro_pro/output/mobile_map.db")
    )

    remappings = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("odom", "/odom"),
        ("scan_cloud", "/wall_plane_cloud"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("delete_db", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("database_path", default_value=default_db),
            DeclareLaunchArgument("camera_pitch_deg", default_value="17.0"),
            DeclareLaunchArgument("camera_roll_deg", default_value="0.0"),
            DeclareLaunchArgument("use_laser_scan", default_value="false"),
            DeclareLaunchArgument("use_wall_plane_cloud", default_value="false"),
            DeclareLaunchArgument("rtabmap_config", default_value=rtabmap_params),
            LogInfo(msg=f"Mobile map database: {default_db}"),
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
                    "oni_log_level": "warning",
                    "oni_log_to_console": "false",
                    "publish_tf": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mobile_share, "launch", "camera_static_tf.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "camera_pitch_deg": LaunchConfiguration("camera_pitch_deg"),
                    "camera_roll_deg": LaunchConfiguration("camera_roll_deg"),
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
                        "scan_height": 1,
                        "range_min": 0.08,
                        "range_max": 4.0,
                        "output_frame": "base_link",
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_laser_scan")),
            ),
            Node(
                package="mobile_base",
                executable="wall_plane_cloud_node",
                name="wall_plane_cloud",
                output="screen",
                remappings=[
                    ("depth/image_raw", "/camera/depth/image_raw"),
                    ("depth/camera_info", "/camera/color/camera_info"),
                ],
                condition=IfCondition(LaunchConfiguration("use_wall_plane_cloud")),
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
                parameters=[{"stride": 2, "max_depth_m": 4.0}],
            ),
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    LaunchConfiguration("rtabmap_config"),
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
                    LaunchConfiguration("rtabmap_config"),
                    {"database_path": LaunchConfiguration("database_path")},
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
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
