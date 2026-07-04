"""Indoor robot bringup (M3-M4): description + sensors + slam_toolbox + RViz.

The base (ESP32) is NOT included here yet. Until the firmware publishes
odom->base_footprint TF + /odom, run with use_fake_odom:=true to verify the
pipeline (TF tree, /scan, slam_toolbox) on a stationary / hand-pushed robot.
Once firmware is ready: set use_fake_odom:=false and launch the base node
alongside this (it must publish the odom->base_footprint transform).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    launch_dir = os.path.join(share, "launch")
    rviz_config = os.path.join(share, "rviz", "indoor_slam.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_odom = LaunchConfiguration("use_fake_odom")
    use_ekf = LaunchConfiguration("use_ekf")
    use_mag = LaunchConfiguration("use_mag")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_rviz = LaunchConfiguration("rviz")

    def include(name, extra=None):
        args = {"use_sim_time": use_sim_time}
        if extra:
            args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_fake_odom",
                default_value="true",
                description="Publish identity odom->base_footprint until ESP32 firmware exists. "
                "Set false (and use_ekf:=true) once /odom + /imu/data_raw are available.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="false",
                description="Fuse wheel /odom + MPU9250 /imu via robot_localization "
                "(owns odom->base_footprint). Mutually exclusive with use_fake_odom.",
            ),
            DeclareLaunchArgument(
                "use_mag",
                default_value="false",
                description="Use MPU9250 magnetometer in the IMU filter (see localization.launch.py).",
            ),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            include("description.launch.py"),
            include("sensors.launch.py", {"enable_camera": enable_camera}),
            include("slam.launch.py"),
            # Temporary stand-in for wheel odometry (debug only, before firmware).
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="fake_odom_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "odom", "--child-frame-id", "base_footprint",
                ],
                condition=IfCondition(use_fake_odom),
            ),
            # Real fusion (wheel + IMU) once firmware publishes /odom + /imu/data_raw.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "localization.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_mag": use_mag,
                }.items(),
                condition=IfCondition(use_ekf),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(enable_rviz),
            ),
        ]
    )
