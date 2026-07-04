"""Sensor fusion: MPU9250 (GY-9250) orientation filter + robot_localization EKF.

Chain:
  ESP32 -> /imu/data_raw (+ /imu/mag) -> imu_filter_madgwick -> /imu/data ┐
  ESP32 -> /odom ─────────────────────────────────────────────────────────┼-> EKF -> odom->base_footprint TF
                                                                           ┘         + /odometry/filtered

Requires: ros-humble-imu-filter-madgwick, ros-humble-robot-localization.
Run this INSTEAD of use_fake_odom once the ESP32 firmware publishes /odom + /imu/data_raw
(the ESP32 wheel-odom node must have publish_tf=false; the EKF owns odom->base_footprint).

use_mag: MPU9250 has a magnetometer (AK8963). Default false (indoor mag is noisy
near steppers/steel). To enable: calibrate hard/soft-iron, have the ESP32 publish
/imu/mag (sensor_msgs/MagneticField), launch with use_mag:=true, and also set
imu0_differential:=false in ekf.yaml to fuse absolute yaw.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    ekf_params = os.path.join(share, "config", "ekf.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mag = LaunchConfiguration("use_mag")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "use_mag",
                default_value="false",
                description="Use MPU9250 magnetometer for absolute heading "
                "(needs calibration + /imu/mag; risky indoors).",
            ),
            # MPU9250 raw -> orientation. Magnetometer optional (use_mag).
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter",
                output="screen",
                parameters=[
                    {
                        "use_mag": use_mag,
                        "world_frame": "enu",
                        "publish_tf": False,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("imu/data_raw", "/imu/data_raw"),
                    ("imu/mag", "/imu/mag"),
                    ("imu/data", "/imu/data"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
