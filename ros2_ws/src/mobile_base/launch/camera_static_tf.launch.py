"""Static TF for Orbbec Astra Pro camera frames (matches driver, no USB needed).

camera_pitch_deg: tilt around +Y (REP-103). POSITIVE = nose down (+17 = 17 deg below horizontal).
camera_roll_deg: fine tilt around +X for when floor plane looks twisted in RViz.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    pitch_deg = LaunchConfiguration("camera_pitch_deg")
    roll_deg = LaunchConfiguration("camera_roll_deg")
    pitch_rad = PythonExpression(["str(", pitch_deg, " * ", str(math.pi), " / 180.0)"])
    roll_rad = PythonExpression(["str(", roll_deg, " * ", str(math.pi), " / 180.0)"])

    ox, oy, oz = -math.pi / 2, 0.0, -math.pi / 2
    optical_args = [
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", str(ox), "--pitch", str(oy), "--yaw", str(oz),
    ]
    mount_args = [
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", roll_rad, "--pitch", pitch_rad, "--yaw", "0",
    ]
    sim = [{"use_sim_time": use_sim_time}]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("camera_pitch_deg", default_value="17.0"),
            DeclareLaunchArgument("camera_roll_deg", default_value="0.0"),
            LogInfo(msg=["Camera static TF: pitch=", pitch_deg, " deg roll=", roll_deg, " deg (REP-103, +pitch = down)."]),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_depth_frame_tf",
                arguments=[*mount_args, "--frame-id", "camera_link",
                           "--child-frame-id", "camera_depth_frame"],
                parameters=sim,
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_depth_optical_tf",
                arguments=[*optical_args, "--frame-id", "camera_depth_frame",
                           "--child-frame-id", "camera_depth_optical_frame"],
                parameters=sim,
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_color_frame_tf",
                arguments=[*mount_args, "--frame-id", "camera_link",
                           "--child-frame-id", "camera_color_frame"],
                parameters=sim,
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_color_optical_tf",
                arguments=[*optical_args, "--frame-id", "camera_color_frame",
                           "--child-frame-id", "camera_color_optical_frame"],
                parameters=sim,
            ),
        ]
    )
