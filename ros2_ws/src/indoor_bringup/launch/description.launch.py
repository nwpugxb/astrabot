"""Robot model TF: robot_state_publisher + placeholder wheel joint_states."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    urdf_path = os.path.join(share, "urdf", "indoor_robot.urdf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(Command(["cat ", urdf_path]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": use_sim_time}
                ],
            ),
            # Wheel joints are continuous; publish zeros until real /joint_states exist.
            Node(
                package="indoor_bringup",
                executable="static_joint_state_node",
                name="static_joint_state",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
