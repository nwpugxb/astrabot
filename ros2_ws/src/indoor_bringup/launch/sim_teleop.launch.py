"""Teleop for Gazebo sim (GUI — works in Cursor / non-TTY terminals).

Must start simulation FIRST:
  ./run_sim_mapping.sh

Then in another terminal:
  ./run_sim_teleop.sh
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="indoor_bringup",
                executable="sim_teleop_gui_node",
                name="sim_teleop_gui",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
