"""Full stack in Gazebo: physics sim + mapping or Nav2 closed loop.

Examples:
  # Nav2 on sim map (depth + ToF layers on)
  ros2 launch indoor_bringup simulation.launch.py stack:=navigation

  # slam_toolbox in sim
  ros2 launch indoor_bringup simulation.launch.py stack:=mapping headless:=true

  # Compare lidar-only vs fused avoidance
  ros2 launch indoor_bringup simulation.launch.py use_nav_depth:=false use_nav_tof:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    launch_dir = os.path.join(share, "launch")
    default_map = os.path.join(share, "maps", "sim_lab.yaml")
    rviz_config = os.path.join(share, "rviz", "mapping_layers.rviz")
    rviz_sim_mapping = os.path.join(share, "rviz", "sim_mapping.rviz")
    slam_sim_params = os.path.join(share, "config", "slam_toolbox_sim.yaml")
    fastdds_no_shm = os.path.join(share, "config", "fastdds_no_shm.xml")

    stack = LaunchConfiguration("stack")
    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
        }.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "mapping.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_fake_odom": "false",
            "use_ekf": "false",
            "use_hardware_sensors": "false",
            "use_robot_description": "false",
            "enable_camera": "false",
            "slam": "true",
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": rviz_sim_mapping,
            "use_layer_depth": "false",
            "use_layer_tof": "false",
            "use_fusion": "false",
            "slam_params_file": slam_sim_params,
        }.items(),
        condition=IfCondition(PythonExpression(["'", stack, "' == 'mapping'"])),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "navigation.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_fake_odom": "false",
            "use_ekf": "false",
            "use_hardware_sensors": "false",
            "use_robot_description": "false",
            "enable_camera": "false",
            "map": LaunchConfiguration("map"),
            "rviz": LaunchConfiguration("rviz"),
            "use_nav_depth": LaunchConfiguration("use_nav_depth"),
            "use_nav_tof": LaunchConfiguration("use_nav_tof"),
            "use_nav_person": LaunchConfiguration("use_nav_person"),
            "person_detection_mode": LaunchConfiguration("person_detection_mode"),
        }.items(),
        condition=IfCondition(PythonExpression(["'", stack, "' == 'navigation'"])),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_no_shm
            ),
            # Gazebo GUI often hangs on Wayland; RViz is enough for Nav2. Use headless:=false to try GUI.
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "headless",
                default_value="true",
                description="true = gz server only (recommended); false = open Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "stack",
                default_value="navigation",
                description="navigation (AMCL+Nav2) or mapping (slam_toolbox).",
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_nav_depth", default_value="true"),
            DeclareLaunchArgument("use_nav_tof", default_value="true"),
            DeclareLaunchArgument("use_nav_person", default_value="false"),
            DeclareLaunchArgument("person_detection_mode", default_value="internal"),
            LogInfo(msg=["Gazebo simulation stack=", stack]),
            gazebo,
            mapping,
            navigation,
        ]
    )
