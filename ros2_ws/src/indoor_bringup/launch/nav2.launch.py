"""Nav2 autonomous navigation (M5) for the indoor robot.

Brings up: map_server + AMCL (localize on a saved map) and the navigation servers
(controller=Regulated Pure Pursuit, planner=NavFn, behaviors, BT, waypoint
follower, velocity smoother), each managed by a lifecycle_manager.

Prereqs at runtime:
  - a saved map (map:=~/maps/indoor.yaml)
  - the base publishing /odom + EKF providing odom->base_footprint (use_ekf:=true)
  - /scan from the A1 and /tof_* from the ESP32

Final velocity command goes out on /cmd_vel (controller -> velocity_smoother -> /cmd_vel),
which the ESP32 micro-ROS node subscribes to.

  ros2 launch indoor_bringup nav2.launch.py map:=$HOME/maps/indoor.yaml

Prefer navigation.launch.py for the full stack (sensors + layers + Nav2):
  ros2 launch indoor_bringup navigation.launch.py map:=$HOME/maps/indoor.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    default_params = os.path.join(share, "config", "nav2_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    common = [params_file, {"use_sim_time": use_sim_time}]

    localization_nodes = ["map_server", "amcl"]
    navigation_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "map", default_value=os.path.expanduser("~/maps/indoor.yaml")
            ),
            # ---- Localization ----
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=common,
            ),
            # ---- Navigation servers ----
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=common,
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=common,
                remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
            ),
            # ---- Lifecycle managers ----
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": localization_nodes,
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": navigation_nodes,
                    }
                ],
            ),
        ]
    )
