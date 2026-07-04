"""One-shot navigation stack: sensors + EKF + optional nav layers + Nav2.

Local costmap fusion (see config/nav2_params*.yaml):
  /scan                        - A1 (always)
  /mapping/layers/depth_scan     - Astra low band (use_nav_depth:=true)
  /tof_front|left|right         - ESP32 Range msgs (use_nav_tof:=true)
  /perception/person_scan       - YOLO + Kalman persons (use_nav_person:=true)

A/B comparison examples:
  # Lidar only (baseline)
  ros2 launch indoor_bringup navigation.launch.py \\
    map:=$HOME/maps/indoor.yaml use_nav_depth:=false use_nav_tof:=false

  # Lidar + depth + ToF (full dynamic avoidance)
  ros2 launch indoor_bringup navigation.launch.py \\
    map:=$HOME/maps/indoor.yaml use_nav_depth:=true use_nav_tof:=true \\
    use_fake_odom:=false use_ekf:=true

Prereq: micro-ROS agent on ESP32 when use_ekf or use_nav_tof.
Do not run slam_toolbox / mapping.launch at the same time (AMCL needs static map).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _pick_nav2_params(share: str, use_depth: bool, use_tof: bool) -> str:
    cfg = os.path.join(share, "config")
    if not use_depth and not use_tof:
        return os.path.join(cfg, "nav2_params_lidar_only.yaml")
    if use_depth and not use_tof:
        return os.path.join(cfg, "nav2_params_no_tof.yaml")
    if not use_depth and use_tof:
        return os.path.join(cfg, "nav2_params_tof_only.yaml")
    return os.path.join(cfg, "nav2_params.yaml")


def _layer_label(use_depth: bool, use_tof: bool, use_person: bool) -> str:
    parts = ["/scan"]
    if use_depth:
        parts.append("/mapping/layers/depth_scan")
    if use_tof:
        parts.append("/tof_*")
    if use_person:
        parts.append("/perception/person_scan")
    return " + ".join(parts)


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("indoor_bringup")
    launch_dir = os.path.join(share, "launch")
    layer_cfg = os.path.join(share, "config", "mapping_layers.yaml")
    rviz_nav = os.path.join(share, "rviz", "mapping_layers.rviz")

    use_nav_depth = LaunchConfiguration("use_nav_depth").perform(context) == "true"
    use_nav_tof = LaunchConfiguration("use_nav_tof").perform(context) == "true"
    use_nav_person = LaunchConfiguration("use_nav_person").perform(context) == "true"
    params_file = _pick_nav2_params(share, use_nav_depth, use_nav_tof)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_odom = LaunchConfiguration("use_fake_odom")
    use_ekf = LaunchConfiguration("use_ekf")
    use_mag = LaunchConfiguration("use_mag")
    enable_camera = LaunchConfiguration("enable_camera")
    use_hardware_sensors = LaunchConfiguration("use_hardware_sensors")
    use_robot_description = LaunchConfiguration("use_robot_description")
    lidar_port = LaunchConfiguration("lidar_port")
    map_yaml = LaunchConfiguration("map")
    rviz = LaunchConfiguration("rviz")

    def include(name, extra=None):
        args = {"use_sim_time": use_sim_time}
        if extra:
            args.update(extra)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items(),
        )

    actions = [
        LogInfo(
            msg=[
                "Navigation local_costmap sources: ",
                _layer_label(use_nav_depth, use_nav_tof, use_nav_person),
                " | params=",
                params_file,
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "description.launch.py")
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
            condition=IfCondition(use_robot_description),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "sensors.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "enable_camera": enable_camera,
                "lidar_port": lidar_port,
            }.items(),
            condition=IfCondition(use_hardware_sensors),
        ),
        Node(
            package="indoor_bringup",
            executable="depth_scan_layer_node",
            name="depth_scan_layer",
            output="screen",
            parameters=[layer_cfg],
            condition=IfCondition(LaunchConfiguration("use_nav_depth")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "person_tracker.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "publish_scan": "true",
                "detection_mode": LaunchConfiguration("person_detection_mode"),
                "model_path": LaunchConfiguration("person_model_path"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_nav_person")),
        ),
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
        include(
            "nav2.launch.py",
            {
                "params_file": params_file,
                "map": map_yaml,
                "use_sim_time": use_sim_time,
            },
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_nav],
            condition=IfCondition(rviz),
        ),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    default_map = os.path.expanduser("~/maps/indoor.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "enable_camera",
                default_value="true",
                description="Astra driver (required when use_nav_depth:=true).",
            ),
            DeclareLaunchArgument(
                "use_hardware_sensors",
                default_value="true",
                description="Launch sllidar + astra_camera (false in Gazebo sim).",
            ),
            DeclareLaunchArgument(
                "use_robot_description",
                default_value="true",
                description="Launch robot_state_publisher from URDF (false if Gazebo already did).",
            ),
            DeclareLaunchArgument(
                "use_fake_odom",
                default_value="false",
                description="Static odom->base_footprint TF for bench tests without ESP32.",
            ),
            DeclareLaunchArgument(
                "use_ekf",
                default_value="true",
                description="robot_localization EKF (/odometry/filtered).",
            ),
            DeclareLaunchArgument("use_mag", default_value="false"),
            DeclareLaunchArgument(
                "use_nav_depth",
                default_value="true",
                description="Astra depth_scan_layer -> local_costmap obstacle_layer.",
            ),
            DeclareLaunchArgument(
                "use_nav_tof",
                default_value="true",
                description="ESP32 /tof_* -> local_costmap RangeSensorLayer.",
            ),
            DeclareLaunchArgument(
                "use_nav_person",
                default_value="false",
                description="YOLO person tracker -> /perception/person_scan in local_costmap.",
            ),
            DeclareLaunchArgument(
                "person_detection_mode",
                default_value="internal",
                description="internal (Ultralytics) or external (vision_msgs / ros2_yolo).",
            ),
            DeclareLaunchArgument(
                "person_model_path",
                default_value="yolov8n.pt",
            ),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
