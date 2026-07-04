"""Full layered mapping: robot + sensors + scan layers + slam_toolbox + RViz.

Layer debug (RViz LaserScan colors):
  Red-ish   /scan or /mapping/layers/lidar_scan
  Depth     /mapping/layers/depth_scan
  ToF       /mapping/layers/tof_scan
  Fused     /scan_fused  -> slam_toolbox when use_fusion:=true

Examples:
  # Default: A1 + depth + ToF fused SLAM
  ros2 launch indoor_bringup mapping.launch.py lidar_port:=/dev/ttyUSB0 \\
    use_fake_odom:=false use_ekf:=true

  # Layer debug: A1 only
  ros2 launch indoor_bringup mapping.launch.py use_layer_depth:=false \\
    use_layer_tof:=false use_fusion:=false

  # Depth layer only (no SLAM fusion — inspect /mapping/layers/depth_scan)
  ros2 launch indoor_bringup mapping.launch.py use_fusion:=false \\
    use_layer_tof:=false slam:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    launch_dir = os.path.join(share, "launch")
    rviz_layers = os.path.join(share, "rviz", "mapping_layers.rviz")
    rviz_default = os.path.join(share, "rviz", "indoor_slam.rviz")
    default_slam_params = os.path.join(share, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_odom = LaunchConfiguration("use_fake_odom")
    use_ekf = LaunchConfiguration("use_ekf")
    use_mag = LaunchConfiguration("use_mag")
    enable_camera = LaunchConfiguration("enable_camera")
    use_fusion = LaunchConfiguration("use_fusion")
    slam = LaunchConfiguration("slam")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    use_hardware_sensors = LaunchConfiguration("use_hardware_sensors")
    use_robot_description = LaunchConfiguration("use_robot_description")
    scan_topic = PythonExpression([
        "'/scan_fused' if '", use_fusion, "' == 'true' else '/scan'",
    ])

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
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
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
            DeclareLaunchArgument("use_fake_odom", default_value="true"),
            DeclareLaunchArgument("use_ekf", default_value="false"),
            DeclareLaunchArgument("use_mag", default_value="false"),
            DeclareLaunchArgument("use_layer_depth", default_value="true"),
            DeclareLaunchArgument("use_layer_tof", default_value="true"),
            DeclareLaunchArgument("use_fusion", default_value="true"),
            DeclareLaunchArgument("slam", default_value="true",
                                  description="Run slam_toolbox on scan_topic."),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_layers),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="slam_toolbox params yaml (use slam_toolbox_sim.yaml in Gazebo).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "description.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(use_robot_description),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "sensors.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "enable_camera": enable_camera,
                    "lidar_port": LaunchConfiguration("lidar_port"),
                }.items(),
                condition=IfCondition(use_hardware_sensors),
            ),
            include(
                "mapping_layers.launch.py",
                {
                    "use_layer_depth": LaunchConfiguration("use_layer_depth"),
                    "use_layer_tof": LaunchConfiguration("use_layer_tof"),
                    "use_fusion": use_fusion,
                },
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "slam_params_file": LaunchConfiguration("slam_params_file"),
                }.items(),
                condition=IfCondition(slam),
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(rviz),
            ),
        ]
    )
