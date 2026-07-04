"""Gazebo Sim: indoor robot with diff drive, lidar, RGB-D, IMU, ToF.

Requires (Ubuntu 22.04 + Humble):
  sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \\
    ros-humble-ros-gz-image ros-humble-gz-ros2-control \\
    ros-humble-controller-manager ros-humble-diff-drive-controller \\
    ros-humble-joint-state-broadcaster ros-humble-xacro

Examples:
  ros2 launch indoor_bringup gazebo.launch.py
  ros2 launch indoor_bringup gazebo.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_robot(context, *args, **kwargs):
    """Spawn from URDF file — create -topic robot_description fails on Humble (no latched publisher)."""
    share = get_package_share_directory("indoor_bringup")
    xacro_path = os.path.join(share, "urdf", "indoor_robot.gazebo.xacro")
    urdf_path = os.path.join(share, "urdf", "indoor_robot.urdf")
    controllers = LaunchConfiguration("controllers_file").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)

    import subprocess

    robot_param_node = LaunchConfiguration("robot_param_node").perform(context)
    urdf_content = subprocess.check_output(
        [
            "xacro",
            xacro_path,
            f"controllers_file:={controllers}",
            f"robot_urdf:={urdf_path}",
            f"robot_param_node:={robot_param_node}",
        ],
        text=True,
    )
    spawn_file = "/tmp/indoor_robot_gazebo_spawn.urdf"
    with open(spawn_file, "w", encoding="utf-8") as f:
        f.write(urdf_content)

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", spawn_file,
            "-name", "indoor_robot",
            "-x", spawn_x, "-y", spawn_y,
            "-z", spawn_z,
            "-Y", "0",
        ],
    )
    return [TimerAction(period=5.0, actions=[spawn])]


def generate_launch_description():
    share = get_package_share_directory("indoor_bringup")
    ros_gz_share = get_package_share_directory("ros_gz_sim")
    world = os.path.join(share, "worlds", "indoor_sim.sdf")
    xacro_path = os.path.join(share, "urdf", "indoor_robot.gazebo.xacro")
    urdf_path = os.path.join(share, "urdf", "indoor_robot.urdf")
    controllers_default = os.path.join(share, "config", "gazebo_diff_drive_sim.yaml")
    controllers_file = LaunchConfiguration("controllers_file")

    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")

    # repr(world) so paths like /home/... are valid inside eval() (slashes are not division).
    world_q = repr(world)
    gz_args = PythonExpression([
        "'-r ' + ", world_q, " + ' -s --headless-rendering' if '", headless,
        "' == 'true' else '-r ' + ", world_q, " + ' -g'",
    ])

    robot_param_node = LaunchConfiguration("robot_param_node")
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " controllers_file:=", controllers_file,
            " robot_urdf:=", urdf_path,
            " robot_param_node:=", robot_param_node,
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=robot_param_node,
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/tof_front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/tof_left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/tof_right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/scan", "/scan_unfixed"),
            ("/imu", "/imu/data_raw"),
        ],
    )

    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/world/indoor_sim/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        output="screen",
        arguments=["/camera/image", "/camera/depth_image"],
        remappings=[
            ("/camera/image", "/camera/color/image_raw"),
            ("/camera/depth_image", "/gz/camera/depth/image_raw"),
        ],
    )

    joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    diff_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
        ],
    )

    sim_camera_info = Node(
        package="indoor_bringup",
        executable="sim_camera_info_node",
        name="sim_camera_info",
        output="screen",
    )

    tof_range = Node(
        package="indoor_bringup",
        executable="gazebo_tof_range_node",
        name="gazebo_tof_range",
        output="screen",
    )

    sim_adapter = Node(
        package="indoor_bringup",
        executable="sim_gazebo_adapter_node",
        name="sim_gazebo_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_in": "/sim/gt_odom",
            }
        ],
    )

    sim_gt_odom = Node(
        package="indoor_bringup",
        executable="sim_gt_odom_node",
        name="sim_gt_odom",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "spawn_x": -2.0,
                "spawn_y": -2.0,
            }
        ],
    )

    sim_world_viz = Node(
        package="indoor_bringup",
        executable="sim_world_viz_node",
        name="sim_world_viz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    sim_slam_scan_viz = Node(
        package="indoor_bringup",
        executable="sim_slam_scan_viz_node",
        name="sim_slam_scan_viz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    delayed_controllers = TimerAction(period=12.0, actions=[joint_spawner, diff_spawner])

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "robot_param_node",
                default_value="indoor_robot_rsp",
                description="Node holding robot_description for gz_ros2_control.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="If true, run gz server only (-s); else open GUI (-g).",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.08",
                description="Spawn height so wheels contact ground.",
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="-2.0",
                description="Robot spawn X in Gazebo world (free corner, away from center obstacle).",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-2.0",
                description="Robot spawn Y in Gazebo world.",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control yaml (sim yaml: wheel odom only, GT node owns TF).",
            ),
            gz_sim,
            rsp,
            bridge,
            pose_bridge,
            image_bridge,
            sim_camera_info,
            tof_range,
            sim_adapter,
            sim_gt_odom,
            sim_world_viz,
            sim_slam_scan_viz,
            OpaqueFunction(function=_spawn_robot),
            delayed_controllers,
        ]
    )
