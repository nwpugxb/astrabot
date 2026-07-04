import os
from glob import glob

from setuptools import setup

package_name = "indoor_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaobo",
    maintainer_email="user@todo.todo",
    description="Indoor inspection robot bringup: A1 lidar + Astra Pro + slam_toolbox.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "patrol_node = indoor_bringup.patrol_node:main",
            "depth_scan_layer_node = indoor_bringup.depth_scan_layer_node:main",
            "tof_scan_layer_node = indoor_bringup.tof_scan_layer_node:main",
            "scan_fusion_node = indoor_bringup.scan_fusion_node:main",
            "person_tracker_node = indoor_bringup.person_tracker_node:main",
            "sim_camera_info_node = indoor_bringup.sim_camera_info_node:main",
            "gazebo_tof_range_node = indoor_bringup.gazebo_tof_range_node:main",
            "sim_gazebo_adapter_node = indoor_bringup.sim_gazebo_adapter_node:main",
            "sim_teleop_gui_node = indoor_bringup.sim_teleop_gui_node:main",
            "motor_pwm_tune_gui_node = indoor_bringup.motor_pwm_tune_gui_node:main",
            "sim_world_viz_node = indoor_bringup.sim_world_viz_node:main",
            "sim_slam_scan_viz_node = indoor_bringup.sim_slam_scan_viz_node:main",
            "sim_gt_odom_node = indoor_bringup.sim_gt_odom_node:main",
            "static_joint_state_node = indoor_bringup.static_joint_state_node:main",
        ],
    },
)
