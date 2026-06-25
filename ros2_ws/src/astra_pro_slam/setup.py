import os
from glob import glob

from setuptools import find_packages, setup

package_name = "astra_pro_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaobo",
    maintainer_email="user@todo.todo",
    description="Orbbec Astra Pro RGB-D SLAM for ROS2 Humble",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = astra_pro_slam.camera_node:main",
            "rgbd_cloud_node = astra_pro_slam.rgbd_cloud_node:main",
        ],
    },
)
