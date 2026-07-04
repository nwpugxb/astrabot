from setuptools import setup
import os
from glob import glob

package_name = "mobile_base"

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
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaobo",
    maintainer_email="user@todo.todo",
    description="Arduino mobile base bridge for deck robot",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arduino_base_node = mobile_base.arduino_base_node:main",
            "odom_path_node = mobile_base.odom_path_node:main",
            "odom_tf_broadcaster = mobile_base.odom_tf_broadcaster:main",
            "odom_covariance = mobile_base.odom_covariance_node:main",
            "wall_plane_cloud_node = mobile_base.wall_plane_cloud_node:main",
        ],
    },
)
