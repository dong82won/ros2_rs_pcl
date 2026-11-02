import os
from glob import glob

from setuptools import find_packages, setup

package_name = "realsense_apriltag_rgbd_fusion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="won",
    maintainer_email="2dongwon@gmail.com",
    description="RGB-D Cross-Validated AprilTag 6-DoF Pose Estimation Pipeline for ROS 2 Jazzy",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "rgbd_tag_fusion_node = realsense_apriltag_rgbd_fusion.rgbd_tag_fusion_node2:main",
        ],
    },
)
