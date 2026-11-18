import os
from glob import glob

from setuptools import setup

name = "vision_pose_estimator"
setup(
    name=name,
    version="0.1.0",
    packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml"]),
    ]
    + [
        (os.path.join("share", name, folder), glob(folder + "/*"))
        for folder in ["launch", "config", "rviz"]
        if os.path.isdir(folder)
    ],# type: ignore

    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS developer",
    maintainer_email="developer@example.com",
    description="UR Eye-in-Hand AprilTag integration",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tag_visualizer_node = vision_pose_estimator.tag_visualizer_node:main",
        ]
    },
)
