import os
from glob import glob

from setuptools import find_packages, setup

package_name = "apriltag_6dof"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        # RViz2 설정 파일 배포 경로 추가
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="won",
    maintainer_email="2dongwon@gmail.com",
    description="Monocular 6-DoF AprilTag pose estimation and AR visualization node",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["tag_fusion_node = apriltag_6dof.tag_fusion_node:main"],
    },
)
