#!/usr/bin/env python3
"""
Production World Launch Script for mybot_simulation.
Manages Gazebo Sim Server/GUI and strictly hosts the single global /clock bridge.
"""  # noqa: EXE001

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_mybot_simulation = get_package_share_directory("mybot_simulation")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    declared_arguments: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            name="world",
            default_value="my_world_exmaple.world",
            description="Name of the world SDF file located inside worlds directory",
        ),
        DeclareLaunchArgument(
            name="headless",
            default_value="false",
            description="Run Gazebo Sim in headless mode without GUI (-s)",
        ),
    ]

    world_arg = LaunchConfiguration("world")
    headless_arg = LaunchConfiguration("headless")

    models_path = os.path.join(pkg_mybot_simulation, "models")
    ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
    additional_shares = ":".join(
        [os.path.join(p, "share") for p in ament_prefix.split(":") if p]
    )

    set_gz_resource_path = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{models_path}:{pkg_mybot_simulation}:{additional_shares}",
    )

    world_full_path = PathJoinSubstitution(
        [FindPackageShare("mybot_simulation"), "worlds", world_arg]
    )

    # Gazebo Sim Server (-r -s: 즉시 연산, 헤드리스)
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [world_full_path, " -r -s"],
        }.items(),
    )

    # Gazebo Sim GUI (-g: 독립 윈도우 렌더링 컨텍스트)
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g"}.items(),
        condition=UnlessCondition(headless_arg),
    )

    # 단일 전역 시뮬레이션 시계 브리지 (전체 시스템에서 유일한 /clock 퍼블리셔)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="global_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            set_gz_resource_path,
            gz_sim_server,
            gz_sim_gui,
            clock_bridge,
        ]
    )
