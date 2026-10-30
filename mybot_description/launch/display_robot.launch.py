#!/usr/bin/env python3
"""
Production-Grade Robot Display Launch File for ROS 2 Jazzy.

Features:
- Dynamically resolves Xacro binary via FindExecutable.
- Conditionally switches between GUI and Headless Joint State Publishing.
- Cleanly shuts down the entire launch system when RViz2 exits (Eliminates zombie processes).
- Fully compliant with Modern Python 3.12+ and PEP 8 standards.
"""  # noqa: EXE001

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_description: str = "mybot_description"

    # 1. 런치 파라미터 정의
    declared_arguments: list[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            name="gui",
            default_value="true",
            description="Flag to enable joint_state_publisher_gui",
        ),
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        DeclareLaunchArgument(
            name="rviz_config",
            default_value="urdf_vis.rviz",
            description="Name of the RViz config file inside rviz directory",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # 2. Xacro 및 RViz 경로 치환 구성
    pkg_share = FindPackageShare(pkg_description)

    xacro_file_path = PathJoinSubstitution(
        [pkg_share, "urdf", "mbot_realsense", "mybot3_cam.xacro"]
    )

    default_rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", rviz_config])

    # 3. 안전한 Xacro 파싱 커맨드 (FindExecutable 적용)
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file_path]),
        value_type=str,
    )

    # 4. 노드 정의
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # GUI 활성화 조건부 구동
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(gui),
    )

    # Headless 환경(CI/CD 등) 구동 시 자동 폴백
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 5. 라이프사이클 이벤트 핸들러: RViz2 종료 시 전체 Launch 종료 (좀비 프로세스 방지)
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_node,
            on_exit=[EmitEvent(event=Shutdown(reason="RViz2 window closed by user."))],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node,
            exit_event_handler,
        ]
    )
