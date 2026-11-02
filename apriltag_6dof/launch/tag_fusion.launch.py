"""2D AprilTag 검출기와 6-DoF 융합 추정 노드를 일괄 기동하는 통합 Launch 스크립트."""

import os
from typing import cast
from xml.dom import minidom

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # 0. 시뮬레이션 시간(rosbag /clock 동기화) 파라미터 선언
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="rosbag 재생 시 시뮬레이션 클록(/clock) 동기화 활성화 여부",
    )

    # 1. 패키지 설치 디렉터리(share) 내 설정 파일 절대 경로 동적 해석
    pkg_share = get_package_share_directory("apriltag_6dof")
    apriltag_config = os.path.join(pkg_share, "config", "apriltag_settings.yaml")
    fusion_config = os.path.join(pkg_share, "config", "tag_params.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "tag_fusion.rviz")

    # 2. 시스템에 설치된 공식 realsense2_description 내 D435 xacro 모델 절대 경로 해석
    realsense_desc_share = get_package_share_directory("realsense2_description")
    urdf_file = os.path.join(
        realsense_desc_share, "urdf", "test_d435_camera.urdf.xacro"
    )

    # 3. Xacro 파싱 및 명시적 타입 단언(Pylance TextIOWrapper 오추론 해결)
    # xacro.process_file()은 런타임에 xml.dom.minidom.Document를 반환합니다.
    robot_description_doc = cast(minidom.Document, xacro.process_file(urdf_file))
    robot_description = {"robot_description": robot_description_doc.toxml()}

    # 4. RealSense D435 3D CAD 메쉬를 /robot_description 토픽으로 발행하는 노드
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 2. AprilTag 2D 검출 노드 (C++ apriltag_ros)
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        parameters=[
            apriltag_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
            ("detections", "/detections"),
            ("tf", "/apriltag_node/tf_unused"),  # C++ 노드의 왜곡된 TF 발행 2중 차단
        ],
        output="screen",
    )

    # 3. 6-DoF 위치 추정 및 AR 오버레이 노드 (Python apriltag_6dof)
    tag_fusion_node = Node(
        package="apriltag_6dof",
        executable="tag_fusion_node",
        name="tag_fusion_node",
        parameters=[
            fusion_config,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 6. RViz2 시각화 자동 기동 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            robot_state_publisher_node,
            apriltag_node,
            tag_fusion_node,
            rviz_node,
        ]
    )
