import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("realsense_apriltag_rgbd_fusion")
    apriltag_config = os.path.join(pkg_share, "config", "apriltag_params.yaml")
    fusion_config = os.path.join(pkg_share, "config", "fusion_params.yaml")
    rviz_config = os.path.join(pkg_share, "config", "apriltag_view.rviz")

    target_tag_id_arg = DeclareLaunchArgument(
        "target_tag_id", default_value="0", description="추적 대상 AprilTag 고유 ID"
    )

    # 1. RealSense 센서 드라이버
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="camera",
        namespace="camera",
        parameters=[
            {
                "camera_name": "camera",
                "initial_reset": True,
                "reconnect_timeout": 6.0,
                "wait_for_device_timeout": 10.0,
                # 스트림 프로파일 명세 (640x480 @ 30FPS)
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
                "enable_color": True,
                "enable_depth": True,
                "align_depth.enable": True,
                "depth_module.emitter_enabled": 1,
                "enable_sync": True,
                # 미사용 센서 완전 비활성화
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_accel": False,
                "enable_gyro": False,
                "depth_module.global_time_enabled": True,
                "rgb_camera.global_time_enabled": True,
                "pointcloud.enable": False,
                "publish_tf": True,
                # [핵심] image_transport 압축 토픽 생성 원천 차단 (raw 토픽만 허용)
                "camera.color.image_raw.enable_pub_plugins": ["image_transport/raw"],
                "camera.aligned_depth_to_color.image_raw.enable_pub_plugins": ["image_transport/raw"],
                "camera.depth.image_rect_raw.enable_pub_plugins": ["image_transport/raw"],
            }
        ],
        output="screen",
    )

    # 2. apriltag_ros C++ 노드
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[apriltag_config],
        remappings=[
            ("image_rect", "/camera/camera/color/image_raw"),
            ("camera_info", "/camera/camera/color/camera_info"),
            ("detections", "/detections"),
        ],
    )

    # 3. Python 기반 RGB-D Cross-Validation 융합 노드
    fusion_node = Node(
        package="realsense_apriltag_rgbd_fusion",
        executable="rgbd_tag_fusion_node",
        name="rgbd_tag_fusion_node",
        output="screen",
        parameters=[
            fusion_config,
            {"target_tag_id": LaunchConfiguration("target_tag_id")},
            # 융합 노드 자체 발행 이미지도 raw 플러그인만 허용
            {"annotated_image.enable_pub_plugins": ["image_transport/raw"]},
        ],
    )

    # 4. 사전 구성된 RViz2 노드 (자동 구동)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            target_tag_id_arg,
            realsense_node,
            apriltag_node,
            fusion_node,
            rviz_node,
        ]
    )
