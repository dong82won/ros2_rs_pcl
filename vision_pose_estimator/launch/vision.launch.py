from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 automatically",
    )
    use_rviz = LaunchConfiguration("use_rviz")

    vision_share = Path(get_package_share_directory("vision_pose_estimator"))
    rs_desc_share = Path(get_package_share_directory("realsense2_description"))

    # 1. RealSense D435i 카메라 노드 직접 실행 (YAML 기반)
    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="",
        name="camera",
        output="screen",
        emulate_tty=True,
        parameters=[str(vision_share / "config" / "realsense_params.yaml")],
    )

    # 2. 카메라 URDF(Xacro) 파싱 및 /robot_description 발행
    xacro_file = rs_desc_share / "urdf" / "test_d435i_camera.urdf.xacro"
    robot_description_xml = subprocess.check_output(
        ["xacro", str(xacro_file)],
        text=True,
    ).replace("package://realsense2_description", f"file://{rs_desc_share}")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xml}],
    )

    # 3. AprilTag 검출 노드
    tags = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[str(vision_share / "config" / "apriltag_params.yaml")],
        remappings=[
            ("image_rect", "/camera/color/image_raw"),
            ("camera_info", "/camera/color/camera_info"),
            ("detections", "/apriltag/detections"),
        ],
    )

    # 5. OpenCV 실시간 영상 마킹 노드
    visualizer = Node(
        package="vision_pose_estimator",
        executable="tag_visualizer_node",
        name="tag_visualizer",
        output="screen",
    )

    # 6. RViz2 시각화 노드
    rviz_config_file = vision_share / "rviz" / "vision.rviz"
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config_file)],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        camera,
        robot_state_publisher_node,
        tags,
        visualizer,
        rviz_node,
    ])
