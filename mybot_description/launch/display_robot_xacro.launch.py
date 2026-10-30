from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ROS 2 Launch 표준 치환기 사용
    pkg_share = FindPackageShare("mybot_description")

    # 하위 폴더 경로 인자 개별 분리
    xacro_file = PathJoinSubstitution(
        [pkg_share, "urdf", "mbot_realsense", "mybot3_cam.xacro"]
    )
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, "rviz", "urdf_vis.rviz"]
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config_path,
        description="Absolute path to rviz config file",
    )

    # Xacro 파싱 Command 인자 분리
    robot_description_content = ParameterValue(
        Command(["xacro", " ", xacro_file]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
