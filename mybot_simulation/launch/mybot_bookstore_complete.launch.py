"""
Master Integration Launch Script for AWS Bookstore World + MyBot Robot.
Architecture:
- Environment Domain: aws_robomaker_bookstore_world (gazebo_ws)
- Robot Spawning Domain: mybot_simulation (ros2_ws)
- Executes World -> Spawns Robot -> Maps Bridges -> Opens RViz2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # 1. 패키지 경로 탐색 (크로스 워크스페이스 체이닝)
    pkg_world_env = FindPackageShare("aws_robomaker_bookstore_world")
    pkg_simulation = FindPackageShare("mybot_simulation")
    pkg_description = FindPackageShare("mybot_description")

    # 2. 런치 아규먼트 선언 (마스터 제어판)
    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="bookstore.world",
            description="Target world file name inside bookstore package",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="bookstore",  # SDF 파일 내부의 <world name="???"> 와 반드시 일치시킬 것
            description="Internal Gazebo world entity name for spawn targeting",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo Sim server only (no OGRE2 GUI)",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="mybot",
            description="Primary robot entity name in Gazebo",
        ),
        DeclareLaunchArgument(
            "x_pose", default_value="0.0", description="Spawn X coordinate"
        ),
        DeclareLaunchArgument(
            "y_pose", default_value="0.0", description="Spawn Y coordinate"
        ),
        DeclareLaunchArgument(
            "z_pose", default_value="0.1", description="Spawn Z coordinate"
        ),
        DeclareLaunchArgument(
            "yaw", default_value="0.0", description="Spawn Yaw angle"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for sensor & TF visualization",
        ),
    ]

    # 3. 런타임 환경 구성 (Gazebo 엔진 + Bookstore 월드 실행)
    start_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_world_env, "launch", "bookstore_new.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
            "use_sim_time": "true",  # 월드에서 /clock 브리지를 전담하도록 강제
        }.items(),
    )

    # 4. 로봇 스폰 및 브리지 구성 (MyBot + ros_gz_bridge)
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_simulation, "launch", "spawn_robot_complete.launch.py"]
            )
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "robot_name": LaunchConfiguration("robot_name"),
            "robot_package": "mybot_description",
            "robot_xacro_relative_path": "urdf/mbot_realsense/mybot3_cam.xacro",
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "z_pose": LaunchConfiguration("z_pose"),
            "yaw": LaunchConfiguration("yaw"),
            "use_sim_time": "true",
        }.items(),
    )

    # 5. RViz2 시각화 클라이언트
    rviz_config_path = PathJoinSubstitution([pkg_description, "rviz", "urdf_vis.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # 6. 파이프라인 반환
    return LaunchDescription(
        declared_arguments
        + [
            start_world_launch,
            spawn_robot_launch,
            rviz_node,
        ]
    )
