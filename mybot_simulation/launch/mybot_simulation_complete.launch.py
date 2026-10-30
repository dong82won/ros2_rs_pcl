from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_simulation = FindPackageShare("mybot_simulation")

    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value="my_world_exmaple.world",
            description="Target world file name",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="default",
            description="Internal Gazebo world entity name",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo Sim without GUI",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="mybot",
            description="Primary robot entity name",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="",
            description="Primary robot ROS namespace",
        ),
        DeclareLaunchArgument(
            "robot_package",
            default_value="mybot_description",
            description="Package defining primary robot description",
        ),
        DeclareLaunchArgument(
            "robot_xacro_relative_path",
            default_value="urdf/mbot_realsense/mybot3_cam.xacro",
            description="Path to robot Xacro relative to robot_package share",
        ),
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.1"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz2 for visualization",
        ),
        DeclareLaunchArgument(
            "rviz_config_pkg",
            default_value="mybot_description",
            description="Package containing RViz configuration",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value="rviz/urdf_vis.rviz",
            description="Relative path to RViz config file",
        ),
    ]

    start_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_simulation, "launch", "start_world_complete.launch.py"]
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_simulation, "launch", "spawn_robot_complete.launch.py"]
            )
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "robot_name": LaunchConfiguration("robot_name"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "robot_package": LaunchConfiguration("robot_package"),
            "robot_xacro_relative_path": LaunchConfiguration(
                "robot_xacro_relative_path"
            ),
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "z_pose": LaunchConfiguration("z_pose"),
            "yaw": LaunchConfiguration("yaw"),
            "use_sim_time": "true",
        }.items(),
    )

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(LaunchConfiguration("rviz_config_pkg")),
            LaunchConfiguration("rviz_config_file"),
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            start_world_launch,
            spawn_robot_launch,
            rviz_node,
        ]
    )
