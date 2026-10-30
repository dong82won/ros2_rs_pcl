from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    declared_arguments = [
        DeclareLaunchArgument("world_name", default_value="default"),
        DeclareLaunchArgument("robot_name", default_value="mybot"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("robot_package", default_value="mybot_description"),

        DeclareLaunchArgument(
            "robot_xacro_relative_path",
            default_value="urdf/mbot_realsense/mybot3_cam.xacro",
        ),

        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        DeclareLaunchArgument("z_pose", default_value="0.2"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    robot_name = LaunchConfiguration("robot_name")
    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_package = LaunchConfiguration("robot_package")
    robot_xacro_path = LaunchConfiguration("robot_xacro_relative_path")

    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")

    yaw = LaunchConfiguration("yaw")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")

    xacro_full_path = PathJoinSubstitution(
        [FindPackageShare(robot_package), robot_xacro_path]
    )

    bridge_config_full_path = PathJoinSubstitution(
        [
            FindPackageShare("mybot_simulation"),
            "config",
            "ros_gz_bridge.yaml",
        ]
    )

    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_full_path]),
        value_type=str,
    )

    robot_spawn_group = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            Node(
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
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="ros_gz_spawner",
                output="screen",
                arguments=[
                    "-world",
                    world_name,
                    "-name",
                    robot_name,
                    "-topic",
                    "robot_description",
                    "-x",
                    x_pose,
                    "-y",
                    y_pose,
                    "-z",
                    z_pose,
                    "-Y",
                    yaw,
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_parameter_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": bridge_config_full_path,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )

    return LaunchDescription(declared_arguments + [robot_spawn_group])
