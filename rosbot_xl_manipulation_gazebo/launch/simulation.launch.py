#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    launch_joy_node = LaunchConfiguration("launch_joy_node")
    declare_launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="False",
    )

    map_package = get_package_share_directory("husarion_office_gz")
    world_file = PathJoinSubstitution([map_package, "worlds", "husarion_world.sdf"])
    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world", default_value=["-r ", world_file], description="SDF world file"
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    controller_config_name = PythonExpression(
        [
            "'mecanum_drive_controller_manipulation.yaml' if ",
            mecanum,
            " else 'diff_drive_controller_manipulation.yaml'",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_manipulation_controller"),
            "config",
            controller_config_name,
        ]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_xl_manipulation_description"),
                    "urdf",
                    "rosbot_xl_manipulation.urdf.xacro",
                ]
            ),
            " mecanum:=",
            mecanum,
            " use_sim:=True",
            " manipulator_collision_enabled:=False",
            " simulation_controllers_config_file:=",
            robot_controllers,
        ]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "rosbot_xl_manipulation",
            "-allow_renaming",
            "true",
            "-string",
            robot_description_content,
            "-x",
            "0",
            "-y",
            "2.0",
            "-z",
            "0.2",
        ],
        output="screen",
    )
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        ],
        output="screen",
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_manipulation_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "mecanum": mecanum,
            "use_sim": "True",
            "launch_joy_node": launch_joy_node,
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_manipulation_moveit"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim": "True"}.items(),
    )

    return LaunchDescription(
        [
            declare_mecanum_arg,
            declare_launch_joy_node_arg,
            declare_world_arg,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            ign_bridge,
            gz_spawn_entity,
            bringup_launch,
            moveit_rviz_launch,
        ]
    )
