#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_joy_node = LaunchConfiguration("launch_joy_node")
    declare_launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="False",
    )

    manipulator_usb_port = LaunchConfiguration("manipulator_usb_port")
    declare_manipulator_usb_port_arg = DeclareLaunchArgument(
        "manipulator_usb_port",
        default_value="/dev/ttyMANIPULATOR",
    )

    manipulator_baud_rate = LaunchConfiguration("manipulator_baud_rate")
    declare_manipulator_baud_rate_arg = DeclareLaunchArgument(
        "manipulator_baud_rate",
        default_value="1000000",
    )

    joy_servo_config = LaunchConfiguration("joy_servo_params_file")
    declare_joy_servo_config_arg = DeclareLaunchArgument(
        "joy_servo_params_file",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rosbot_xl_manipulation_moveit"),
                "config",
                "joy_servo.yaml",
            ]
        ),
        description="ROS2 parameters file to use with joy_servo node",
    )

    joint1_limit_min = LaunchConfiguration("joint1_limit_min")
    declare_joint1_limit_min_arg = DeclareLaunchArgument(
        "joint1_limit_min",
        default_value="-2.356",
        description="Min angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )
    joint1_limit_max = LaunchConfiguration("joint1_limit_max")
    declare_joint1_limit_max_arg = DeclareLaunchArgument(
        "joint1_limit_max",
        default_value="5.934",
        description="Max angle (in radians) that can be achieved by rotating joint1 of the manipulator",
    )

    antenna_rotation_angle = LaunchConfiguration("antenna_rotation_angle")
    declare_antenna_rotation_angle_arg = DeclareLaunchArgument(
        "antenna_rotation_angle",
        default_value="0.0",
        description="Angle (in radians) of the antenna. 0 angle means that antenna is in the default upward orientation",
    )

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description=(
            "Whether to use mecanum drive controller (otherwise diff drive controller"
            " is used)"
        ),
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_manipulation_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "manipulator_usb_port": manipulator_usb_port,
            "manipulator_baud_rate": manipulator_baud_rate,
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "antenna_rotation_angle": antenna_rotation_angle,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_manipulation_moveit"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "antenna_rotation_angle": antenna_rotation_angle,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_manipulation_moveit"),
                    "launch",
                    "servo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_joy_node": launch_joy_node,
            "joy_servo_params_file": joy_servo_config,
            "joint1_limit_min": joint1_limit_min,
            "joint1_limit_max": joint1_limit_max,
            "antenna_rotation_angle": antenna_rotation_angle,
            "mecanum": mecanum,
            "use_sim": use_sim,
        }.items(),
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [get_package_share_directory("rosbot_xl_bringup"), "config", "ekf.yaml"]
            )
        ],
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution(
                [
                    get_package_share_directory("rosbot_xl_bringup"),
                    "config",
                    "laser_filter.yaml",
                ]
            )
        ],
    )

    actions = [
        declare_joy_servo_config_arg,
        declare_manipulator_usb_port_arg,
        declare_manipulator_baud_rate_arg,
        declare_launch_joy_node_arg,
        declare_joint1_limit_min_arg,
        declare_joint1_limit_max_arg,
        declare_antenna_rotation_angle_arg,
        declare_mecanum_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        controller_launch,
        moveit_launch,
        servo_launch,
        robot_localization_node,
        laser_filter_node,
    ]

    return LaunchDescription(actions)
