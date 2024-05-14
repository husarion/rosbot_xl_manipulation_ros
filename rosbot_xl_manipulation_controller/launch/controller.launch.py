#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    controller_manager_name = "controller_manager"

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_xl_base_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of imu_broadcaster after robot_controller
    # when spawning without delay ros2_control_node sometimes crashed
    delay_imu_broadcaster_spawner_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        )
    )

    manipulator_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "manipulator_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    delay_manipulator_spawner_after_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_broadcaster_spawner,
            on_exit=[manipulator_spawner],
        )
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    delay_gripper_spawner_after_manipulator_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=manipulator_spawner,
            on_exit=[gripper_spawner],
        )
    )

    return [
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_imu_broadcaster_spawner_after_robot_controller_spawner,
        delay_manipulator_spawner_after_imu_broadcaster_spawner,
        delay_gripper_spawner_after_manipulator_spawner,
    ]


def generate_launch_description():
    manipulator_usb_port = LaunchConfiguration("manipulator_usb_port")
    declare_manipulator_usb_port_arg = DeclareLaunchArgument(
        "manipulator_usb_port",
        default_value="/dev/ttyUSB0",
    )

    manipulator_baud_rate = LaunchConfiguration("manipulator_baud_rate")
    declare_manipulator_baud_rate_arg = DeclareLaunchArgument(
        "manipulator_baud_rate",
        default_value="115200",
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

    mecanum = LaunchConfiguration("mecanum")
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="False",
        description="Whether to use mecanum drive controller, otherwise use diff drive",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    simulation_engine = LaunchConfiguration("simulation_engine")
    declare_simulation_engine_arg = DeclareLaunchArgument(
        "simulation_engine",
        default_value="ignition-gazebo",
        description="Which simulation engine will be used",
    )

    controller_config_name = PythonExpression(
        [
            "'mecanum_drive_controller_manipulation.yaml' if ",
            mecanum,
            " else 'diff_drive_controller_manipulation.yaml'",
        ]
    )

    # Get URDF via xacro
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
            " manipulator_usb_port:=",
            manipulator_usb_port,
            " manipulator_baud_rate:=",
            manipulator_baud_rate,
            " joint1_limit_min:=",
            joint1_limit_min,
            " joint1_limit_max:=",
            joint1_limit_max,
            " mecanum:=",
            mecanum,
            " use_sim:=",
            use_sim,
            " simulation_engine:=",
            simulation_engine,
            " namespace:=None",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_manipulation_controller"),
            "config",
            controller_config_name,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers,
        ],
        remappings=[
            ("imu_sensor_node/imu", "/_imu/data_raw"),
            ("~/motors_cmd", "/_motors_cmd"),
            ("~/motors_response", "/_motors_response"),
            ("rosbot_xl_base_controller/cmd_vel_unstamped", "cmd_vel"),
        ],
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            declare_manipulator_usb_port_arg,
            declare_manipulator_baud_rate_arg,
            declare_joint1_limit_min_arg,
            declare_joint1_limit_max_arg,
            declare_mecanum_arg,
            declare_use_sim_arg,
            declare_simulation_engine_arg,
            SetParameter(name="use_sim_time", value=use_sim),
            control_node,
            robot_state_pub_node,
            OpaqueFunction(function=launch_setup),
        ]
    )
