from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
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
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Start robot in Gazebo simulation.",
    )

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_xl_manipulation_moveit"
    ).to_moveit_configs()

    # Overwrite description to include potential changes - moveit config builder will construct urdf
    # with default values
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
            " manipulator_collision_enabled:=True",
            " joint1_limit_min:=",
            joint1_limit_min,
            " joint1_limit_max:=",
            joint1_limit_max,
            " mecanum:=",
            mecanum,
            " use_sim:=",
            use_sim,
        ]
    )
    moveit_config.robot_description = {"robot_description": robot_description_content}

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "monitor_dynamics": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        # Set the display variable, in case OpenGL code is used internally
        # additional_env={"DISPLAY": ":1"},
    )

    actions = [
        declare_joint1_limit_min_arg,
        declare_joint1_limit_max_arg,
        declare_mecanum_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        move_group_node,
    ]
    return LaunchDescription(actions)
