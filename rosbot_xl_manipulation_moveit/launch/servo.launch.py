import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    launch_joy_node = LaunchConfiguration("launch_joy_node")
    declare_launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="False",
    )

    joy_servo_config = LaunchConfiguration("joy_servo_params_file")
    declare_servo_joy_arg = DeclareLaunchArgument(
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
        description="Whether to use mecanum drive controller (otherwise diff drive controller is used)",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_xl_manipulation_moveit"
    ).to_moveit_configs()

    # Get parameters for the Servo node
    servo_yaml = load_yaml("rosbot_xl_manipulation_moveit", "config/servo.yaml")
    servo_params = {
        "moveit_servo": servo_yaml,
        "moveit_servo.use_gazebo": use_sim,
        # What to publish? Can save some bandwidth as most robots only require positions or velocities
        # In general velocity should be chosen, because it better integrates with setting manipulator back to Home position
        # if position publishing is used, last position, pre homing, will be once again published, which will cause
        # manipulator to move abruptly back to position pre homing
        # velocity publishing respects changing position of the manipulator from other source
        # In simulation it is necessary to publish position though - velocity causes manipulator to fall down at the start
        # (bug only present in simulation)
        "moveit_servo.publish_joint_positions": use_sim,
        "moveit_servo.publish_joint_velocities": PythonExpression(["not ", use_sim]),
        "moveit_servo.publish_joint_accelerations": False,
    }

    # Manually load description to include potential changes - moveit config builder will construct urdf
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
            " antenna_rotation_angle:=",
            antenna_rotation_angle,
            " mecanum:=",
            mecanum,
            " use_sim:=",
            use_sim,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            moveit_config.robot_description_semantic,
            # if inverse kinamtics isn't specified inverse Jacobian will be used
            # moveit_config.robot_description_kinematics
        ],
        output="screen",
    )

    joy_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="joy_servo_node",
        name="joy_servo_node",
        parameters=[joy_servo_config],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(launch_joy_node),
    )

    actions = [
        declare_launch_joy_node_arg,
        declare_servo_joy_arg,
        declare_joint1_limit_min_arg,
        declare_joint1_limit_max_arg,
        declare_antenna_rotation_angle_arg,
        declare_mecanum_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        joy_servo_node,
        joy_node,
    ]

    return LaunchDescription(actions)
