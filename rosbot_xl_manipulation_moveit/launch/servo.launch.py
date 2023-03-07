import yaml
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode
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
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

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

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # if inverse kinamtics isn't specified inverse Jacobian will be used
            # moveit_config.robot_description_kinematics
        ],
        output="screen",
    )

    container_joy_servo = ComposableNodeContainer(
        name="joy_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="rosbot_xl_manipulation_moveit",
                plugin="rosbot_xl_manipulation::JoyServoNode",
                name="joy_servo_node",
                parameters=[joy_servo_config],
            ),
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(launch_joy_node),
    )

    start_moveit_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="start_moveit_servo_node.py",
        name="start_moveit_servo_node",
    )

    actions = [
        declare_use_sim_arg,
        declare_launch_joy_node_arg,
        declare_servo_joy_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        container_joy_servo,
        joy_node,
    ]

    return LaunchDescription(actions)
