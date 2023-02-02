import yaml
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

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
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
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
    servo_params = {"moveit_servo": servo_yaml, "moveit_servo.use_gazebo": use_sim}

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

    joy_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="joy_servo_node.py",
        name="joy_servo_node",
        parameters=[joy_servo_config],
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    start_moveit_servo_node = Node(
        package="rosbot_xl_manipulation_moveit",
        executable="start_moveit_servo_node.py",
        name="start_moveit_servo_node",
    )

    actions = [
        declare_use_sim_arg,
        declare_servo_joy_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        servo_node,
        joy_servo_node,
        joy_node,
        start_moveit_servo_node,
    ]

    return LaunchDescription(actions)
