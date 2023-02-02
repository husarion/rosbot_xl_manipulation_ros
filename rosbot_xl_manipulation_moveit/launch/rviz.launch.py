from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    moveit_config = MoveItConfigsBuilder(
        "rosbot_xl", package_name="rosbot_xl_manipulation_moveit"
    ).to_moveit_configs()

    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("rosbot_xl_manipulation_moveit"),
            "config",
            "moveit.rviz",
        ]
    )

    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
    )

    actions = [
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        rviz_node,
    ]

    return LaunchDescription(actions)
