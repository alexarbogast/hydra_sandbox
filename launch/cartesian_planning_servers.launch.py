from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    FindExecutable,
    Command,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("hydra_sandbox"), "config", "cartesian_planners.yaml"]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool",
            default_value="typhoon_extruder",
            description="The tool to use",
            choices=["typhoon_extruder", "tool0"],
        )
    )

    # fmt: off
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [FindPackageShare("hydra_description"), "urdf", "hydra.xacro"]
        ),
        " tool:=",
        LaunchConfiguration("tool"),
    ])
    # fmt: on

    rob1_server_node = Node(
        package="cartesian_planning_server",
        executable="cartesian_planning_server",
        name="cartesian_planning_server",
        namespace="rob1",
        output="screen",
        parameters=[config_file, {"robot_description": robot_description}],
    )
    rob2_server_node = Node(
        package="cartesian_planning_server",
        executable="cartesian_planning_server",
        name="cartesian_planning_server",
        namespace="rob2",
        output="screen",
        parameters=[config_file, {"robot_description": robot_description}],
    )
    rob3_server_node = Node(
        package="cartesian_planning_server",
        executable="cartesian_planning_server",
        name="cartesian_planning_server",
        namespace="rob3",
        output="screen",
        parameters=[config_file, {"robot_description": robot_description}],
    )

    nodes_to_start = [
        rob1_server_node,
        rob2_server_node,
        rob3_server_node,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
