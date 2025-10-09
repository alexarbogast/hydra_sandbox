from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("hydra_sandbox"), "config", "hydra_sandbox.rviz"]
    )

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim", default_value="true", description="Should sim hardware be launched?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "exec",
            default_value="planning",
            description="Schedule execution type. One of 'control', 'planning'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Should RVIz be launched?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool",
            default_value="marker_tool",
            description="Tool: 'typhoon_extruder', 'marker_tool', or 'tool0'",
        )
    )

    # fmt: off
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch",
                "hydra_system_sim.launch.py",
            ])
        ),
        launch_arguments={
            "rviz": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("sim")),
    )

    # Visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hydra_bringup"),
                "launch",
                "hydra_visualization.launch.py",
            ])
        ),
        launch_arguments={
            "rviz_config_file": rviz_config,
        }.items(),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # Execution mode: planning
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("hydra_sandbox"),
                "launch",
                "cartesian_planning_servers.launch.py",
            ])
        ),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("exec"), "' == 'planning'"])
        ),
    )

    # Execution mode: control
    control_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("exec"), "' == 'control'"])
        ),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("hydra_sandbox"),
                        "launch",
                        "taskspace_controllers.launch.py",
                    ])
                ),
                condition=IfCondition(LaunchConfiguration("sim")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("hydra_sandbox"),
                        "launch",
                        "schedule_execution_servers.launch.py",
                    ])
                )
            ),
        ],
    )
    # fmt: on

    nodes_to_start = [
        rviz_launch,
        sim_launch,
        planning_launch,
        # control_group,
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
