from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("hydra_sandbox"), "config", "collision_geometry.yaml"]
    )

    node1 = Node(
        package="hydra_sandbox",
        executable="visualize_collision_geometry.py",
        namespace="rob1",
        name="collision_visualization",
        parameters=[config_file],
        output="screen",
    )

    node2 = Node(
        package="hydra_sandbox",
        executable="visualize_collision_geometry.py",
        namespace="rob2",
        name="collision_visualization",
        parameters=[config_file],
        output="screen",
    )

    node3 = Node(
        package="hydra_sandbox",
        executable="visualize_collision_geometry.py",
        namespace="rob3",
        name="collision_visualization",
        parameters=[config_file],
        output="screen",
    )

    return LaunchDescription([node1, node2, node3])
