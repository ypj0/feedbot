from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [DeclareLaunchArgument("map_yaml")]
    config = str(Path(__file__).parent / "config" / "localization.yaml")
    return LaunchDescription(arguments + [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[config, {"yaml_filename": LaunchConfiguration("map_yaml")}],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=[config],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            parameters=[config],
            output="screen",
        ),
    ])
