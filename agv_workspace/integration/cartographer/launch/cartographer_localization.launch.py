from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = str(Path(__file__).parent.parent / "config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "load_state_filename",
            description="Cartographer pbstream state to localize against",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            parameters=[{"use_sim_time": False}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "feedbot_2d_localization.lua",
                "-load_state_filename", LaunchConfiguration("load_state_filename"),
            ],
            remappings=[
                ("scan", "/scan"),
                ("odom", "/odometry/filtered"),
            ],
            output="screen",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            parameters=[
                {"use_sim_time": False},
                {"resolution": 0.05},
            ],
            output="screen",
        ),
    ])
