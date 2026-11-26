from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = str(Path(__file__).parent.parent / "config")
    return LaunchDescription([
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            parameters=[{"use_sim_time": False}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "feedbot_2d_mapping.lua",
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
        ExecuteProcess(
            cmd=["python3", "-B", str(Path(__file__).parent.parent / "cartographer_web_api.py")],
            output="screen",
        ),
    ])
