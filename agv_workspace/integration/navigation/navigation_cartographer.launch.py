"""Cartographer localization + Nav2 navigation.

Map is served by nav2_map_server; Cartographer runs in a separate container and
publishes map -> odom. AMCL is not used.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    declare = [
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("map", default_value=""),
    ]
    remap = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params, {"yaml_filename": map_yaml}],
            remappings=remap,
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params],
            remappings=remap,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params],
            remappings=remap,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params],
            remappings=remap,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params],
            remappings=remap,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[params],
            remappings=remap,
        ),
        ExecuteProcess(
            cmd=["python3", "-B", str(Path(__file__).parent / "twist_stamp_relay.py")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", "-B", str(Path(__file__).parent / "tracked_pose_relay.py")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["python3", "-B", str(Path(__file__).parent / "charge_dock_node.py")],
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                params,
                {
                    "autostart": True,
                    "node_names": [
                        "map_server",
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "velocity_smoother",
                    ],
                },
            ],
            remappings=remap,
        ),
    ]
    return LaunchDescription(declare + nodes)
