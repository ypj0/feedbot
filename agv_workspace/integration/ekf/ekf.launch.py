from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            executable=str(Path(__file__).resolve().parents[3] /
                           "install/lib/feedbot_ekf/feedbot_ekf"), name="ekf_filter_node",
            parameters=[str(Path(__file__).parent / "config/ekf.yaml")],
            additional_env={"RMW_FASTRTPS_PUBLICATION_MODE": "ASYNCHRONOUS"},
            output="screen",
        ),
    ])
