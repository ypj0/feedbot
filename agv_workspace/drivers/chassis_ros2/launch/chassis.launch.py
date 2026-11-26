import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def controller_exited(event, context):
    if event.returncode != 0:
        raise RuntimeError(f"controller_manager exited with code {event.returncode}")
    return [EmitEvent(event=Shutdown(reason="controller_manager exited"))]


def start(context):
    package = Path(get_package_share_directory("feedbot_chassis_ros2"))
    calibration_path = Path(LaunchConfiguration("covariance_file").perform(context))
    calibration = yaml.safe_load(calibration_path.read_text())["wheel"]["ros__parameters"]
    for name in ("twist_covariance_diagonal", "pose_covariance_diagonal"):
        values = calibration[name]
        if len(values) != 6 or any(not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError(f"{name} requires six finite positive measured/modelled variances")
    description = (package / "config/chassis.urdf").read_text()
    controller = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[str(package / "config/controllers.yaml")],
        output="screen",
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=controller,
            on_exit=controller_exited,
        )),
        controller,
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": False}],
            output="screen",
        ),
        Node(
            package="controller_manager", executable="spawner",
            arguments=["joint_state_broadcaster", "wheel", "--param-file", str(package / "config/controllers.yaml"),
                       "--param-file", str(calibration_path)],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("covariance_file", description="Measured covariance YAML, with evidence provenance"),
        OpaqueFunction(function=start),
    ])
