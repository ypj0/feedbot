from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def require_activation_success(event, unused_context):
    if event.returncode != 0:
        raise RuntimeError("slam_toolbox activation exited with %s" % event.returncode)
    return []


def generate_launch_description():
    activation = ExecuteProcess(
        cmd=["python3", "-B", str(Path(__file__).parent / "activate_mapping.py")],
        output="screen",
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py")
        ),
        launch_arguments={
            "autostart": "false",
            "use_lifecycle_manager": "false",
            "use_sim_time": "false",
            "slam_params_file": str(Path(__file__).parent / "config" / "slam_toolbox.yaml"),
        }.items(),
    )
    activation_exit = RegisterEventHandler(
        OnProcessExit(target_action=activation, on_exit=require_activation_success))
    return LaunchDescription([slam_launch, activation_exit, activation])
