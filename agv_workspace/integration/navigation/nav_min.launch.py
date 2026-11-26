"""最小导航 launch：只启动第一版需要的规划/控制/行为/平滑节点。
不含 smoother/route/waypoint/collision_monitor/docking 等未使用节点。
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    declare = DeclareLaunchArgument('params_file', default_value='')
    nav2_container = 'nav2_container'

    return LaunchDescription([
        declare,
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[params], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen',
            parameters=[params], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            name='behavior_server', output='screen',
            parameters=[params], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[params],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                        ('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[params, {'autostart': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
