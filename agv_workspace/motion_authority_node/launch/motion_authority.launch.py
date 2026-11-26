from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feedbot_motion_authority_node',
            executable='motion_authority_node',
            name='motion_authority',
            output='screen',
            parameters=[{
                'feedbot_contract.timing.control_rate_hz': 50.0,
                'feedbot_contract.timing.lease_timeout_ms': 300.0,
                'feedbot_contract.timing.max_stamp_age_ms': 100.0,
                'feedbot_contract.timing.max_stamp_lead_ms': 50.0,
                'feedbot_contract.timing.status_rate_hz': 10,
                'feedbot_contract.timing.stats_period_s': 1.0,
                'feedbot_contract.limits.max_linear_mps': 0.5,
                'feedbot_contract.limits.max_angular_rps': 1.0,
                'feedbot_contract.topics.motion_request': '/motion/request',
                'feedbot_contract.topics.motion_status': '/motion/status',
                'feedbot_contract.topics.motion_perf': '/motion/perf',
                'feedbot_contract.topics.wheel_cmd_vel': '/wheel/cmd_vel',
            }],
        ),
    ])
