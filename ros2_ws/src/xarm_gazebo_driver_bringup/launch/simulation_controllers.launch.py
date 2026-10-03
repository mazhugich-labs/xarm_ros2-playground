"""Activate the simulation controllers after Gazebo has spawned the robot."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                'joint_state_broadcaster', 'joint_position_controller',
                '--controller-manager', '/sim/controller_manager',
                '--controller-manager-timeout', '30',
            ],
        ),
    ])
