"""Publish the xArm7 simulation description and isolated simulation TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = Path(get_package_share_directory('xarm_gazebo_driver_bringup'))
    robot_description = xacro.process_file(
        str(package_share / 'urdf' / 'xarm7_sim.urdf.xacro')
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='sim',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
            remappings=[
                ('joint_states', 'joint_state_broadcaster/joint_states'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
