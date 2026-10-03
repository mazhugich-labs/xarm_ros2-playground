"""Start the real-driver MoveIt stack after Gazebo emulator readiness."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('xarm_gazebo_driver_bringup'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/simulation.launch.py')),
            launch_arguments={'launch_moveit': 'true'}.items(),
        ),
    ])
