"""Start Gazebo, controllers, and the emulator in dependency order."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('xarm_gazebo_driver_bringup'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    timeout = LaunchConfiguration('startup_timeout')
    startup = {'ready': False}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_share / 'launch/gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'server_required': 'true',
            'gui_required': 'true',
            'pause': 'false',
        }.items(),
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/simulation_description.launch.py')),
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', '/sim/robot_description', '-entity', 'xarm7',
                   '-timeout', timeout],
    )
    controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'joint_position_controller',
                   '--controller-manager', '/sim/controller_manager',
                   '--controller-manager-timeout', timeout],
    )
    emulator = Node(
        package='xarm_controller_emulator', executable='xarm_controller_emulator',
        output='screen', parameters=[{
            'backend': 'gazebo',
            'feedback_timeout': ParameterValue(
                LaunchConfiguration('feedback_timeout'), value_type=float),
        }],
    )
    feedback = Node(
        package='xarm_gazebo_driver_bringup', executable='wait_for_feedback',
        output='screen', arguments=['--timeout', timeout],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('xarm_moveit_config'))
            / 'launch/xarm7_moveit_realmove.launch.py')),
        condition=IfCondition(LaunchConfiguration('launch_moveit')),
        launch_arguments={
            'robot_ip': '127.0.0.1',
            'prefix': '',
            'hw_ns': 'xarm',
            'add_gripper': 'false',
            'add_bio_gripper': 'false',
            'add_vacuum_gripper': 'false',
            'attach_to': 'world',
            'attach_xyz': '0 0 0',
            'attach_rpy': '0 0 0',
            'show_rviz': LaunchConfiguration('show_rviz'),
        }.items(),
    )

    def after_success(stage, next_actions):
        def exited(event, context):
            if context.is_shutdown:
                return []
            if event.returncode != 0:
                raise RuntimeError(f'{stage} failed with exit code {event.returncode}')
            return next_actions
        return exited

    def feedback_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError('Emulator feedback readiness check failed')
        startup['ready'] = True
        return [LogInfo(msg='SIMULATION_READY: fresh Gazebo feedback; emulator at 127.0.0.1'),
                moveit]

    def emulator_exited(event, context):
        if not context.is_shutdown:
            raise RuntimeError(f'Emulator exited with code {event.returncode}')

    def check_deadline(context):
        if not context.is_shutdown and not startup['ready']:
            raise RuntimeError('Simulation startup deadline exceeded')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds/xarm.world')),
        DeclareLaunchArgument('startup_timeout', default_value='60.0'),
        DeclareLaunchArgument('feedback_timeout', default_value='0.5'),
        DeclareLaunchArgument('launch_moveit', default_value='false'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=after_success('Robot spawn', [controllers]))),
        RegisterEventHandler(OnProcessExit(
            target_action=controllers,
            on_exit=after_success('Controller activation', [emulator, feedback]))),
        RegisterEventHandler(OnProcessExit(target_action=feedback, on_exit=feedback_ready)),
        RegisterEventHandler(OnProcessExit(target_action=emulator, on_exit=emulator_exited)),
        TimerAction(period=timeout, actions=[OpaqueFunction(function=check_deadline)]),
        gazebo, description, spawn,
    ])
