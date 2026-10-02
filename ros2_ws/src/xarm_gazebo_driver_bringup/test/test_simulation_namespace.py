"""Spawn in headless Gazebo while a global description publisher coexists."""

from contextlib import contextmanager, ExitStack
import os
import signal
import socket
import subprocess
import time

from controller_manager_msgs.srv import ListHardwareInterfaces
from gazebo_msgs.srv import SpawnEntity
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
import yaml


@contextmanager
def child_process(command, log_path, env=None):
    with log_path.open('w') as log:
        process = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True, env=env,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def wait_for(node, predicate, processes, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        assert all(process.poll() is None for process in processes), 'Child process exited'
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    assert predicate(), 'Timed out waiting for simulation graph'


def test_gazebo_namespace_and_description_source(tmp_path):
    # A separate Gazebo master avoids addressing an existing user's world.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        gazebo_port = probe.getsockname()[1]
    env = dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{gazebo_port}')
    env['GAZEBO_MODEL_DATABASE_URI'] = ''
    # No downloaded models or rendering are needed to validate the robot plugin.
    world = tmp_path / 'empty.world'
    world.write_text('<sdf version="1.6"><world name="test">'
                     '<gravity>0 0 0</gravity></world></sdf>')
    global_params = tmp_path / 'global_description.yaml'
    global_params.write_text(yaml.safe_dump({
        '/robot_state_publisher': {'ros__parameters': {
            'robot_description': '<robot name="global_reference">'
                                 '<link name="global_only"/></robot>',
        }},
    }))

    rclpy.init()
    node = rclpy.create_node('simulation_namespace_test')
    descriptions, global_descriptions, transforms, static_transforms = [], [], [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/sim/robot_description', descriptions.append, latched)
    node.create_subscription(String, '/robot_description', global_descriptions.append, latched)
    node.create_subscription(TFMessage, '/sim/tf', transforms.append, 10)
    node.create_subscription(TFMessage, '/sim/tf_static', static_transforms.append, latched)
    feedback = node.create_publisher(
        JointState, '/sim/joint_state_broadcaster/joint_states', 10,
    )
    try:
        with ExitStack() as stack:
            commands = [
                ['gzserver', '--verbose', str(world),
                 '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                ['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                 '--ros-args', '--params-file', str(global_params)],
                ['ros2', 'launch', 'xarm_gazebo_driver_bringup',
                 'simulation_description.launch.py'],
            ]
            processes = [stack.enter_context(child_process(
                command, tmp_path / f'process_{index}.log', env,
            )) for index, command in enumerate(commands)]
            wait_for(node, lambda: descriptions and global_descriptions and static_transforms,
                     processes)
            assert 'global_only' in global_descriptions[-1].data
            assert 'global_only' not in descriptions[-1].data
            assert 'link_base' in {
                tf.child_frame_id for msg in static_transforms for tf in msg.transforms
            }

            spawn = node.create_client(SpawnEntity, '/spawn_entity')
            wait_for(node, spawn.service_is_ready, processes)
            request = SpawnEntity.Request()
            request.name = 'xarm7_namespace_test'
            request.xml = descriptions[-1].data
            request.initial_pose.orientation.w = 1.0
            result = spawn.call_async(request)
            wait_for(node, result.done, processes)
            assert result.result().success, result.result().status_message

            interfaces = node.create_client(
                ListHardwareInterfaces, '/sim/controller_manager/list_hardware_interfaces',
            )
            wait_for(node, interfaces.service_is_ready, processes)
            result = interfaces.call_async(ListHardwareInterfaces.Request())
            wait_for(node, result.done, processes)
            expected = {f'joint{i}/{kind}' for i in range(1, 8)
                        for kind in ('position', 'velocity')}
            assert {item.name for item in result.result().state_interfaces} == expected
            assert {item.name for item in result.result().command_interfaces} == expected
            assert '/controller_manager/list_hardware_interfaces' not in dict(
                node.get_service_names_and_types()
            )

            # Synthetic feedback tests description/TF routing, not physics or control.
            wait_for(node, lambda: feedback.get_subscription_count() == 1, processes)
            state = JointState()
            state.header.stamp.sec = 1
            state.name = [f'joint{i}' for i in range(1, 8)]
            state.position = [0.0] * 7
            feedback.publish(state)
            wait_for(node, lambda: bool(transforms), processes)
            assert {tf.child_frame_id for msg in transforms for tf in msg.transforms} == {
                f'link{i}' for i in range(1, 8)
            }
            for topic in ('/tf', '/tf_static', '/robot_description'):
                assert all(endpoint.node_namespace != '/sim'
                           for endpoint in node.get_publishers_info_by_topic(topic))
            assert node.count_publishers('/joint_states') == 0
            assert node.count_subscribers('/joint_states') == 1  # Global publisher only.
    except Exception:
        for log in sorted(tmp_path.glob('process_*.log')):
            print(f'\n{log.name}:\n{log.read_text()}')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()
