"""Spawn in headless Gazebo while a global description publisher coexists."""

from contextlib import contextmanager, ExitStack
import os
import signal
import socket
import subprocess
import time

from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from gazebo_msgs.srv import SpawnEntity
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Empty, SetBool
from tf2_msgs.msg import TFMessage
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, GetFloat32List, MoveJoint, SetInt16, SetInt16ById
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


@pytest.mark.parametrize('with_emulator', [False, True])
def test_gazebo_namespace_and_description_source(tmp_path, with_emulator):
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
    samples = []
    node.create_subscription(
        JointState, '/sim/joint_state_broadcaster/joint_states', samples.append, 10,
    )
    commands_pub = node.create_publisher(
        Float64MultiArray, '/sim/joint_position_controller/commands', 10,
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

            spawner = stack.enter_context(child_process(
                ['ros2', 'launch', 'xarm_gazebo_driver_bringup',
                 'simulation_controllers.launch.py'], tmp_path / 'process_spawner.log', env,
            ))
            wait_for(node, lambda: spawner.poll() is not None, processes)
            assert spawner.returncode == 0
            controllers = node.create_client(
                ListControllers, '/sim/controller_manager/list_controllers',
            )
            wait_for(node, controllers.service_is_ready, processes)
            result = controllers.call_async(ListControllers.Request())
            wait_for(node, result.done, processes)
            loaded = {item.name: item for item in result.result().controller}
            assert set(loaded) == {'joint_state_broadcaster', 'joint_position_controller'}
            assert all(item.state == 'active' for item in loaded.values())
            assert set(loaded['joint_position_controller'].claimed_interfaces) == {
                f'joint{i}/position' for i in range(1, 8)
            }
            assert not loaded['joint_state_broadcaster'].claimed_interfaces

            parameters = node.create_client(
                GetParameters, '/sim/controller_manager/get_parameters',
            )
            wait_for(node, parameters.service_is_ready, processes)
            result = parameters.call_async(GetParameters.Request(names=['use_sim_time']))
            wait_for(node, result.done, processes)
            assert result.result().values[0].bool_value
            wait_for(node, lambda: len(samples) >= 3, processes)
            assert samples[-1].header.stamp != samples[0].header.stamp
            wait_for(node, lambda: commands_pub.get_subscription_count() == 1, processes)
            for target in ([0.1, -0.1, 0.05, 0.25, -0.05, 0.1, -0.1], [0.0] * 7):
                samples.clear()
                commands_pub.publish(Float64MultiArray(data=target))

                def reached_target():
                    if not samples:
                        return False
                    measured = dict(zip(samples[-1].name, samples[-1].position))
                    return all(abs(measured.get(f'joint{i}', float('inf')) - value) < 0.005
                               for i, value in enumerate(target, 1))

                wait_for(node, reached_target, processes)
            wait_for(node, lambda: bool(transforms), processes)
            assert {tf.child_frame_id for msg in transforms for tf in msg.transforms} == {
                f'link{i}' for i in range(1, 8)
            }
            for topic in ('/tf', '/tf_static', '/robot_description'):
                assert all(endpoint.node_namespace != '/sim'
                           for endpoint in node.get_publishers_info_by_topic(topic))
            assert node.count_publishers('/joint_states') == 0
            assert node.count_subscribers('/joint_states') == 1  # Global publisher only.
            if with_emulator:
                node.destroy_publisher(commands_pub)
                verify_emulator_and_driver(node, processes, stack, tmp_path, env, samples)
    except Exception:
        for log in sorted(tmp_path.glob('process_*.log')):
            print(f'\n{log.name}:\n{log.read_text()}')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


def verify_emulator_and_driver(node, processes, stack, tmp_path, env, samples,
                               start_emulator=True):
    """Run real driver commands, C54 recovery, and a paused-physics watchdog."""
    readiness, reports = [], []
    node.create_subscription(Bool, '/xarm_controller_emulator/feedback_ready',
                             readiness.append, 1)
    node.create_subscription(RobotMsg, '/xarm/robot_states', reports.append, 10)
    if start_emulator:
        processes.append(stack.enter_context(child_process(
            ['ros2', 'run', 'xarm_controller_emulator', 'xarm_controller_emulator',
             '--ros-args', '-p', 'backend:=gazebo'], tmp_path / 'process_emulator.log', env,
        )))
    wait_for(node, lambda: readiness and readiness[-1].data, processes)
    processes.append(stack.enter_context(child_process(
        ['ros2', 'launch', 'xarm_api', 'xarm7_driver.launch.py',
         'robot_ip:=127.0.0.1'], tmp_path / 'process_driver.log', env,
    )))

    def invoke(service_type, name, **fields):
        client = node.create_client(service_type, name)
        try:
            wait_for(node, client.service_is_ready, processes)
            result = client.call_async(service_type.Request(**fields))
            wait_for(node, result.done, processes)
            return result.result()
        finally:
            node.destroy_client(client)

    def ready():
        assert invoke(SetInt16ById, '/xarm/motion_enable', id=8, data=1).ret == 0
        assert invoke(SetInt16, '/xarm/set_mode', data=1).ret == 0
        assert invoke(SetInt16, '/xarm/set_state', data=0).ret == 0
        wait_for(node, lambda: reports and reports[-1].err == 0
                 and reports[-1].mode == 1 and reports[-1].state in (1, 2), processes)

    def move(target):
        return invoke(MoveJoint, '/xarm/set_servo_angle_j',
                      angles=[target] + [0.0] * 6).ret

    def measured():
        return dict(zip(samples[-1].name, samples[-1].position))['joint1']

    def check_query_matches_feedback(target):
        wait_for(node, lambda: abs(measured() - target) < 0.005, processes)
        reply = invoke(GetFloat32List, '/xarm/get_servo_angle')
        assert reply.ret == 0
        assert abs(reply.datas[0] - measured()) < 0.005

    ready()
    assert move(0.15) == 0
    check_query_matches_feedback(0.15)
    assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=True).success
    wait_for(node, lambda: reports[-1].err == 54 and reports[-1].state == 4, processes)
    assert move(0.3) != 0
    assert invoke(Call, '/xarm/clean_error').ret != 0
    assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=False).success
    count = len(reports)
    wait_for(node, lambda: len(reports) >= count + 2, processes)
    assert reports[-1].err == 54
    assert invoke(Call, '/xarm/clean_error').ret == 0
    wait_for(node, lambda: reports[-1].err == 0, processes)
    assert reports[-1].state == 5
    assert move(0.3) != 0
    ready()
    check_query_matches_feedback(0.15)
    assert move(-0.1) == 0
    check_query_matches_feedback(-0.1)

    invoke(Empty, '/pause_physics')
    wait_for(node, lambda: readiness and not readiness[-1].data, processes)
    wait_for(node, lambda: reports[-1].state == 4 and reports[-1].mode == 0, processes)
    assert move(0.3) != 0
    check_query_matches_feedback(-0.1)
    invoke(Empty, '/unpause_physics')
    wait_for(node, lambda: readiness[-1].data, processes)
    assert move(0.3) != 0  # Fresh feedback alone must not re-enable motion.
    ready()
    check_query_matches_feedback(-0.1)
    assert move(0.0) == 0
    check_query_matches_feedback(0.0)
