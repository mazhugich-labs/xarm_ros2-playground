"""Exercise the installed startup sequence, readiness gate, and failed stages."""

from contextlib import ExitStack
import os
import socket
import subprocess

from controller_manager_msgs.srv import ListControllers
import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from test_simulation_namespace import child_process, verify_emulator_and_driver, wait_for


@pytest.fixture
def environment():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    return dict(os.environ, GAZEBO_MASTER_URI=f'http://127.0.0.1:{port}',
                GAZEBO_MODEL_DATABASE_URI='')


def launch_command(*arguments):
    return ['ros2', 'launch', 'xarm_gazebo_driver_bringup', 'simulation.launch.py',
            'gui:=false', *arguments]


def test_coordinated_startup_and_driver_recovery(tmp_path, environment):
    rclpy.init()
    node = rclpy.create_node('simulation_bringup_test')
    readiness, samples = [], []
    node.create_subscription(Bool, '/xarm_controller_emulator/feedback_ready',
                             readiness.append, 1)
    node.create_subscription(JointState, '/sim/joint_state_broadcaster/joint_states',
                             samples.append, 10)
    log = tmp_path / 'bringup.log'
    try:
        with ExitStack() as stack:
            launch = stack.enter_context(child_process(launch_command(), log, environment))
            processes = [launch]
            wait_for(node, lambda: readiness and readiness[-1].data and samples, processes)
            wait_for(node, lambda: 'SIMULATION_READY' in log.read_text(), processes)
            controllers = node.create_client(ListControllers, '/sim/controller_manager/list_controllers')
            assert controllers.service_is_ready()
            result = controllers.call_async(ListControllers.Request())
            wait_for(node, result.done, processes)
            assert {item.name: item.state for item in result.result().controller} == {
                'joint_state_broadcaster': 'active', 'joint_position_controller': 'active',
            }
            nodes = node.get_node_names_and_namespaces()
            assert ('robot_state_publisher', '/sim') in nodes
            assert ('controller_manager', '/sim') in nodes
            assert ('controller_manager', '/') not in nodes
            assert not any(name == 'move_group' for name, namespace in nodes)
            for port in (502, 30001, 30002):
                with socket.create_connection(('127.0.0.1', port), timeout=1):
                    pass
            verify_emulator_and_driver(node, processes, stack, tmp_path, environment,
                                       samples, start_emulator=False)
    except Exception:
        print(log.read_text())
        for child_log in tmp_path.glob('process_*.log'):
            print(child_log.read_text())
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('failure', ['spawn', 'deadline', 'emulator_port'])
def test_failed_stage_stops_launch(tmp_path, environment, failure):
    log = tmp_path / 'failure.log'
    arguments = []
    with ExitStack() as stack:
        if failure == 'spawn':
            world = tmp_path / 'duplicate.world'
            world.write_text('<sdf version="1.6"><world name="test">'
                             '<model name="xarm7"><static>true</static>'
                             '<link name="base"/></model></world></sdf>')
            arguments.append(f'world:={world}')
            expected = 'Robot spawn failed'
        elif failure == 'deadline':
            arguments.append('startup_timeout:=0.1')
            expected = 'Simulation startup deadline exceeded'
        else:
            occupied = stack.enter_context(socket.socket())
            occupied.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            occupied.bind(('127.0.0.1', 502))
            occupied.listen(1)
            expected = 'Emulator exited'
        process = stack.enter_context(child_process(launch_command(*arguments), log, environment))
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            pytest.fail(f'Failed startup did not exit:\n{log.read_text()}')
        assert process.returncode != 0, log.read_text()
        output = log.read_text()
        assert expected in output, output
        assert 'SIMULATION_READY' not in output, output
        if failure in ('spawn', 'deadline'):
            assert 'xArm emulator ROS interface ready' not in output


def test_feedback_gate_times_out(tmp_path, environment):
    log = tmp_path / 'feedback.log'
    with child_process(
        ['ros2', 'run', 'xarm_gazebo_driver_bringup', 'wait_for_feedback', '--timeout', '0.2'],
        log, environment,
    ) as process:
        process.wait(timeout=5)
        assert process.returncode != 0
        assert 'Timed out waiting for fresh emulator feedback' in log.read_text()
