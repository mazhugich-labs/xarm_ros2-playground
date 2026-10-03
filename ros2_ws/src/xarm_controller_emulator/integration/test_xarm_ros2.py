"""Opt-in tests using the real xArm driver and MoveIt hardware launch."""

from contextlib import contextmanager
import os
import signal
import socket
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get('XARM_ROS2_INTEGRATION') != '1',
    reason='requires sourced xarm_ros2/MoveIt packages and isolated loopback ports',
)


@contextmanager
def running(command, log_path):
    """Own a process group and retain its output for failed-test diagnosis."""
    with log_path.open('w') as output:
        process = subprocess.Popen(
            command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            yield process
        except Exception:
            print(log_path.read_text())
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
            # ROS launch children belong to this test's process group only.
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


@pytest.fixture
def emulator(tmp_path):
    """Start a fresh real emulator process, never attach to an existing server."""
    for port in (502, 30001, 30002):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', port))
    with running(
        [sys.executable, '-m', 'xarm_controller_emulator.main'],
        tmp_path / 'emulator.log',
    ) as process:
        deadline = time.monotonic() + 10
        for port in (502, 30001, 30002):
            while True:
                assert process.poll() is None, 'emulator exited at startup'
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=1):
                        break
                except ConnectionRefusedError:
                    assert time.monotonic() < deadline, 'emulator startup timeout'
                    time.sleep(0.05)
        yield process


@pytest.fixture
def node():
    """Create the independent ROS test client."""
    import rclpy
    rclpy.init()
    client = rclpy.create_node('emulator_integration_client')
    yield client
    client.destroy_node()
    rclpy.try_shutdown()


def wait_until(node, predicate, timeout=15):
    """Spin until a condition holds, with a bounded timeout."""
    import rclpy
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out waiting for ROS state'
        rclpy.spin_once(node, timeout_sec=0.1)


def result(node, future, timeout=15):
    """Wait for a ROS service or action response."""
    wait_until(node, future.done, timeout)
    return future.result()


def assert_protocol_connected(log_path):
    """Verify the actual SDK switched protocols without losing the connection."""
    log = log_path.read_text()
    assert 'change protocol identifier to 3' in log
    assert 'CONTROL: 1, REPORT: 1' in log
    assert 'Control Connection Failed' not in log
    assert 'socket read failed' not in log


@pytest.mark.parametrize('report_type', ['normal', 'rich'])
def test_standalone_xarm_driver(emulator, node, tmp_path, report_type):
    """Require repeated driver RPCs and a continuous seven-joint report stream."""
    from sensor_msgs.msg import JointState
    from xarm_msgs.srv import GetFloat32List

    log_path = tmp_path / 'driver.log'
    with running([
        'ros2', 'launch', 'xarm_api', 'xarm7_driver.launch.py',
        'robot_ip:=127.0.0.1', f'report_type:={report_type}',
    ], log_path) as driver:
        samples = []
        subscription = node.create_subscription(
            JointState, '/xarm/joint_states', samples.append, 10)
        client = node.create_client(GetFloat32List, '/xarm/get_servo_angle')
        assert client.wait_for_service(timeout_sec=15)
        for _ in range(3):
            reply = result(node, client.call_async(GetFloat32List.Request()))
            assert reply.ret == 0
            assert list(reply.datas) == [0.0] * 7
            count = len(samples)
            wait_until(node, lambda: len(samples) >= count + 5)
        assert len(samples[-1].position) == 7
        assert set(samples[-1].name) == {f'joint{i}' for i in range(1, 8)}
        assert driver.poll() is None
        assert emulator.poll() is None
        assert_protocol_connected(log_path)
        node.destroy_subscription(subscription)


def test_driver_c54_blocks_motion_and_recovers(emulator, node, tmp_path):
    """Inject C54 through ROS and recover through the actual driver and SDK."""
    from std_srvs.srv import SetBool
    from xarm_msgs.msg import RobotMsg
    from xarm_msgs.srv import Call, GetFloat32List, MoveJoint, SetInt16, SetInt16ById

    with running([
        'ros2', 'launch', 'xarm_api', 'xarm7_driver.launch.py',
        'robot_ip:=127.0.0.1',
    ], tmp_path / 'c54_driver.log') as driver:
        reports = []
        subscription = node.create_subscription(
            RobotMsg, '/xarm/robot_states', reports.append, 10)

        def invoke(service_type, name, **fields):
            client = node.create_client(service_type, name)
            try:
                assert client.wait_for_service(timeout_sec=15), name
                return result(node, client.call_async(service_type.Request(**fields)))
            finally:
                node.destroy_client(client)

        def angle():
            reply = invoke(GetFloat32List, '/xarm/get_servo_angle')
            assert reply.ret == 0
            return reply.datas[0]

        def ready():
            assert invoke(SetInt16ById, '/xarm/motion_enable', id=8, data=1).ret == 0
            assert invoke(SetInt16, '/xarm/set_mode', data=1).ret == 0
            assert invoke(SetInt16, '/xarm/set_state', data=0).ret == 0
            wait_until(node, lambda: reports and reports[-1].err == 0
                       and reports[-1].mode == 1 and reports[-1].state in (1, 2))

        def move(target):
            return invoke(MoveJoint, '/xarm/set_servo_angle_j',
                          angles=[target] + [0.0] * 6).ret

        ready()
        assert move(0.1) == 0
        assert abs(angle() - 0.1) < 0.001
        assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=True).success
        wait_until(node, lambda: reports[-1].err == 54 and reports[-1].state == 4)
        assert move(0.2) != 0
        assert abs(angle() - 0.1) < 0.001
        assert invoke(Call, '/xarm/clean_error').ret != 0
        assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=False).success
        count = len(reports)
        wait_until(node, lambda: len(reports) >= count + 2)
        assert reports[-1].err == 54  # Releasing the cause does not clear the latch.
        assert invoke(Call, '/xarm/clean_error').ret == 0
        wait_until(node, lambda: reports[-1].err == 0)
        assert reports[-1].state == 5  # CLEAN_ERR resets readiness (configuration changed).
        assert move(0.2) != 0  # Clearing alone must not restore motion readiness.
        ready()
        assert abs(angle() - 0.1) < 0.001  # Rejected targets never replay on recovery.
        assert move(-0.1) == 0
        assert abs(angle() + 0.1) < 0.001
        assert driver.poll() is None
        assert emulator.poll() is None
        node.destroy_subscription(subscription)


def test_moveit_plans_and_executes_on_real_hardware_plugin(emulator, node, tmp_path):
    """Execute MoveGroup goals through ros2_control, the SDK, and the emulator."""
    from controller_manager_msgs.srv import ListControllers
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState

    log_path = tmp_path / 'moveit.log'
    with running([
        'ros2', 'launch', 'xarm_moveit_config', 'xarm7_moveit_realmove.launch.py',
        'robot_ip:=127.0.0.1', 'show_rviz:=false',
    ], log_path) as moveit:
        samples = []
        subscription = node.create_subscription(
            JointState, '/joint_states',
            lambda msg: samples.append(dict(zip(msg.name, msg.position))), 10)
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=20)

        def controller_active():
            reply = result(node, controllers.call_async(ListControllers.Request()))
            return any(c.name == 'xarm7_traj_controller' and c.state == 'active'
                       for c in reply.controller)

        wait_until(node, controller_active, timeout=20)
        wait_until(node, lambda: bool(samples))
        action = ActionClient(node, MoveGroup, '/move_action')
        assert action.wait_for_server(timeout_sec=15)
        for target in (0.15, 0.0):
            goal = MoveGroup.Goal()
            goal.request.group_name = 'xarm7'
            goal.request.num_planning_attempts = 3
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.2
            goal.request.max_acceleration_scaling_factor = 0.2
            goal.request.start_state.is_diff = True
            goal.planning_options.plan_only = False
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.request.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(
                    joint_name=f'joint{i}', position=target if i == 1 else 0.0,
                    tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
                for i in range(1, 8)
            ])]
            handle = result(node, action.send_goal_async(goal))
            assert handle.accepted
            response = result(node, handle.get_result_async(), timeout=30)
            assert response.status == 4  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            assert response.result.error_code.val == 1  # MoveIt SUCCESS
            assert response.result.planned_trajectory.joint_trajectory.points
            assert response.result.executed_trajectory.joint_trajectory.points
            wait_until(node, lambda: all(
                abs(samples[-1].get(f'joint{i}', float('inf'))
                    - (target if i == 1 else 0.0)) < 0.01
                for i in range(1, 8)
            ))
        assert moveit.poll() is None
        assert emulator.poll() is None
        assert_protocol_connected(log_path)
        action.destroy()
        node.destroy_subscription(subscription)
