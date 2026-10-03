"""Exercise real MoveIt execution, faults, and whole-stack restart recovery."""

from contextlib import contextmanager
import os
from pathlib import Path
import re
import signal
import socket
import time

from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, GetFloat32List, SetInt16, SetInt16ById

from test_simulation_bringup import environment  # noqa: F401 (pytest fixture)
from test_simulation_namespace import child_process, wait_for


def moveit_command(*arguments):
    return ['ros2', 'launch', 'xarm_gazebo_driver_bringup', 'moveit.launch.py',
            'gui:=false', 'show_rviz:=false', *arguments]


class MoveItClient:
    """Keep service/action checks identical across fault and restart scenarios."""

    def __init__(self, node, launch):
        self.node = node
        self.launch = launch
        self.measured, self.driver_states, self.moveit_states = [], [], []
        self.reports, self.readiness = [], []
        for topic, samples in (
            ('/sim/joint_state_broadcaster/joint_states', self.measured),
            ('/xarm/joint_states', self.driver_states), ('/joint_states', self.moveit_states),
        ):
            node.create_subscription(JointState, topic, samples.append, 10)
        node.create_subscription(RobotMsg, '/xarm/robot_states', self.reports.append, 10)
        node.create_subscription(Bool, '/xarm_controller_emulator/feedback_ready',
                                 self.readiness.append, 1)
        self.action = ActionClient(node, MoveGroup, '/move_action')

    def wait(self, predicate, timeout=30):
        wait_for(self.node, predicate, [self.launch], timeout=timeout)

    def result(self, future, timeout=30):
        self.wait(future.done, timeout)
        return future.result()

    def invoke(self, service_type, name, **fields):
        client = self.node.create_client(service_type, name)
        try:
            self.wait(client.service_is_ready)
            return self.result(client.call_async(service_type.Request(**fields)))
        finally:
            self.node.destroy_client(client)

    def controllers(self, namespace=''):
        reply = self.invoke(ListControllers, f'{namespace}/controller_manager/list_controllers')
        return {item.name: item for item in reply.controller}

    def active(self):
        controller = self.controllers().get('xarm7_traj_controller')
        return controller and controller.state == 'active'

    @staticmethod
    def positions(samples):
        return dict(zip(samples[-1].name, samples[-1].position))

    def joint1(self):
        return self.positions(self.measured)['joint1']

    def compare_feedback(self, target):
        def matches():
            return all(abs(self.positions(samples).get(f'joint{i}', float('inf'))
                           - (target if i == 1 else 0.0)) < 0.01
                       for samples in (self.measured, self.driver_states, self.moveit_states)
                       for i in range(1, 8))
        self.wait(matches)
        reply = self.invoke(GetFloat32List, '/xarm/get_servo_angle')
        assert reply.ret == 0
        assert all(abs(reply.datas[i - 1] - self.positions(self.measured)[f'joint{i}']) < 0.01
                   for i in range(1, 8))

    def goal(self, target, scaling=0.2):
        request = MoveGroup.Goal()
        request.request.group_name = 'xarm7'
        request.request.num_planning_attempts = 3
        request.request.allowed_planning_time = 5.0
        request.request.max_velocity_scaling_factor = scaling
        request.request.max_acceleration_scaling_factor = scaling
        request.request.start_state.is_diff = True
        request.planning_options.planning_scene_diff.is_diff = True
        request.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=f'joint{i}', position=target if i == 1 else 0.0,
                            tolerance_above=0.005, tolerance_below=0.005, weight=1.0)
            for i in range(1, 8)
        ])]
        handle = self.result(self.action.send_goal_async(request))
        assert handle.accepted
        return handle.get_result_async()

    def execute(self, target):
        future = self.goal(target)
        self.wait(future.done)
        # Check already-received physical feedback at completion, before waiting
        # for further convergence. A commanded target alone is insufficient.
        assert abs(self.joint1() - target) < 0.01
        response = future.result()
        assert response.status == 4
        assert response.result.error_code.val == 1
        assert response.result.executed_trajectory.joint_trajectory.points
        self.compare_feedback(target)

    def start_slow_motion(self):
        future = self.goal(0.6, scaling=0.05)
        self.wait(lambda: self.joint1() > 0.02)
        assert self.joint1() < 0.3
        assert not future.done()
        return future

    def recover(self, stopped_position):
        assert self.invoke(SetInt16ById, '/xarm/motion_enable', id=8, data=1).ret == 0
        assert self.invoke(SetInt16, '/xarm/set_mode', data=1).ret == 0
        assert self.invoke(SetInt16, '/xarm/set_state', data=0).ret == 0
        self.wait(self.active)
        count = len(self.reports)
        self.wait(lambda: len(self.reports) >= count + 3)
        assert abs(self.joint1() - stopped_position) < 0.01
        self.execute(-0.1)
        self.execute(0.0)

    def check_graph(self):
        self.wait(self.active)
        self.wait(lambda: self.measured and self.driver_states and self.moveit_states
                  and self.reports and self.readiness and self.readiness[-1].data)
        self.wait(self.action.server_is_ready)
        root, sim = self.controllers(), self.controllers('/sim')
        assert set(root) == {'xarm7_traj_controller'}
        assert set(sim) == {'joint_state_broadcaster', 'joint_position_controller'}
        assert all(item.state == 'active' for item in [*root.values(), *sim.values()])
        expected = {f'joint{i}/position' for i in range(1, 8)}
        root_expected = expected | {f'joint{i}/velocity' for i in range(1, 8)}
        assert set(root['xarm7_traj_controller'].claimed_interfaces) == root_expected
        assert set(sim['joint_position_controller'].claimed_interfaces) == expected
        assert not sim['joint_state_broadcaster'].claimed_interfaces
        # Names coincide, but each claim belongs to a distinct manager/hardware.
        for namespace, claims in (('', root_expected), ('/sim', expected)):
            interfaces = self.invoke(
                ListHardwareInterfaces, f'{namespace}/controller_manager/list_hardware_interfaces')
            assert {item.name for item in interfaces.command_interfaces if item.is_claimed} == claims
        nodes = self.node.get_node_names_and_namespaces()
        assert sorted(namespace for name, namespace in nodes if name == 'controller_manager') == [
            '/', '/sim',
        ]
        assert sum(name == 'move_group' for name, namespace in nodes) == 1
        for name, expected_clock in (('/controller_manager', False), ('/move_group', False),
                                     ('/sim/controller_manager', True)):
            values = self.invoke(GetParameters, name + '/get_parameters', names=['use_sim_time'])
            assert values.values[0].bool_value == expected_clock
        description = self.invoke(GetParameters, '/controller_manager/get_parameters',
                                  names=['robot_description']).values[0].string_value
        assert 'uf_robot_hardware/UFRobotSystemHardware' in description
        assert 'gazebo_ros2_control/GazeboSystem' not in description


@contextmanager
def running_moveit(log, environment):
    rclpy.init()
    node = rclpy.create_node('gazebo_moveit_test')
    client = None
    try:
        with child_process(moveit_command(), log, environment) as launch:
            client = MoveItClient(node, launch)
            client.check_graph()
            output = log.read_text()
            assert output.index('SIMULATION_READY') < output.index('[ros2_control_node-')
            yield client
    except Exception:
        print(log.read_text())
        raise
    finally:
        if client is not None:
            client.action.destroy()
        node.destroy_node()
        rclpy.shutdown()


def test_moveit_does_not_start_before_simulation(tmp_path, environment):
    log = tmp_path / 'failed_moveit.log'
    with child_process(moveit_command('startup_timeout:=0.1'), log, environment) as launch:
        launch.wait(timeout=20)
        assert launch.returncode != 0, log.read_text()
        assert 'Simulation startup deadline exceeded' in log.read_text()
        assert 'move_group-' not in log.read_text()
        assert 'ros2_control_node-' not in log.read_text()


@pytest.mark.parametrize('fault', ['c54', 'pause', 'late_pause', 'stop', 'disable'])
def test_moveit_execution_and_fault_recovery(tmp_path, environment, fault):
    with running_moveit(tmp_path / 'moveit.log', environment) as client:
        client.execute(0.15)
        client.execute(0.0)
        if fault == 'late_pause':
            interrupted = client.goal(0.6)
            client.wait(lambda: client.joint1() > 0.55)
            assert client.joint1() < 0.58
            assert not interrupted.done()
        else:
            interrupted = client.start_slow_motion()
        if fault == 'c54':
            assert client.invoke(SetBool, '/xarm_controller_emulator/set_c54', data=True).success
            client.wait(lambda: client.reports[-1].err == 54)
        elif fault in ('pause', 'late_pause'):
            client.invoke(Empty, '/pause_physics')
            client.wait(lambda: not client.readiness[-1].data)
            client.wait(lambda: client.reports[-1].state == 4 and client.reports[-1].mode == 0)
        elif fault == 'stop':
            assert client.invoke(SetInt16, '/xarm/set_state', data=4).ret == 0
        else:
            assert client.invoke(SetInt16ById, '/xarm/motion_enable', id=8, data=0).ret == 0

        response = client.result(interrupted)
        assert response.status != 4
        assert response.result.error_code.val != 1
        client.wait(lambda: not client.active())
        stopped_position = client.joint1()
        assert stopped_position < (0.59 if fault == 'late_pause' else 0.55)
        client.compare_feedback(stopped_position)

        if fault == 'c54':
            assert client.invoke(Call, '/xarm/clean_error').ret != 0
            assert client.invoke(SetBool, '/xarm_controller_emulator/set_c54', data=False).success
            count = len(client.reports)
            client.wait(lambda: len(client.reports) >= count + 2)
            assert client.reports[-1].err == 54
            assert client.invoke(Call, '/xarm/clean_error').ret == 0
            client.wait(lambda: client.reports[-1].err == 0)
            assert client.reports[-1].state > 2
        elif fault in ('pause', 'late_pause'):
            stamp = client.measured[-1].header.stamp
            count = len(client.reports)
            client.wait(lambda: len(client.reports) >= count + 3)
            assert client.measured[-1].header.stamp == stamp
            assert abs(client.joint1() - stopped_position) < 0.001
            client.invoke(Empty, '/unpause_physics')
            client.wait(lambda: client.readiness[-1].data)
            client.wait(lambda: client.measured[-1].header.stamp != stamp)
            assert client.reports[-1].state > 2
        assert not client.active()
        client.recover(stopped_position)


def launched_pids(log):
    return {name: int(pid) for name, pid in re.findall(
        r'\[INFO\] \[([^]]+-\d+)\]: process started with pid \[(\d+)\]', log.read_text())}


def process_alive(pid):
    try:
        # An exited child can remain a zombie until its parent reaps it.
        return Path(f'/proc/{pid}/stat').read_text().split(') ')[1][0] != 'Z'
    except FileNotFoundError:
        return False


@pytest.mark.parametrize('component', ['xarm_controller_emulator', 'gzserver'])
def test_process_failure_and_full_restart(tmp_path, environment, component):
    log = tmp_path / 'failure.log'
    with running_moveit(log, environment) as client:
        interrupted = client.start_slow_motion()
        children = launched_pids(log)
        targets = [pid for name, pid in children.items() if name.startswith(component + '-')]
        assert len(targets) == 1
        os.kill(targets[0], signal.SIGKILL)
        # A lost action server need not return a result, but must never succeed.
        deadline = time.monotonic() + 25
        while client.launch.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(client.node, timeout_sec=0.1)
            if interrupted.done():
                response = interrupted.result()
                assert response.status != 4
                assert response.result.error_code.val != 1
        assert client.launch.poll() is not None, 'Stack did not shut down after component failure'
        assert all(not process_alive(pid) for pid in children.values()), log.read_text()
        if component == 'xarm_controller_emulator':
            assert client.launch.returncode != 0
            assert 'Emulator exited' in log.read_text()
        else:
            assert 'was required: shutting down launched system' in log.read_text()
        for port in (502, 30001, 30002):
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(('127.0.0.1', port))

    # New processes, same ROS domain and TCP/Gazebo ports. Old goals must not replay.
    with running_moveit(tmp_path / 'restarted.log', environment) as client:
        client.compare_feedback(0.0)
        count = len(client.reports)
        client.wait(lambda: len(client.reports) >= count + 3)
        client.compare_feedback(0.0)
        client.execute(-0.1)
        client.execute(0.0)
