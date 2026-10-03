"""Execute MoveIt trajectories and recover from C54 through measured Gazebo state."""

from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, GetFloat32List, SetInt16, SetInt16ById

from test_simulation_bringup import environment  # noqa: F401 (pytest fixture)
from test_simulation_namespace import child_process, wait_for


def moveit_command(*arguments):
    return ['ros2', 'launch', 'xarm_gazebo_driver_bringup', 'moveit.launch.py',
            'gui:=false', 'show_rviz:=false', *arguments]


def test_moveit_does_not_start_before_simulation(tmp_path, environment):
    log = tmp_path / 'failed_moveit.log'
    with child_process(moveit_command('startup_timeout:=0.1'), log, environment) as launch:
        launch.wait(timeout=20)
        assert launch.returncode != 0, log.read_text()
        assert 'Simulation startup deadline exceeded' in log.read_text()
        assert 'move_group-' not in log.read_text()
        assert 'ros2_control_node-' not in log.read_text()


def test_moveit_execution_and_c54_recovery(tmp_path, environment):
    rclpy.init()
    node = rclpy.create_node('gazebo_moveit_test')
    measured, driver_states, moveit_states, reports = [], [], [], []
    for topic, samples in (
        ('/sim/joint_state_broadcaster/joint_states', measured),
        ('/xarm/joint_states', driver_states), ('/joint_states', moveit_states),
    ):
        node.create_subscription(JointState, topic, samples.append, 10)
    node.create_subscription(RobotMsg, '/xarm/robot_states', reports.append, 10)
    action = ActionClient(node, MoveGroup, '/move_action')
    log = tmp_path / 'moveit.log'
    try:
        with child_process(moveit_command(), log, environment) as launch:
            processes = [launch]

            def result(future, timeout=30):
                wait_for(node, future.done, processes, timeout=timeout)
                return future.result()

            def invoke(service_type, name, **fields):
                client = node.create_client(service_type, name)
                try:
                    wait_for(node, client.service_is_ready, processes)
                    return result(client.call_async(service_type.Request(**fields)))
                finally:
                    node.destroy_client(client)

            def controllers(namespace=''):
                reply = invoke(ListControllers, f'{namespace}/controller_manager/list_controllers')
                return {item.name: item for item in reply.controller}

            def active():
                controller = controllers().get('xarm7_traj_controller')
                return controller and controller.state == 'active'

            wait_for(node, active, processes)
            wait_for(node, lambda: measured and driver_states and moveit_states and reports,
                     processes)
            wait_for(node, action.server_is_ready, processes)
            assert set(controllers()) == {'xarm7_traj_controller'}
            assert set(controllers('/sim')) == {'joint_state_broadcaster',
                                               'joint_position_controller'}
            nodes = node.get_node_names_and_namespaces()
            assert sorted(namespace for name, namespace in nodes if name == 'controller_manager') == [
                '/', '/sim',
            ]
            assert sum(name == 'move_group' for name, namespace in nodes) == 1
            for name, expected_clock in (('/controller_manager', False), ('/move_group', False),
                                         ('/sim/controller_manager', True)):
                values = invoke(GetParameters, name + '/get_parameters', names=['use_sim_time'])
                assert values.values[0].bool_value == expected_clock
            description = invoke(GetParameters, '/controller_manager/get_parameters',
                                 names=['robot_description']).values[0].string_value
            assert 'uf_robot_hardware/UFRobotSystemHardware' in description
            assert 'gazebo_ros2_control/GazeboSystem' not in description
            assert log.read_text().index('SIMULATION_READY') < log.read_text().index(
                '[ros2_control_node-')

            def positions(samples):
                return dict(zip(samples[-1].name, samples[-1].position))

            def joint1():
                return positions(measured)['joint1']

            def compare_feedback(target):
                def matches():
                    return all(abs(positions(samples).get(f'joint{i}', float('inf'))
                                   - (target if i == 1 else 0.0)) < 0.01
                               for samples in (measured, driver_states, moveit_states)
                               for i in range(1, 8))
                wait_for(node, matches, processes)
                reply = invoke(GetFloat32List, '/xarm/get_servo_angle')
                assert reply.ret == 0
                assert all(abs(reply.datas[i - 1] - positions(measured)[f'joint{i}']) < 0.01
                           for i in range(1, 8))

            def goal(target, scaling=0.2):
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
                handle = result(action.send_goal_async(request))
                assert handle.accepted
                return handle.get_result_async()

            def execute(target):
                response = result(goal(target))
                assert response.status == 4
                assert response.result.error_code.val == 1
                assert response.result.executed_trajectory.joint_trajectory.points
                compare_feedback(target)

            execute(0.15)
            execute(0.0)

            # Fault a slow trajectory only once Gazebo has actually begun moving.
            interrupted = goal(0.6, scaling=0.05)
            wait_for(node, lambda: joint1() > 0.02, processes)
            assert not interrupted.done()
            assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=True).success
            wait_for(node, lambda: reports[-1].err == 54, processes)
            response = result(interrupted)
            assert response.status != 4
            assert response.result.error_code.val != 1
            wait_for(node, lambda: not active(), processes)
            stopped_position = joint1()
            assert stopped_position < 0.55
            assert invoke(Call, '/xarm/clean_error').ret != 0
            assert invoke(SetBool, '/xarm_controller_emulator/set_c54', data=False).success
            count = len(reports)
            wait_for(node, lambda: len(reports) >= count + 2, processes)
            assert reports[-1].err == 54
            assert invoke(Call, '/xarm/clean_error').ret == 0
            wait_for(node, lambda: reports[-1].err == 0, processes)
            assert reports[-1].state > 2
            assert not active()

            assert invoke(SetInt16ById, '/xarm/motion_enable', id=8, data=1).ret == 0
            assert invoke(SetInt16, '/xarm/set_mode', data=1).ret == 0
            assert invoke(SetInt16, '/xarm/set_state', data=0).ret == 0
            wait_for(node, active, processes)
            count = len(reports)
            wait_for(node, lambda: len(reports) >= count + 3, processes)
            assert abs(joint1() - stopped_position) < 0.01  # No pre-fault trajectory replay.
            execute(-0.1)
            execute(0.0)
    except Exception:
        print(log.read_text())
        raise
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.shutdown()
