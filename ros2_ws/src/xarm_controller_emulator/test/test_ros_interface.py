"""Exercise the ROS fault-injection service through a real service client."""

import time

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from std_srvs.srv import SetBool

from xarm_controller_emulator import main as entrypoint
from xarm_controller_emulator.emulator_node import XArmEmulatorNode
from xarm_controller_emulator.state import RobotState


def test_c54_service():
    """Verify ROS requests latch and release the condition in shared state."""
    rclpy.init()
    state = RobotState(mode=1, cmdnum=3)
    node = XArmEmulatorNode(state)
    client_node = rclpy.create_node('emulator_test_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    client = client_node.create_client(SetBool, '/xarm_controller_emulator/set_c54')
    try:
        assert client.wait_for_service(timeout_sec=5)
        for active in (True, False):
            future = client.call_async(SetBool.Request(data=active))
            deadline = time.monotonic() + 5
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            assert future.done()
            assert future.result().success
            assert state.c54_active == active
            assert (state.error_code, state.state, state.mode, state.cmdnum) == (54, 4, 0, 0)
    finally:
        executor.shutdown()
        client_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('shutdown_exception', [KeyboardInterrupt, ExternalShutdownException])
def test_main_handles_already_shutdown_context(monkeypatch, shutdown_exception):
    """ROS signal handlers can shut down the context before spin returns."""
    monkeypatch.setattr(entrypoint.ControlServer, 'start', lambda self: None)
    monkeypatch.setattr(entrypoint.ReportServer, 'start', lambda self: None)

    def signal_shutdown(node):
        node.context.shutdown()
        raise shutdown_exception()

    monkeypatch.setattr(rclpy, 'spin', signal_shutdown)
    entrypoint.main()
    assert not rclpy.ok()
