"""Verify that external measurements, readiness, and fault holds stay separate."""

import struct
import time

import pytest
import rclpy
from sensor_msgs.msg import JointState

from xarm_controller_emulator.control_server import ControlServer
from xarm_controller_emulator.gazebo_backend import GazeboBackend
from xarm_controller_emulator.report_server import build_normal_report, build_rich_report
from xarm_controller_emulator.state import RobotState


@pytest.fixture
def backend():
    """Create a backend without spinning so feedback and watchdog ticks are explicit."""
    rclpy.init()
    node = rclpy.create_node('gazebo_backend_test')
    bridge = GazeboBackend(node, RobotState(), 0.5)
    yield bridge
    node.destroy_node()
    rclpy.shutdown()


def sample(position=0.1, stamp=1):
    """Build intentionally reordered feedback to verify mapping by joint name."""
    message = JointState()
    message.header.stamp.sec = stamp
    message.name = [f'joint{i}' for i in range(7, 0, -1)]
    message.position = [position * i for i in range(7, 0, -1)]
    return message


def enable(backend):
    """Restore ServoJ mode and readiness through the wire command handlers."""
    server = ControlServer(backend.state)
    server._handle_request(0x13, b'\x01')
    assert server._handle_request(0x0C, b'\x00')[0] == 0
    return server


def servo(server, target=0.2):
    """Submit a streamed joint target."""
    return server._handle_request(0x1D, struct.pack('<10f', *([target] * 7), 0, 0, 0))


def test_reports_and_queries_use_measured_positions(backend):
    """A successful ServoJ reply must not turn its target into a measurement."""
    server = ControlServer(backend.state)
    assert server._handle_request(0x0C, b'\x00')[0] & 0x10
    backend.receive(sample())
    server = enable(backend)
    assert servo(server)[0] == 0
    assert backend.state.joint_targets == pytest.approx([0.2] * 7)
    expected = [0.1 * i for i in range(1, 8)]
    assert struct.unpack('<7f', server._handle_request(0x2A, b'')[1]) == pytest.approx(expected)
    for build in (build_normal_report, build_rich_report):
        assert struct.unpack_from('<7f', build(backend.state), 7) == pytest.approx(expected)
    measured = sample(stamp=2)
    measured.position = [0.2] * 7
    backend.receive(measured)
    assert backend.state.joint_angles == pytest.approx([0.2] * 7)
    assert backend.state.state == 2
    assert server._handle_request(0x17, struct.pack('<10f', *([0.0] * 10)))[0] & 0x08


@pytest.mark.parametrize('invalid', ['missing', 'duplicate', 'nan', 'length'])
def test_invalid_feedback_does_not_enable_motion(backend, invalid):
    """Reject partial, ambiguous, and nonfinite measurements."""
    message = sample()
    if invalid == 'missing':
        message.name[-1] = 'unexpected_joint'
    elif invalid == 'duplicate':
        message.name[-1] = message.name[0]
    elif invalid == 'nan':
        message.position[-1] = float('nan')
    else:
        message.position.pop()
    backend.receive(message)
    assert not backend.state.feedback_fresh
    assert not backend.state.motion_ready


@pytest.mark.parametrize('trigger', ['watchdog', 'query', 'report', 'repeated_stamp'])
def test_stale_feedback_discards_targets(backend, trigger):
    """All request/report paths stop stale motion and recovery requires readiness."""
    backend.receive(sample())
    server = enable(backend)
    servo(server)
    backend.state.feedback_received_at = time.monotonic() - 2.0
    if trigger == 'watchdog':
        backend.update()
    elif trigger == 'query':
        server._handle_request(0x2A, b'')
    elif trigger == 'report':
        build_normal_report(backend.state)
    else:
        backend.receive(sample())
    assert backend.state.state == 4
    assert not backend.state.feedback_fresh
    assert backend.state.joint_targets == backend.state.joint_angles
    assert servo(server)[0] & 0x10
    backend.receive(sample(position=0.05, stamp=2))
    assert backend.state.feedback_fresh
    assert not backend.state.motion_ready
    assert backend.state.joint_targets == backend.state.joint_angles
    enable(backend)
    assert servo(server)[0] == 0


def test_reset_simulation_time_requires_recovery(backend):
    """A restarted world cannot resume the previous world's target."""
    backend.receive(sample(stamp=10))
    server = enable(backend)
    servo(server)
    backend.receive(sample(position=0.0, stamp=0))
    assert backend.state.joint_targets == [0.0] * 7
    assert backend.state.state == 4
    assert not backend.state.motion_ready


@pytest.mark.parametrize('command,params', [
    (0x0C, b'\x04'), (0x0C, b'\x03'), (0x0B, b'\x08\x00'), (0x13, b'\x00'),
])
def test_stops_and_mode_changes_hold_measurements(backend, command, params):
    """Discard pending targets on stop, pause, disable, and mode changes."""
    backend.receive(sample())
    server = enable(backend)
    servo(server)
    server._handle_request(command, params)
    assert backend.state.joint_targets == backend.state.joint_angles
    assert servo(server)[0] & 0x10


def test_c54_hold_and_explicit_recovery(backend):
    """Keep the fault latched and never replay the pre-fault target."""
    backend.receive(sample())
    server = enable(backend)
    servo(server)
    backend.state.set_c54(True)
    assert backend.state.joint_targets == backend.state.joint_angles
    assert servo(server)[0] & 0x40
    server._handle_request(0x10, b'')
    assert backend.state.error_code == 54
    backend.state.set_c54(False)
    assert backend.state.error_code == 54
    server._handle_request(0x10, b'')
    assert backend.state.error_code == 0
    assert not backend.state.motion_ready
    enable(backend)
    assert backend.state.joint_targets == backend.state.joint_angles
    assert servo(server)[0] == 0
