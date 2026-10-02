"""Wire-level regressions for the supported control-box protocol subset."""

from contextlib import contextmanager
import socket
import struct
import threading

import pytest

from xarm_controller_emulator.control_server import ControlServer
from xarm_controller_emulator.report_server import (
    build_normal_report, build_rich_report, ReportServer,
)
from xarm_controller_emulator.state import RobotState


def receive_exact(sock, size):
    """Read one field without assuming TCP packet boundaries."""
    data = b''
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        assert chunk, 'server closed before completing the response'
        data += chunk
    return data


def request_frame(command, payload=b'', tid=1, protocol=2):
    """Encode an independent client request."""
    return struct.pack('>HHHB', tid, protocol, len(payload) + 1, command) + payload


def receive_reply(sock, command, tid=1, expected_protocol=2):
    """Validate response framing and return status and data."""
    transaction, protocol, length = struct.unpack('>HHH', receive_exact(sock, 6))
    assert (transaction, protocol) == (tid, expected_protocol)
    body = receive_exact(sock, length)
    assert body[0] == command
    return body[1], body[2:]


@contextmanager
def connection(state):
    """Run the production connection handler on a local stream socket."""
    server, client = socket.socketpair()
    client.settimeout(1)
    failures = []

    def serve():
        try:
            ControlServer(state)._client(server)
        except Exception as exc:
            failures.append(exc)

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    try:
        yield client
    finally:
        client.close()
        worker.join(2)
        assert not worker.is_alive()
        assert not failures


def exchange(state, command, payload=b''):
    """Execute a complete request through the real framing code."""
    with connection(state) as client:
        client.sendall(request_frame(command, payload))
        return receive_reply(client, command)


def motion_payload(target=1.0):
    """Use the manual's little-endian float encoding."""
    return struct.pack('<10f', target, 0, 0, 0, 0, 0, 0, 0.5, 1, 0)


@pytest.mark.parametrize('command,mode', [(0x17, 0), (0x1D, 1)])
def test_motion_decodes_little_endian(command, mode):
    """Validate internal state as well as readback to expose double swapping."""
    state = RobotState(mode=mode)
    status, payload = exchange(state, command, motion_payload())
    assert status == 0
    assert payload == (b'\x00\x00' if command == 0x17 else b'')
    assert state.joint_angles == [1.0, 0, 0, 0, 0, 0, 0]
    assert state.state == 2
    assert exchange(state, 0x2A)[1] == struct.pack('<7f', *state.joint_angles)


def test_tcp_pose_query():
    """A fresh controller returns exactly six Cartesian coordinates."""
    assert exchange(RobotState(), 0x29) == (0, bytes(24))


@pytest.mark.parametrize('builder,size', [
    (build_normal_report, 145), (build_rich_report, 245),
])
def test_report_layout(builder, size):
    """Decode each field independently at its documented byte offset."""
    state = RobotState(mode=1, state=3, cmdnum=0x1234)
    state.joint_angles = [float(i) for i in range(1, 8)]
    state.joint_torques = [float(i) for i in range(11, 18)]
    packet = builder(state)
    assert len(packet) == size
    assert struct.unpack_from('>I', packet)[0] == size
    assert packet[4] == 0x13
    assert struct.unpack_from('>H', packet, 5)[0] == 0x1234
    assert struct.unpack_from('<7f', packet, 7) == tuple(state.joint_angles)
    assert struct.unpack_from('<6f', packet, 35) == (0,) * 6
    assert struct.unpack_from('<7f', packet, 59) == tuple(state.joint_torques)
    assert packet[87:91] == bytes([0x7F, 0x7F, 0, 0])
    assert struct.unpack_from('<3f', packet, 133) == (0, 0, -1)
    if size == 245:
        assert packet[145:151] == bytes([7, 7, 0xAA, 0x55, 0, 0])
        assert struct.unpack_from('<f', packet, 181)[0] == 1000


@pytest.mark.parametrize('fields,expected', [
    ({'error_code': 54, 'state': 4}, 0x50),
    ({'warn_code': 12}, 0x20),
    ({'servo_enable': 0}, 0x10),
    ({'state': 3}, 0x10),
    ({'error_code': 54, 'warn_code': 12, 'state': 4}, 0x70),
])
def test_status_flags(fields, expected):
    """Report status bits on reads as well as writes."""
    state = RobotState(**fields)
    assert exchange(state, 0x0F) == (
        expected, bytes([state.error_code, state.warn_code]))


@pytest.mark.parametrize('command,mode', [(0x17, 0), (0x1D, 1)])
@pytest.mark.parametrize('fields', [
    {'state': 3}, {'state': 4}, {'state': 5}, {'servo_enable': 0},
    {'error_code': 54, 'c54_active': True},
])
def test_motion_interlocks(command, mode, fields):
    """Faults, pause, stop, reset and disabled motors prevent execution."""
    state = RobotState(mode=mode, **fields)
    status, _ = exchange(state, command, motion_payload())
    assert status & 0x10
    assert state.joint_angles == [0] * 7


@pytest.mark.parametrize('command,mode', [(0x17, 1), (0x1D, 0)])
def test_wrong_motion_mode(command, mode):
    """Reject a motion command that belongs to a different control mode."""
    state = RobotState(mode=mode)
    assert exchange(state, command, motion_payload())[0] & 0x10
    assert state.joint_angles == [0] * 7


@pytest.mark.parametrize('command,payload', [
    (0x13, b'\x01'), (0x0B, b'\x08\x01'), (0x10, b''),
])
def test_reset_commands(command, payload):
    """Reset operations stop execution and empty the command buffer."""
    state = RobotState(state=1, cmdnum=3)
    assert exchange(state, command, payload)[0] & 0x10
    assert state.state == 5
    assert state.cmdnum == 0
    assert exchange(state, 0x0C, b'\x00')[0] == 0
    assert state.state == 2


@pytest.mark.parametrize('check_type', [1, 2, 3])
def test_check_only_is_explicitly_unsupported(check_type):
    """Do not move or claim successful collision checks without a planner."""
    state = RobotState()
    status, _ = exchange(state, 0x17, motion_payload() + bytes([check_type]))
    assert status & 0x08
    assert state.joint_angles == [0] * 7


@pytest.mark.parametrize('command,payload', [
    (0x0C, b'\xff'), (0x0C, b'\x01'), (0x13, b'\xff'),
    (0x13, b'\x04'), (0x13, b'\x05'),
    (0x0B, b'\x08\x02'), (0x0B, b'\x00\x01'),
    (0x0D, b'\x00'), (0x10, b'\x00'), (0x13, b'\x01\x00'),
    (0x17, b''), (0x1D, motion_payload() + b'\x00'), (0xFF, b''),
    (0x17, motion_payload(float('nan'))),
    (0x1D, motion_payload(float('inf'))),
])
def test_invalid_commands_leave_state_unchanged(command, payload):
    """Reject malformed or unsupported requests without partial mutation."""
    state = RobotState()
    before = build_normal_report(state)
    assert exchange(state, command, payload)[0] & 0x08
    assert build_normal_report(state) == before


@pytest.mark.parametrize('header', [
    struct.pack('>HHH', 1, 2, 0),
    request_frame(0x0D, protocol=999),
])
def test_invalid_headers_close_cleanly(header):
    """Invalid protocol IDs and empty frames do not crash the handler."""
    with connection(RobotState()) as client:
        client.sendall(header)
        try:
            assert client.recv(1) == b''
        except ConnectionResetError:
            pass  # Closing with an unread invalid body may produce a TCP reset.


def test_fragmented_and_coalesced_requests():
    """Support partial reads and multiple commands in one TCP write."""
    with connection(RobotState()) as client:
        frame = request_frame(0x0D, tid=5)
        client.sendall(frame[:3])
        client.sendall(frame[3:] + request_frame(0x0F, tid=6))
        assert receive_reply(client, 0x0D, 5) == (0, b'\x02')
        assert receive_reply(client, 0x0F, 6) == (0, b'\x00\x00')


@pytest.mark.parametrize('builder', [build_normal_report, build_rich_report])
def test_report_respects_state_lock(builder):
    """Do not publish a report in the middle of a state transaction."""
    state = RobotState()
    started, finished = threading.Event(), threading.Event()

    def build():
        started.set()
        builder(state)
        finished.set()

    with state.lock:
        worker = threading.Thread(target=build, daemon=True)
        worker.start()
        assert started.wait(1)
        assert not finished.wait(0.05)
    worker.join(1)
    assert finished.is_set()


def test_c54_recovery_sequence():
    """Require condition release, error clearing, and readiness before motion."""
    state = RobotState(mode=1, cmdnum=3)
    state.set_c54(True)
    assert (state.state, state.mode, state.cmdnum) == (4, 0, 0)
    assert exchange(state, 0x10)[0] == 0x50
    assert exchange(state, 0x0C, b'\x00')[0] == 0x50
    assert state.error_code == 54
    state.set_c54(False)
    assert exchange(state, 0x0F) == (0x50, b'\x36\x00')
    assert exchange(state, 0x10) == (0x10, b'')
    assert exchange(state, 0x17, motion_payload())[0] == 0x10
    assert exchange(state, 0x0C, b'\x00') == (0, b'')
    assert exchange(state, 0x17, motion_payload()) == (0, b'\x00\x00')
    assert state.joint_angles[0] == 1


def test_warning_clear_and_motion():
    """Warnings are reported but do not prevent otherwise valid motion."""
    state = RobotState(warn_code=12)
    assert exchange(state, 0x17, motion_payload())[0] == 0x20
    assert state.joint_angles[0] == 1
    assert exchange(state, 0x11) == (0, b'')
    assert state.state == 2


@pytest.mark.parametrize('dof', [5, 6, 7])
def test_joint_slots_and_individual_enables(dof):
    """Preserve seven wire slots while masking nonexistent motors."""
    state = RobotState(dof=dof)
    target = struct.pack('<10f', *([1] * 7), 0.5, 1, 0)
    assert exchange(state, 0x17, target)[0] == 0
    assert state.joint_angles == [1] * dof + [0] * (7 - dof)
    assert state.servo_enable == (1 << dof) - 1
    assert build_rich_report(state)[145:147] == bytes([dof, dof])
    assert exchange(state, 0x0B, b'\x02\x00')[0] == 0x10
    assert state.servo_enable == ((1 << dof) - 1) & ~2
    assert exchange(state, 0x0C, b'\x00')[0] == 0x10
    assert exchange(state, 0x0B, b'\x02\x01')[0] == 0x10
    assert exchange(state, 0x0C, b'\x00')[0] == 0


def test_pose_data_and_bounded_version_field():
    """Keep nonzero pose data intact and never resize a rich report."""
    state = RobotState(firmware_version='x' * 40)
    state.tcp_pose = [207, 0.25, 112, 3.0, -0.5, 1.0]
    expected = struct.pack('<6f', *state.tcp_pose)
    assert exchange(state, 0x29) == (0, expected)
    packet = build_rich_report(state)
    assert packet[35:59] == expected
    assert packet[151:181] == b'x' * 30
    assert len(packet) == 245


def test_control_respects_state_lock():
    """A concurrent fault transaction must complete before executing motion."""
    state = RobotState()
    with connection(state) as client:
        with state.lock:
            client.sendall(request_frame(0x17, motion_payload()))
            client.settimeout(0.05)
            with pytest.raises(socket.timeout):
                client.recv(1)
            state.set_c54(True)
        client.settimeout(1)
        assert receive_reply(client, 0x17)[0] == 0x50
    assert state.joint_angles == [0] * 7


@pytest.mark.parametrize('partial', [b'\x00', request_frame(0x17)[:6]])
def test_partial_request_disconnect(partial):
    """EOF during either header or body is handled without an exception."""
    with connection(RobotState()) as client:
        client.sendall(partial)
        client.shutdown(socket.SHUT_WR)
        assert client.recv(1) == b''


@pytest.mark.parametrize('builder,size', [
    (build_normal_report, 145), (build_rich_report, 245),
])
def test_report_stream_observes_fault(builder, size):
    """Check live report framing and propagation of the shared fault state."""
    state = RobotState()
    server, client = socket.socketpair()
    client.settimeout(1)
    worker = threading.Thread(
        target=ReportServer(state, 0, builder, 'test')._client,
        args=(server,), daemon=True)
    worker.start()
    try:
        first = receive_exact(client, size)
        assert struct.unpack_from('>I', first)[0] == size
        assert first[89] == 0
        state.set_c54(True)
        fault = receive_exact(client, size)
        assert fault[4] == 4
        assert fault[89] == 54
    finally:
        client.close()
        worker.join(2)
        assert not worker.is_alive()


def test_sdk_protocol_switch_on_existing_connection():
    """The SDK switches to private protocol 3 after reading firmware >= 1.8.6."""
    with connection(RobotState()) as client:
        client.sendall(request_frame(0x01))
        assert receive_reply(client, 0x01) == (0, b'v1.8.102')
        for protocol in (3, 3, 2):
            client.sendall(request_frame(0x0D, protocol=protocol))
            assert receive_reply(client, 0x0D, expected_protocol=protocol) == (0, b'\x02')
