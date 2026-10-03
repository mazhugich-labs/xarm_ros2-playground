"""Implement the supported subset of xArm private Modbus-TCP commands."""

import logging
import math
import socket
import struct
import threading

_logger = logging.getLogger(__name__)

GET_VERSION = 0x01
MOTION_EN = 0x0B
SET_STATE = 0x0C
GET_STATE = 0x0D
GET_CMDNUM = 0x0E
GET_ERROR = 0x0F
CLEAN_ERR = 0x10
CLEAN_WARN = 0x11
SET_MODE = 0x13
MOVE_JOINT = 0x17
MOVE_SERVOJ = 0x1D
GET_TCP_POSE = 0x29
GET_JOINT_POS = 0x2A
SERVO_DBMSG = 0x6A

# Validate the whole request before mutating controller state.
_PARAMETER_LENGTHS = {
    GET_VERSION: (0,), MOTION_EN: (2,), SET_STATE: (1,), GET_STATE: (0,),
    GET_CMDNUM: (0,), GET_ERROR: (0,), CLEAN_ERR: (0,), CLEAN_WARN: (0,),
    SET_MODE: (1,), MOVE_JOINT: (40, 41), MOVE_SERVOJ: (40,),
    GET_TCP_POSE: (0,), GET_JOINT_POS: (0,), SERVO_DBMSG: (0,),
}


class ControlServer:
    """Serve control requests with atomic state updates and response snapshots."""

    def __init__(self, state, host='127.0.0.1', port=502):
        """Configure the control endpoint."""
        self.state = state
        self.host = host
        self.port = port

    def start(self):
        """Bind synchronously so startup cannot hide an unavailable TCP endpoint."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(5)
            threading.Thread(target=self._run, args=(sock,), daemon=True).start()
        except Exception:
            sock.close()
            raise

    def _run(self, sock):
        with sock:
            _logger.info('Control listening on %s:%s', self.host, self.port)
            while True:
                conn, _ = sock.accept()
                threading.Thread(
                    target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn):
        with conn:
            try:
                while True:
                    header = self._recv_exact(conn, 6)
                    if header is None:
                        return
                    transaction_id, protocol_id, length = struct.unpack('>HHH', header)
                    # There is no register to echo for an empty frame. Unsupported
                    # protocols cannot safely be interpreted as private commands.
                    # SDK firmware >= 1.8.6 uses ID 3 for heartbeat-enabled
                    # private control; command/response framing is unchanged.
                    if protocol_id not in (2, 3) or length < 1:
                        return
                    body = self._recv_exact(conn, length)
                    if body is None:
                        return
                    status, payload = self._handle_request(body[0], body[1:])
                    frame = struct.pack(
                        '>HHHBB', transaction_id, protocol_id,
                        len(payload) + 2, body[0], status)
                    conn.sendall(frame + payload)
            except OSError as exc:
                _logger.debug('Control connection closed: %s', exc)

    def _handle_request(self, command, params):
        with self.state.lock:
            self.state.check_feedback()
            if len(params) not in _PARAMETER_LENGTHS.get(command, ()):
                return self._reply(extra_status=0x08)
            return self._dispatch(command, params)

    def _reply(self, payload=b'', extra_status=0):
        # Capture the status under the same lock as the command's payload.
        return self.state.response_status | extra_status, payload

    def _dispatch(self, command, params):
        state = self.state
        if command == GET_VERSION:
            return self._reply(state.firmware_version.encode('ascii'))
        if command == GET_STATE:
            return self._reply(bytes([state.state]))
        if command == GET_CMDNUM:
            return self._reply(struct.pack('>H', state.cmdnum))
        if command == GET_ERROR:
            return self._reply(bytes([state.error_code, state.warn_code]))
        if command == GET_TCP_POSE:
            return self._reply(struct.pack('<6f', *state.tcp_pose))
        if command == GET_JOINT_POS:
            return self._reply(struct.pack('<7f', *state.joint_angles))
        if command == SERVO_DBMSG:
            return self._reply(bytes(16))
        if command == CLEAN_WARN:
            state.warn_code = 0
        elif command == CLEAN_ERR:
            if not state.c54_active:
                state.error_code = 0
            state.reset()
        elif command == MOTION_EN:
            servo_id, enable = params
            if enable not in (0, 1) or servo_id not in (*range(1, state.dof + 1), 8):
                return self._reply(extra_status=0x08)
            mask = state.joint_mask if servo_id == 8 else 1 << (servo_id - 1)
            if enable:
                state.servo_enable |= mask
            else:
                state.servo_enable &= ~mask
            state.reset()
        elif command == SET_STATE:
            requested = params[0]
            if requested not in (0, 3, 4):
                return self._reply(extra_status=0x08)
            if requested == 0:
                if (state.error_code or state.c54_active
                        or not state.feedback_fresh
                        or state.servo_enable != state.joint_mask):
                    return self._reply(extra_status=0x10)
                state.state = 2
            else:
                state.state = requested
                state.hold()
                if requested == 4:
                    state.cmdnum = 0
        elif command == SET_MODE:
            # Only the advertised position, servo, and joint teaching modes
            # are supported; there are no velocity-mode handlers in this emulator.
            if params[0] not in (0, 1, 2):
                return self._reply(extra_status=0x08)
            state.mode = params[0]
            state.reset()
        elif command in (MOVE_JOINT, MOVE_SERVOJ):
            if state.external_feedback and command == MOVE_JOINT:
                # Gazebo currently supports streamed ServoJ, not joint planning.
                return self._reply(extra_status=0x08)
            values = struct.unpack('<10f', params[:40])
            if not all(math.isfinite(value) for value in values):
                return self._reply(extra_status=0x08)
            if len(params) == 41 and params[40] != 0:
                # No collision/limit planner: never claim a check succeeded.
                return self._reply(extra_status=0x08)
            if command == MOVE_JOINT and (values[7] < 0 or values[8] < 0):
                return self._reply(extra_status=0x08)
            required_mode = 0 if command == MOVE_JOINT else 1
            if not state.motion_ready or state.mode != required_mode:
                return self._reply(extra_status=0x10)
            state.joint_targets[:] = list(values[:state.dof]) + [0.0] * (7 - state.dof)
            if state.external_feedback:
                state.state = 1
            else:
                state.joint_angles[:] = state.joint_targets
                state.state = 2
            state.cmdnum = 0
            if command == MOVE_JOINT:
                return self._reply(struct.pack('>H', state.cmdnum))
        return self._reply()

    @staticmethod
    def _recv_exact(conn, count):
        buf = bytearray()
        while len(buf) < count:
            part = conn.recv(count - len(buf))
            if not part:
                return None
            buf.extend(part)
        return bytes(buf)
