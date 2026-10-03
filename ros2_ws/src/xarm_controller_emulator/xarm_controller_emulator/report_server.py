"""Build the normal and legacy rich xArm controller reports."""

import logging
import math
import socket
import struct
import threading
import time

_logger = logging.getLogger(__name__)

NORMAL_REPORT_SIZE = 145
RICH_REPORT_SIZE = 245


def _fill_common_report(buf, state):
    # Report integers are big-endian; all float fields are little-endian.
    struct.pack_into(
        '>IBH', buf, 0, len(buf), (state.mode << 4) | state.state, state.cmdnum)
    struct.pack_into('<7f', buf, 7, *state.joint_angles)
    struct.pack_into('<6f', buf, 35, *state.tcp_pose)
    struct.pack_into('<7f', buf, 59, *state.joint_torques)
    struct.pack_into(
        '4B', buf, 87, state.servo_brake, state.servo_enable,
        state.error_code, state.warn_code)
    # TCP offset, load, and collision sensitivity remain zero in this model.
    buf[132] = 1  # Teaching sensitivity: valid range is 1..5.
    struct.pack_into('<3f', buf, 133, 0.0, 0.0, -1.0)


def build_normal_report(state):
    """Take an atomic snapshot of the 145-byte normal report."""
    with state.lock:
        state.check_feedback()
        buf = bytearray(NORMAL_REPORT_SIZE)
        _fill_common_report(buf, state)
        return bytes(buf)


def build_rich_report(state):
    """Take an atomic snapshot of the SDK-compatible 245-byte rich report."""
    with state.lock:
        state.check_feedback()
        buf = bytearray(RICH_REPORT_SIZE)
        _fill_common_report(buf, state)
        struct.pack_into('6B', buf, 145, state.dof, state.dof, 0xAA, 0x55, 0, 0)
        struct.pack_into('30s', buf, 151, state.firmware_version.encode('ascii'))
        struct.pack_into('<5f', buf, 181, 1000.0, 1.0, 50000.0, 0.1, 1000.0)
        struct.pack_into('<5f', buf, 201, 20.0, 0.01, 20.0, 0.01, math.pi)
        struct.pack_into('<2f', buf, 221, 2.3, 2.7)
        # Servo and tool IO status/error pairs at 229..244 remain zero.
        return bytes(buf)


class ReportServer:
    """Publish a report stream at the documented normal/rich rate of 5 Hz."""

    def __init__(self, state, port, packet_builder, name, host='127.0.0.1'):
        """Configure the reporting endpoint and packet builder."""
        self.state = state
        self.host = host
        self.port = port
        self.packet_builder = packet_builder
        self.name = name

    def start(self):
        """Bind synchronously so a reporting-port conflict fails startup."""
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
            _logger.info('Report %s listening on %s:%s', self.name, self.host, self.port)
            while True:
                conn, _ = sock.accept()
                threading.Thread(
                    target=self._client, args=(conn,), daemon=True).start()

    def _client(self, conn):
        with conn:
            try:
                while True:
                    conn.sendall(self.packet_builder(self.state))
                    time.sleep(0.2)
            except OSError as exc:
                _logger.debug('Report connection closed: %s', exc)
