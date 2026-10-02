import socket
import struct
import threading
import time
import logging

logging.basicConfig(
    level=logging.INFO, format="[%(levelname)s] [%(name)s]: %(message)s"
)
_logger = logging.getLogger(__name__)
_logger.setLevel(logging.INFO)

NORMAL_REPORT_SIZE = 145
RICH_REPORT_SIZE = 245


def put_u8(buf, offset, value):
    struct.pack_into(">B", buf, offset, value)


def put_u16(buf, offset, value):
    struct.pack_into(">H", buf, offset, value)


def put_u32(buf, offset, value):
    struct.pack_into(">I", buf, offset, value)


def put_f32(buf, offset, value):
    struct.pack_into(">f", buf, offset, value)


def fill_common_report(buf, state):
    """
    Fill bytes common to normal/rich reports.
    """

    # First four bytes of the actual TCP report are its size.
    put_u32(buf, 0, len(buf))

    # low nibble = state
    # high nibble = mode
    put_u8(
        buf,
        4,
        ((state.mode & 0x0F) << 4) | (state.state & 0x0F),
    )

    put_u16(buf, 5, state.cmdnum)

    # 7 joint positions
    offset = 7

    for value in state.joint_angles:
        put_f32(buf, offset, value)
        offset += 4

    # 6 TCP pose values
    for value in state.tcp_pose:
        put_f32(buf, offset, value)
        offset += 4

    # 7 joint torques
    for value in state.joint_torques:
        put_f32(buf, offset, value)
        offset += 4

    # 87
    put_u8(buf, 87, state.servo_brake)

    # 88
    put_u8(buf, 88, state.servo_enable)

    # 89, 90
    put_u8(buf, 89, state.error_code)
    put_u8(buf, 90, state.warn_code)

    # TCP offset
    for i in range(6):
        put_f32(buf, 91 + i * 4, 0.0)

    # TCP load
    for i in range(4):
        put_f32(buf, 115 + i * 4, 0.0)

    # collision / teach sensitivity
    put_u8(buf, 131, 0)
    put_u8(buf, 132, 0)

    # gravity direction
    put_f32(buf, 133, 0.0)
    put_f32(buf, 137, 0.0)
    put_f32(buf, 141, -1.0)


def build_normal_report(state):
    buf = bytearray(NORMAL_REPORT_SIZE)

    fill_common_report(buf, state)

    return bytes(buf)


def build_rich_report(state):
    buf = bytearray(RICH_REPORT_SIZE)

    fill_common_report(buf, state)

    # Rich-specific data begins at byte 145.

    # SDK defaults device_type to 7.
    # Fine for our generic xArm emulator for now.
    put_u8(buf, 145, 7)

    # Number of joints
    put_u8(buf, 146, state.dof)

    # master_id
    put_u8(buf, 147, 0)

    # slave_id
    put_u8(buf, 148, 0)

    # motor_tid
    put_u8(buf, 149, 0)

    # motor_fid
    put_u8(buf, 150, 0)

    # Firmware/version text occupies 30 bytes.
    version = state.firmware_version.encode("ascii")
    buf[151 : 151 + len(version)] = version

    # 181..200:
    # TCP jerk, min acc, max acc, min velocity, max velocity
    tcp_params = [
        1000.0,
        1.0,
        50000.0,
        0.1,
        1000.0,
    ]

    offset = 181

    for value in tcp_params:
        put_f32(buf, offset, value)
        offset += 4

    # 201..220:
    # joint jerk, min acc, max acc, min velocity, max velocity
    joint_params = [
        20.0,
        0.01,
        20.0,
        0.01,
        4.0,
    ]

    offset = 201

    for value in joint_params:
        put_f32(buf, offset, value)
        offset += 4

    # 221..228
    put_f32(buf, 221, 2.3)
    put_f32(buf, 225, 2.7)

    # 229..244 = servo status/error data.
    #
    # Zero is sufficient for our initial emulator.
    # bytearray is already zero-filled.

    return bytes(buf)


class ReportServer:
    def __init__(
        self,
        state,
        port,
        packet_builder,
        name,
        host="127.0.0.1",
    ):
        self.state = state
        self.host = host
        self.port = port
        self.packet_builder = packet_builder
        self.name = name

    def start(self):
        thread = threading.Thread(
            target=self._run,
            daemon=True,
        )
        thread.start()

    def _run(self):
        sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )

        sock.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )

        sock.bind((self.host, self.port))
        sock.listen(5)

        _logger.info(f"[report:{self.name}] " f"listening on {self.host}:{self.port}")

        while True:
            conn, addr = sock.accept()

            _logger.info(f"[report:{self.name}] " f"connection from {addr}")

            threading.Thread(
                target=self._client,
                args=(conn,),
                daemon=True,
            ).start()

    def _client(self, conn):
        with conn:
            try:
                while True:
                    packet = self.packet_builder(self.state)

                    conn.sendall(packet)

                    # normal/rich reports are nominally 5 Hz
                    time.sleep(0.2)

            except (
                BrokenPipeError,
                ConnectionResetError,
            ):
                _logger.info(f"[report:{self.name}] disconnected")
