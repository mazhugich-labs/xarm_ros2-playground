import socket
import struct
import threading
import logging

logging.basicConfig(
    level=logging.INFO, format="[%(levelname)s] [%(name)s]: %(message)s"
)
_logger = logging.getLogger(__name__)
_logger.setLevel(logging.INFO)

GET_VERSION = 0x01

MOTION_EN = 0x0B
SET_STATE = 0x0C
GET_STATE = 0x0D
GET_CMDNUM = 0x0E
GET_ERROR = 0x0F
SET_MODE = 0x13

GET_TCP_POSE = 0x29
GET_JOINT_POS = 0x2A

SERVO_DBMSG = 0x6A

GET_JOINT_POS = 0x2A

MOVE_JOINT = 0x17
MOVE_SERVOJ = 0x1D

CLEAN_ERR = 0x10
CLEAN_WARN = 0x11


class ControlServer:
    def __init__(
        self,
        state,
        host="127.0.0.1",
        port=502,
    ):
        self.state = state
        self.host = host
        self.port = port

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

        _logger.info(f"listening on " f"{self.host}:{self.port}")

        while True:
            conn, addr = sock.accept()

            _logger.info(f"connection from {addr}")

            threading.Thread(
                target=self._client,
                args=(conn,),
                daemon=True,
            ).start()

    def _client(self, conn):
        with conn:
            while True:
                header = self._recv_exact(conn, 6)

                if header is None:
                    _logger.info("disconnected")
                    return

                transaction_id, protocol_id, length = struct.unpack(">HHH", header)

                body = self._recv_exact(conn, length)

                if body is None:
                    return

                command = body[0]
                params = body[1:]

                _logger.debug(
                    f"tid={transaction_id} \
                    proto=0x{protocol_id:04X} \
                    len={length} \
                    reg=0x{command:02X} \
                    data={params.hex(' ')}",
                )

                response = self._handle_request(
                    command,
                    params,
                )

                if response is None:
                    # xArm private protocol:
                    # bit 3 means invalid.
                    self._send_response(
                        conn,
                        transaction_id,
                        protocol_id,
                        command,
                        status=0x08,
                    )

                    _logger.warning(f"UNIMPLEMENTED reg=0x{command:02X}")

                else:
                    self._send_response(
                        conn,
                        transaction_id,
                        protocol_id,
                        command,
                        payload=response,
                    )

    def _handle_request(self, command, params):
        if command == CLEAN_ERR:
            with self.state.lock:

                if self.state.error_code == 54 and self.state.c54_active:
                    _logger.warning(
                        "[control] CLEAN_ERR rejected: C54 condition still active"
                    )

                    # Keep C54 latched.
                    #
                    # We still return protocol success because
                    # the command itself was accepted; the next
                    # report/GET_ERROR shows that C54 remains.
                    return b""

                old_error = self.state.error_code
                self.state.error_code = 0

                _logger.info(f"[control] CLEAN_ERR {old_error} -> 0")

            return b""

        if command == CLEAN_WARN:
            _logger.info("CLEAN_WARN -> 0")
            self.state.warn_code = 0
            return b""

        if command == MOVE_SERVOJ:
            # SDK sends exactly 10 float32 values:
            #
            # 0..6 = joint target positions
            # 7    = speed       (reserved for servoj)
            # 8    = acceleration (reserved)
            # 9    = mvtime       (reserved)

            if len(params) != 40:
                _logger.warning(f"MOVE_SERVOJ invalid payload length: {len(params)}")
                return None

            values = struct.unpack(">10f", params)

            target_angles = list(values[:7])
            speed = values[7]
            acceleration = values[8]
            mvtime = values[9]

            # For the initial emulator, treat ServoJ as instantaneous.
            #
            # xArm protocol always carries 7 slots, even for xArm6/xArm5.
            for i in range(7):
                if i < self.state.dof:
                    self.state.joint_angles[i] = target_angles[i]
                else:
                    self.state.joint_angles[i] = 0.0

            # Do NOT log every ServoJ packet once things work.
            # During trajectory execution this can be high frequency.
            #
            _logger.debug(f"MOVE_SERVOJ {target_angles}")

            return b""

        if command == MOVE_JOINT:
            # 10 float32 values:
            #
            # 0..6 = joint targets
            # 7    = speed
            # 8    = acceleration
            # 9    = mvtime
            #
            # Optional byte 40 = only_check_type

            if len(params) not in (40, 41):
                _logger.warning(f"MOVE_JOINT invalid payload length: {len(params)}")
                return None

            values = struct.unpack(
                ">10f",
                params[:40],
            )

            target_angles = list(values[:7])
            speed = values[7]
            acceleration = values[8]
            mvtime = values[9]

            only_check_type = params[40] if len(params) == 41 else 0

            _logger.debug(f"MOVE_JOINT \
                angles={target_angles} \
                speed={speed:.4f} \
                acc={acceleration:.4f} \
                mvtime={mvtime:.4f} \
                only_check={only_check_type}")

            # Initial emulator behaviour:
            # move instantaneously to the requested position.
            for i in range(7):
                if i < self.state.dof:
                    self.state.joint_angles[i] = target_angles[i]
                else:
                    self.state.joint_angles[i] = 0.0

            # If only_check_type is enabled, the SDK expects
            # three response bytes and examines byte 2 as
            # only_check_result.
            if only_check_type > 0:
                return bytes(
                    [
                        0,
                        0,
                        0,  # only_check_result = OK
                    ]
                )

            return b""

        if command == GET_JOINT_POS:
            payload = struct.pack(
                ">7f",
                *self.state.joint_angles,
            )

            _logger.debug(f"GET_JOINT_POS -> {self.state.joint_angles}")

            return payload

        # --------------------------------------------------
        # GET_VERSION
        # --------------------------------------------------

        if command == GET_VERSION:
            _logger.info(f"GET_VERSION -> {self.state.firmware_version}")

            return self.state.firmware_version.encode("ascii")

        # --------------------------------------------------
        # GET_ERROR
        #
        # response:
        #   byte 0 error
        #   byte 1 warning
        # --------------------------------------------------

        if command == GET_ERROR:
            _logger.info(
                f"GET_ERROR -> error={self.state.error_code} warn={self.state.warn_code}"
            )

            return bytes(
                [
                    self.state.error_code,
                    self.state.warn_code,
                ]
            )

        # --------------------------------------------------
        # SERVO_DBMSG
        #
        # 16 bytes = status/error pairs
        # for 8 servo slots.
        #
        # All zero means no servo faults.
        # --------------------------------------------------

        if command == SERVO_DBMSG:
            _logger.info(f"SERVO_DBMSG -> 0 (no servo faults)")

            return bytes(16)

        # --------------------------------------------------
        # GET_STATE
        # --------------------------------------------------
        if command == GET_STATE:
            _logger.info(f"GET_STATE -> {self.state.state}")

            return bytes([self.state.state & 0xFF])

        # --------------------------------------------------
        # GET_CMDNUM
        # --------------------------------------------------
        if command == GET_CMDNUM:
            _logger.info(f"GET_CMDNUM -> {self.state.cmdnum}")
            return struct.pack(
                ">H",
                self.state.cmdnum,
            )

        # --------------------------------------------------
        # MOTION_EN
        #
        # params:
        #   byte 0: servo id
        #   byte 1: enable
        #
        # servo id:
        #   1..7 = individual joint
        #   8    = all joints
        # --------------------------------------------------
        if command == MOTION_EN:
            if len(params) < 2:
                _logger.warning("MOTION_EN invalid payload")
                return None

            servo_id = params[0]
            enable = params[1] != 0

            if servo_id == 8:
                mask = (1 << self.state.dof) - 1

                self.state.servo_enable = mask if enable else 0

            elif 1 <= servo_id <= self.state.dof:
                bit = 1 << (servo_id - 1)

                if enable:
                    self.state.servo_enable |= bit
                else:
                    self.state.servo_enable &= ~bit

            else:
                _logger.warning(f" invalid servo id: {servo_id}")
                return None

            _logger.info(
                f"MOTION_EN id={servo_id} enable={enable} mask=0x{self.state.servo_enable:02X}"
            )

            return b""

        # --------------------------------------------------
        # GET_TCP_POSE
        # --------------------------------------------------

        if command == GET_TCP_POSE:
            return struct.pack(
                ">6f",
                *self.state.tcp_pose,
            )

        # --------------------------------------------------
        # GET_JOINT_POS
        # --------------------------------------------------

        if command == GET_JOINT_POS:
            return struct.pack(
                ">7f",
                *self.state.joint_angles,
            )

        # --------------------------------------------------
        # SET_STATE
        #
        # Important values:
        #   0 = motion/ready
        #   3 = pause
        #   4 = stop
        # --------------------------------------------------
        if command == SET_STATE:
            if len(params) < 1:
                _logger.warning("SET_STATE invalid payload")
                return None

            new_state = params[0]

            self.state.state = new_state

            _logger.info(f"SET_STATE -> {new_state}")

            return b""

        # --------------------------------------------------
        # SET_MODE
        #
        # Important values:
        #   0 = position
        #   1 = servo motion
        #   2 = joint teaching
        #   4 = joint velocity
        #   5 = cartesian velocity
        # --------------------------------------------------
        if command == SET_MODE:
            if len(params) < 1:
                _logger.warning("SET_MODE invalid payload")
                return None

            new_mode = params[0]

            self.state.mode = new_mode

            _logger.info(f"SET_MODE -> {new_mode}")

            return b""

        return None

    @staticmethod
    def _send_response(
        conn,
        transaction_id,
        protocol_id,
        command,
        payload=b"",
        status=0,
    ):
        length = 2 + len(payload)

        frame = struct.pack(
            ">HHHBB",
            transaction_id,
            protocol_id,
            length,
            command,
            status,
        )

        frame += payload

        conn.sendall(frame)

    @staticmethod
    def _recv_exact(conn, count):
        buf = bytearray()

        while len(buf) < count:
            part = conn.recv(count - len(buf))

            if not part:
                return None

            buf.extend(part)

        return bytes(buf)
