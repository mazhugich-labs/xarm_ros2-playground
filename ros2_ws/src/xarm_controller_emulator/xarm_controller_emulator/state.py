from dataclasses import dataclass, field
from threading import RLock


@dataclass
class RobotState:
    dof: int = 7

    firmware_version: str = "v1.8.102"

    state: int = 0
    mode: int = 0
    cmdnum: int = 0

    joint_angles: list[float] = field(default_factory=lambda: [0.0] * 7)

    tcp_pose: list[float] = field(default_factory=lambda: [0.0] * 7)

    joint_torques: list[float] = field(default_factory=lambda: [0.0] * 7)

    error_code: int = 0
    warn_code: int = 0

    servo_brake: int = 0x7F
    servo_enable: int = 0x7F

    c54_active: bool = False

    c54_direction: int = 0

    c54_threshold: float = 0.0
    c54_actual: float = 0.0

    lock: RLock = field(
        default_factory=RLock,
        repr=False,
    )
