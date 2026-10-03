"""Shared measured state and command targets for the controller emulator."""

from dataclasses import dataclass, field
from threading import RLock
import time


@dataclass
class RobotState:
    """Keep state changes and report snapshots under the same reentrant lock."""

    dof: int = 7
    firmware_version: str = 'v1.8.102'
    state: int = 2  # Idle; SET_STATE(0) requests readiness, not reported state 0.
    mode: int = 0
    cmdnum: int = 0
    joint_angles: list[float] = field(default_factory=lambda: [0.0] * 7)
    joint_targets: list[float] = field(default_factory=lambda: [0.0] * 7)
    external_feedback: bool = False
    feedback_timeout: float = 0.5
    feedback_received_at: float | None = None
    tcp_pose: list[float] = field(default_factory=lambda: [0.0] * 6)
    joint_torques: list[float] = field(default_factory=lambda: [0.0] * 7)
    error_code: int = 0
    warn_code: int = 0
    servo_brake: int = 0x7F
    servo_enable: int = 0x7F
    c54_active: bool = False
    c54_direction: int = 0
    c54_threshold: float = 0.0
    c54_actual: float = 0.0
    lock: RLock = field(default_factory=RLock, repr=False, compare=False)

    def __post_init__(self):
        """Validate the model and clear nonexistent joint bits."""
        if self.dof not in (5, 6, 7):
            raise ValueError('dof must be 5, 6 or 7')
        self.servo_brake &= self.joint_mask
        self.servo_enable &= self.joint_mask
        self.hold()
        if self.external_feedback:
            self.state = 4

    @property
    def joint_mask(self):
        """Return the enable mask for the configured robot."""
        return (1 << self.dof) - 1

    @property
    def motion_ready(self):
        """Return readiness while the caller holds the state lock."""
        return (
            self.state in (1, 2)
            and not self.error_code
            and not self.c54_active
            and self.feedback_fresh
            and self.servo_enable & self.joint_mask == self.joint_mask
        )

    @property
    def feedback_fresh(self):
        """Check receipt age independently of ROS or simulation time."""
        return not self.external_feedback or (
            self.feedback_received_at is not None
            and time.monotonic() - self.feedback_received_at < self.feedback_timeout
        )

    def hold(self):
        """Discard any target in favor of measured position; caller holds the lock."""
        self.joint_targets[:] = self.joint_angles

    def check_feedback(self):
        """Latch a stopped state on stale feedback; caller holds the lock."""
        if not self.feedback_fresh:
            self.state = 4
            self.mode = 0
            self.cmdnum = 0
            self.hold()

    @property
    def response_status(self):
        """Encode the controller status bits while holding the state lock."""
        return (
            (0x40 if self.error_code else 0)
            | (0x20 if self.warn_code else 0)
            | (0x00 if self.motion_ready else 0x10)
        )

    def reset(self):
        """Stop execution and discard queued commands; caller holds the lock."""
        self.state = 5
        self.cmdnum = 0
        self.hold()

    def set_c54(self, active):
        """Latch a simulated C54; releasing its condition leaves the error set."""
        with self.lock:
            self.c54_active = active
            if active:
                self.error_code = 54
                self.state = 4
                self.mode = 0
                self.cmdnum = 0
                self.hold()
