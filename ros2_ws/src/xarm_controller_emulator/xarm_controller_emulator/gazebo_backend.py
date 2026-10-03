"""Bridge ServoJ targets and measured Gazebo joint states without TCP waits."""

import math
import time

from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


class GazeboBackend:
    """Use advancing Gazebo samples as the only source of measured positions."""

    def __init__(self, node, state, feedback_timeout):
        """Configure fixed xArm7 simulation topics and a steady-clock watchdog."""
        if not math.isfinite(feedback_timeout) or feedback_timeout <= 0:
            raise ValueError('feedback_timeout must be finite and positive')
        self.state = state
        self.stamp = None
        self.joints = [f'joint{i}' for i in range(1, state.dof + 1)]
        with state.lock:
            state.external_feedback = True
            state.feedback_timeout = feedback_timeout
            state.feedback_received_at = None
            state.check_feedback()
        self.commands = node.create_publisher(
            Float64MultiArray, '/sim/joint_position_controller/commands', 1)
        self.readiness = node.create_publisher(Bool, '~/feedback_ready', 1)
        self.subscription = node.create_subscription(
            JointState, '/sim/joint_state_broadcaster/joint_states', self.receive, 1)
        self.timer = node.create_timer(
            0.004, self.update, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, message):
        """Accept complete finite joint samples, rejecting repeated simulation time."""
        if len(message.name) != len(message.position) or len(set(message.name)) != len(
                message.name):
            return
        positions = dict(zip(message.name, message.position))
        if any(name not in positions or not math.isfinite(positions[name])
               for name in self.joints):
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        with self.state.lock:
            state = self.state
            state.check_feedback()
            if self.stamp is not None and stamp == self.stamp:
                return
            if self.stamp is not None and stamp < self.stamp:
                # A reset world must not inherit commands from the previous world.
                state.state = 4
                state.mode = 0
                state.cmdnum = 0
            self.stamp = stamp
            state.joint_angles[:] = [positions[name] for name in self.joints] + [0.0] * (
                7 - state.dof)
            state.feedback_received_at = time.monotonic()
            if not state.motion_ready:
                state.hold()
            else:
                state.state = 2 if all(abs(actual - target) < 0.001 for actual, target in
                                       zip(state.joint_angles, state.joint_targets)) else 1

    def update(self):
        """Publish the current target or hold, even while simulation time is paused."""
        with self.state.lock:
            self.state.check_feedback()
            ready = self.state.feedback_fresh
            targets = None
            if self.state.feedback_received_at is not None:
                targets = self.state.joint_targets[:self.state.dof]
        if targets is not None:
            self.commands.publish(Float64MultiArray(data=targets))
        self.readiness.publish(Bool(data=ready))
