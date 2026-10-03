"""ROS service for injecting a latched controller fault."""

from rclpy.node import Node
from std_srvs.srv import SetBool

from .gazebo_backend import GazeboBackend


class XArmEmulatorNode(Node):
    """Expose the simulated C54 condition through a private ROS service."""

    def __init__(self, state):
        """Bind the ROS interface to the shared controller state."""
        super().__init__('xarm_controller_emulator')
        self.state = state
        backend = self.declare_parameter('backend', 'instantaneous').value
        feedback_timeout = self.declare_parameter('feedback_timeout', 0.5).value
        if backend not in ('instantaneous', 'gazebo'):
            raise ValueError('backend must be instantaneous or gazebo')
        self.backend = (
            GazeboBackend(self, state, feedback_timeout) if backend == 'gazebo' else None)
        self.create_service(SetBool, '~/set_c54', self._set_c54)
        self.get_logger().info('xArm emulator ROS interface ready')

    def _set_c54(self, request, response):
        self.state.set_c54(request.data)
        response.success = True
        response.message = (
            'C54 collision fault triggered' if request.data else
            'C54 condition released; error remains latched until clean_error'
        )
        self.get_logger().info(response.message)
        return response
