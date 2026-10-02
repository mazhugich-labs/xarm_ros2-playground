import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool


class XArmEmulatorNode(Node):

    def __init__(self, state):
        super().__init__("xarm_controller_emulator")

        self.state = state

        self.create_service(
            SetBool,
            "~/set_c54",
            self._set_c54,
        )

        self.get_logger().info("xArm emulator ROS interface ready")

    def _set_c54(self, request, response):
        with self.state.lock:

            if request.data:
                # Fault condition becomes active.
                self.state.c54_active = True

                # Controller latches C54.
                self.state.error_code = 54

                # xArm error behavior:
                # stop robot and return to position mode.
                self.state.state = 4
                self.state.mode = 0

                response.success = True
                response.message = "C54 collision fault triggered"

                self.get_logger().warn(
                    "C54 triggered: "
                    f"direction={self.state.c54_direction}, "
                    f"threshold={self.state.c54_threshold}, "
                    f"actual={self.state.c54_actual}"
                )

            else:
                # Physical collision condition has disappeared,
                # but controller error remains latched.
                self.state.c54_active = False

                response.success = True
                response.message = (
                    "C54 condition released; " "error remains latched until clean_error"
                )

                self.get_logger().info("C54 condition released")

        return response
