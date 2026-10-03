"""Start the TCP controller, reporting endpoints, and ROS fault service."""

import logging

import rclpy
from rclpy.executors import ExternalShutdownException

from .state import RobotState
from .control_server import ControlServer
from .report_server import (
    ReportServer,
    build_normal_report,
    build_rich_report,
)
from .emulator_node import XArmEmulatorNode


def main(args=None):
    """Run the emulator until ROS shuts down."""
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)

    state = RobotState(
        dof=7,
    )

    control = ControlServer(
        state=state,
        host="127.0.0.1",
        port=502,
    )

    normal_report = ReportServer(
        state=state,
        host="127.0.0.1",
        port=30001,
        packet_builder=build_normal_report,
        name="normal",
    )

    rich_report = ReportServer(
        state=state,
        host="127.0.0.1",
        port=30002,
        packet_builder=build_rich_report,
        name="rich",
    )

    # Configure feedback readiness before any TCP endpoint accepts clients.
    node = XArmEmulatorNode(state)

    control.start()
    normal_report.start()
    rich_report.start()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
