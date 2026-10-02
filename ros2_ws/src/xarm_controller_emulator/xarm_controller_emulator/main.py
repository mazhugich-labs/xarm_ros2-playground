import rclpy

from .state import RobotState
from .control_server import ControlServer
from .report_server import (
    ReportServer,
    build_normal_report,
    build_rich_report,
)
from .emulator_node import XArmEmulatorNode


def main(args=None):
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

    control.start()
    normal_report.start()
    rich_report.start()

    node = XArmEmulatorNode(state)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
