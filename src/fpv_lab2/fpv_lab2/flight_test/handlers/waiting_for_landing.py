from typing import TYPE_CHECKING

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class WaitingForLandingHandler(FlightStateHandler):

    def handle(self, node: "FlightTestNode") -> None:
        if node.status is None:
            return

        if not node.status.flying:
            node.get_logger().info("Landing completed")

            if node.odometry is not None:
                altitude = node.odometry.pose.pose.position.z
                node.get_logger().info(
                    f"Final Gazebo altitude: {altitude:.2f} m"
                )


            if node.start_position is not None:
                node.get_logger().info(
                    f"Початкова позиція: "
                    f"({node.start_position[0]:.2f}, "
                    f"{node.start_position[1]:.2f}) м"
                )

            if node.target_position is not None:
                node.get_logger().info(
                    f"Цільова позиція: "
                    f"({node.target_position[0]:.2f}, "
                    f"{node.target_position[1]:.2f}) м"
                )

            if (
                node.final_position is not None
                and node.position_error is not None
            ):
                node.get_logger().info(
                    f"Фактична позиція перед посадкою: "
                    f"({node.final_position[0]:.2f}, "
                    f"{node.final_position[1]:.2f}) м, "
                    f"похибка позиціонування: {node.position_error:.3f} м"
                )

            node.state = FlightState.FINISHED
            node.get_logger().info("Flight test completed successfully")
