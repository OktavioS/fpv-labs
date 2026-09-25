import math
from typing import TYPE_CHECKING

from geometry_msgs.msg import TwistStamped

from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode


class FlyingToTargetHandler(FlightStateHandler):
    """Завдання 4: горизонтальний переліт до індивідуальної цільової точки.

    Ціль задається як зміщення (DELTA_X, DELTA_Y) від фактичної початкової
    позиції дрона (варіант 45, Таблиця 2.3 методички):
        x_target = x0 - DELTA_X
        y_target = y0 - DELTA_Y
    """

    DELTA_X = 5.8
    DELTA_Y = -0.4

    POSITION_TOLERANCE = 0.25

    MAX_HORIZONTAL_SPEED = 2.0
    SPEED_GAIN = 0.6

    def handle(self, node: "FlightTestNode") -> None:
        if node.odometry is None or node.start_position is None:
            return

        if node.target_position is None:
            x0, y0 = node.start_position
            node.target_position = (
                x0 - self.DELTA_X,
                y0 - self.DELTA_Y,
            )
            node.get_logger().info(
                f"Ціль (варіант 45): "
                f"({node.target_position[0]:.2f}, "
                f"{node.target_position[1]:.2f}) м"
            )


        target_x, target_y = node.target_position
        current_x = node.odometry.pose.pose.position.x
        current_y = node.odometry.pose.pose.position.y

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)

        if distance <= self.POSITION_TOLERANCE:
            self._publish_velocity(node, 0.0, 0.0)

            node.final_position = (current_x, current_y)
            node.position_error = distance

            node.get_logger().info(
                f"Ціль досягнута. Фактична позиція: "
                f"({current_x:.2f}, {current_y:.2f}) м, "
                f"похибка: {distance:.3f} м"
            )

            node.state = FlightState.LANDING
            return

        speed = min(self.MAX_HORIZONTAL_SPEED, distance * self.SPEED_GAIN)
        vx = speed * dx / distance
        vy = speed * dy / distance

        self._publish_velocity(node, vx, vy)

    def _publish_velocity(
        self, node: "FlightTestNode", vx: float, vy: float
    ) -> None:
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = vx
        message.twist.linear.y = vy
        message.twist.linear.z = 0.0
        message.twist.angular.z = 0.0

        node.cmd_vel_publisher.publish(message)
