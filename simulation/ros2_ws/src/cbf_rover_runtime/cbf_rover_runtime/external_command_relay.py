"""Disabled-by-default integration relay for externally produced commands."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class ExternalCommandRelay(Node):
    """Forward an external stamped command without assigning safety semantics."""

    def __init__(self) -> None:
        super().__init__("external_command_relay")
        self.declare_parameter("input_topic", "/external_command")
        self.declare_parameter("output_topic", "/cmd_vel")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._subscription = self.create_subscription(
            TwistStamped,
            input_topic,
            self._on_command,
            10,
        )
        self.get_logger().warning(
            "external_command_relay only adapts message types; it does not "
            "validate a certificate, action, deadline, or safety property"
        )

    def _on_command(self, message: TwistStamped) -> None:
        self._publisher.publish(message.twist)

    def publish_stop(self) -> None:
        """Publish one zero command during an orderly relay shutdown."""

        self._publisher.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ExternalCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
