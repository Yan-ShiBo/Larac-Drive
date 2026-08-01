"""Publish the resolved ScenarioSpec as durable RViz markers."""

from __future__ import annotations

from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from cbf_rover.application.scenario_spec import load_scenario_spec
from cbf_rover.domain.planar import CANONICAL_SCENARIO


class ScenarioVisualizerNode(Node):
    """Render physical and inflated obstacle geometry from one YAML snapshot."""

    def __init__(self) -> None:
        super().__init__("scenario_visualizer")
        self.declare_parameter("scenario_path", "")
        self.declare_parameter("marker_topic", "/cbf_rover/scenario/markers")
        self.declare_parameter("marker_frame", "odom")
        self.declare_parameter("odom_is_world_identity", True)

        scenario_path = Path(
            str(self.get_parameter("scenario_path").value)
        ).expanduser()
        if not scenario_path.is_file():
            raise RuntimeError("scenario_path must name the canonical YAML")
        if not bool(self.get_parameter("odom_is_world_identity").value):
            raise RuntimeError(
                "scenario markers require an explicit world-to-odom transform"
            )
        self._resolved = load_scenario_spec(scenario_path)
        self._frame = str(self.get_parameter("marker_frame").value)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter("marker_topic").value),
            qos,
        )
        self._timer = self.create_timer(1.0, self._publish)
        self.get_logger().info(
            "resolved ScenarioSpec "
            f"{self._resolved.scenario_id} "
            f"sha256={self._resolved.source_sha256}"
        )

    def _base_marker(
        self,
        *,
        namespace: str,
        marker_id: int,
        marker_type: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _physical_markers(self) -> list[Marker]:
        markers: list[Marker] = []
        for index, obstacle in enumerate(self._resolved.physical_obstacles):
            marker = self._base_marker(
                namespace="physical_obstacles",
                marker_id=100 + index,
                marker_type=Marker.CUBE,
            )
            marker.pose.position.x = obstacle.center_m[0]
            marker.pose.position.y = obstacle.center_m[1]
            marker.pose.position.z = obstacle.center_m[2]
            marker.scale.x = obstacle.size_m[0]
            marker.scale.y = obstacle.size_m[1]
            marker.scale.z = obstacle.size_m[2]
            marker.color.r = 0.82
            marker.color.g = 0.18
            marker.color.b = 0.16
            marker.color.a = 1.0
            markers.append(marker)
        return markers

    def _set_markers(self) -> list[Marker]:
        markers: list[Marker] = []
        for index, region in enumerate(
            CANONICAL_SCENARIO.unsafe_components
        ):
            marker = self._base_marker(
                namespace="inflated_unsafe_sets",
                marker_id=200 + index,
                marker_type=Marker.CUBE,
            )
            marker.pose.position.x = (region.x_min + region.x_max) / 2.0
            marker.pose.position.y = (region.y_min + region.y_max) / 2.0
            marker.pose.position.z = 0.025
            marker.scale.x = region.x_max - region.x_min
            marker.scale.y = region.y_max - region.y_min
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.45
            marker.color.b = 0.05
            marker.color.a = 0.32
            markers.append(marker)

        for marker_id, namespace, disk, red, green, blue in (
            (
                300,
                "initial_set",
                CANONICAL_SCENARIO.initial,
                0.2,
                0.8,
                0.9,
            ),
            (
                301,
                "goal_set",
                CANONICAL_SCENARIO.goal,
                0.1,
                0.8,
                0.25,
            ),
        ):
            marker = self._base_marker(
                namespace=namespace,
                marker_id=marker_id,
                marker_type=Marker.CYLINDER,
            )
            marker.pose.position.x = disk.center_x
            marker.pose.position.y = disk.center_y
            marker.pose.position.z = 0.035
            marker.scale.x = 2.0 * disk.radius
            marker.scale.y = 2.0 * disk.radius
            marker.scale.z = 0.07
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 0.65
            markers.append(marker)

        domain = CANONICAL_SCENARIO.domain
        boundary = self._base_marker(
            namespace="domain_boundary",
            marker_id=400,
            marker_type=Marker.LINE_STRIP,
        )
        boundary.scale.x = 0.025
        boundary.color.r = 0.85
        boundary.color.g = 0.85
        boundary.color.b = 0.85
        boundary.color.a = 0.9
        for x_value, y_value in (
            (domain.x_min, domain.y_min),
            (domain.x_max, domain.y_min),
            (domain.x_max, domain.y_max),
            (domain.x_min, domain.y_max),
            (domain.x_min, domain.y_min),
        ):
            point = Point()
            point.x = x_value
            point.y = y_value
            point.z = 0.06
            boundary.points.append(point)
        markers.append(boundary)
        return markers

    def _publish(self) -> None:
        message = MarkerArray()
        message.markers = self._physical_markers() + self._set_markers()
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScenarioVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
