"""Empirical ROS2 tracker for one frozen polynomial controller."""

from __future__ import annotations

import json
from math import atan2
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

from cbf_rover.adapters.gazebo.tracker import EmpiricalDifferentialTracker
from cbf_rover.application.certificate_runtime import (
    build_qsbc_runtime_facts,
    runtime_facts_json,
)
from cbf_rover.domain.planar import (
    CANONICAL_SCENARIO,
    CANONICAL_SYSTEM,
    PlanarAction,
    PlanarState,
)
from cbf_rover.methods.polynomial_sbc.certificate import QSBCertificate
from cbf_rover.methods.polynomial_sbc.polynomial import (
    load_frozen_controller,
)


def _yaw_from_odometry(message: Odometry) -> float:
    quaternion = message.pose.pose.orientation
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return atan2(sin_yaw, cos_yaw)


class PolynomialTrackerNode(Node):
    """Track controller actions while publishing W as monitor-only telemetry."""

    def __init__(self) -> None:
        super().__init__("polynomial_tracker")
        self.declare_parameter("controller_path", "")
        self.declare_parameter("certificate_path", "")
        self.declare_parameter("odometry_topic", "/odom")
        self.declare_parameter("command_topic", "/cmd_vel")
        self.declare_parameter("barrier_topic", "/cbf_rover/barrier_value")
        self.declare_parameter(
            "certificate_facts_topic",
            "/cbf_rover/certificate/facts",
        )
        self.declare_parameter(
            "certificate_identity_topic",
            "/cbf_rover/certificate/identity",
        )
        self.declare_parameter(
            "certificate_marker_topic",
            "/cbf_rover/certificate/marker",
        )
        self.declare_parameter("expected_odometry_frame", "odom")
        self.declare_parameter("odom_is_world_identity", True)
        self.declare_parameter("heading_gain", 2.0)
        self.declare_parameter("max_linear_mps", 0.5)
        self.declare_parameter("max_angular_radps", 1.5)
        self.declare_parameter("stale_timeout_s", 0.15)

        controller_path = Path(
            str(self.get_parameter("controller_path").value)
        ).expanduser()
        if not controller_path.is_file():
            raise RuntimeError("controller_path must name a frozen artifact")
        if not bool(self.get_parameter("odom_is_world_identity").value):
            raise RuntimeError(
                "GZ1 requires an explicit odom/world transform; this node only "
                "supports the declared simulation identity"
            )
        self._controller = load_frozen_controller(controller_path)
        certificate_value = str(
            self.get_parameter("certificate_path").value
        ).strip()
        self._certificate = (
            QSBCertificate.load(
                Path(certificate_value).expanduser(),
                controller=self._controller,
            )
            if certificate_value
            else None
        )
        if self._certificate is not None and (
            self._certificate.validation.get("status") != "validated"
        ):
            raise RuntimeError(
                "certificate_path must name an independently validated artifact"
            )
        self._tracker = EmpiricalDifferentialTracker(
            heading_gain=float(self.get_parameter("heading_gain").value),
            max_linear_mps=float(
                self.get_parameter("max_linear_mps").value
            ),
            max_angular_radps=float(
                self.get_parameter("max_angular_radps").value
            ),
        )
        self._expected_frame = str(
            self.get_parameter("expected_odometry_frame").value
        )
        self._stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        self._last_odometry_time = None
        self._command_publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self._barrier_publisher = self.create_publisher(
            Float64,
            str(self.get_parameter("barrier_topic").value),
            10,
        )
        self._facts_publisher = self.create_publisher(
            String,
            str(self.get_parameter("certificate_facts_topic").value),
            10,
        )
        identity_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._identity_publisher = self.create_publisher(
            String,
            str(self.get_parameter("certificate_identity_topic").value),
            identity_qos,
        )
        self._marker_publisher = self.create_publisher(
            Marker,
            str(self.get_parameter("certificate_marker_topic").value),
            10,
        )
        self._odometry_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("odometry_topic").value),
            self._on_odometry,
            10,
        )
        self._watchdog = self.create_timer(0.05, self._on_watchdog)
        self._publish_certificate_identity()
        self.get_logger().warning(
            "GZ1 is empirical: actuator tracking has no formal error bound; "
            "the certificate value is monitor-only and never changes actions"
        )

    def _publish_certificate_identity(self) -> None:
        if self._certificate is None:
            return
        payload = {
            "schema_version": 1,
            "kind": "q_sbc_runtime_identity",
            "mode": "monitor_only",
            "runtime_claim_level": "empirical",
            "certificate_id": self._certificate.certificate_id,
            "certificate_content_sha256": self._certificate.sha256,
            "controller_content_sha256": self._controller.sha256,
            "theorem_id": self._certificate.theorem_id,
            "q": self._certificate.q,
            "epsbar": self._certificate.epsbar,
            "independent_validation_status": (
                self._certificate.validation.get("status")
            ),
            "first_slice_ready": bool(
                self._certificate.validation.get(
                    "first_slice_ready",
                    False,
                )
            ),
        }
        message = String()
        message.data = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        self._identity_publisher.publish(message)

    def _publish_stop(self) -> None:
        self._command_publisher.publish(Twist())

    def _fail_closed(self, message: str) -> None:
        self.get_logger().error(message)
        self._publish_stop()

    def _publish_certificate_facts(
        self,
        *,
        odometry: Odometry,
        state: PlanarState,
        yaw_rad: float,
        nominal_action: PlanarAction,
        command_linear_mps: float,
        command_angular_radps: float,
        saturation_applied: bool,
        terminal_reason: str | None,
    ) -> None:
        if self._certificate is None:
            return
        stamp_s = (
            float(odometry.header.stamp.sec)
            + float(odometry.header.stamp.nanosec) / 1.0e9
        )
        payload = build_qsbc_runtime_facts(
            certificate=self._certificate,
            state=state,
            nominal_action=nominal_action,
            stamp_s=stamp_s,
            yaw_rad=yaw_rad,
            command_linear_mps=command_linear_mps,
            command_angular_radps=command_angular_radps,
            tracker_saturation_applied=saturation_applied,
            geometry_terminal_reason=terminal_reason,
        )
        barrier_value = float(
            payload["certificate"]["barrier_value_W"]
        )
        telemetry = Float64()
        telemetry.data = barrier_value
        self._barrier_publisher.publish(telemetry)

        facts = String()
        facts.data = runtime_facts_json(payload)
        self._facts_publisher.publish(facts)

        marker = Marker()
        marker.header = odometry.header
        marker.ns = "q_sbc_live_value"
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = state.x_m
        marker.pose.position.y = state.y_m
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.a = 1.0
        if terminal_reason == "unsafe_entry":
            marker.color.r = 1.0
            marker.color.g = 0.1
            marker.color.b = 0.1
        elif terminal_reason == "goal_reached":
            marker.color.r = 0.1
            marker.color.g = 0.5
            marker.color.b = 1.0
        else:
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.4
        readiness = (
            "READY"
            if self._certificate.validation.get("first_slice_ready")
            else "VALID / NOT_READY"
        )
        marker.text = (
            f"W(x)={barrier_value:.6f}  q={self._certificate.q:.6f}\n"
            f"{readiness} | EMPIRICAL MONITOR_ONLY"
        )
        self._marker_publisher.publish(marker)

    def _on_odometry(self, message: Odometry) -> None:
        if message.header.frame_id != self._expected_frame:
            self._fail_closed(
                "odometry frame mismatch: "
                f"{message.header.frame_id!r} != {self._expected_frame!r}"
            )
            return
        position = message.pose.pose.position
        state = PlanarState(position.x, position.y)
        yaw_rad = _yaw_from_odometry(message)
        reason = CANONICAL_SCENARIO.terminal_reason(state)
        if reason is not None:
            self._publish_stop()
            self._publish_certificate_facts(
                odometry=message,
                state=state,
                yaw_rad=yaw_rad,
                nominal_action=PlanarAction(0.0, 0.0),
                command_linear_mps=0.0,
                command_angular_radps=0.0,
                saturation_applied=False,
                terminal_reason=reason,
            )
            self._last_odometry_time = self.get_clock().now()
            return
        action = self._controller.act(state)
        if not CANONICAL_SYSTEM.action_is_admissible(
            action,
            tolerance=1.0e-12,
        ):
            self._fail_closed("frozen controller violated the S1 action disk")
            return
        tracked = self._tracker.command(
            action,
            heading_rad=yaw_rad,
        )
        command = Twist()
        command.linear.x = tracked.linear_mps
        command.angular.z = tracked.angular_radps
        self._command_publisher.publish(command)
        self._publish_certificate_facts(
            odometry=message,
            state=state,
            yaw_rad=yaw_rad,
            nominal_action=action,
            command_linear_mps=tracked.linear_mps,
            command_angular_radps=tracked.angular_radps,
            saturation_applied=tracked.saturation_applied,
            terminal_reason=None,
        )
        self._last_odometry_time = self.get_clock().now()

    def _on_watchdog(self) -> None:
        if self._last_odometry_time is None:
            self._publish_stop()
            return
        age_s = (
            self.get_clock().now() - self._last_odometry_time
        ).nanoseconds / 1.0e9
        if age_s > self._stale_timeout_s:
            self._fail_closed("odometry stale; publishing zero command")

    def destroy_node(self) -> bool:
        self._publish_stop()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PolynomialTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
