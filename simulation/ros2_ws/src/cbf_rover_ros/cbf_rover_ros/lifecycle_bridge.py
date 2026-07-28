"""Lifecycle boundary between ROS 2 transport and an external safe controller."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from sensor_msgs.msg import LaserScan


REQUIRED_ARTIFACT_SLOTS = (
    "rgbd_vae",
    "rgbd_mdrnn",
    "learned_cbf",
    "learned_clf",
)


class CbfRoverLifecycleBridge(LifecycleNode):
    """Validate external inputs and forward already-safe velocity commands."""

    def __init__(self) -> None:
        super().__init__("cbf_rover_bridge")

        self.declare_parameter("artifact_root", "")
        self.declare_parameter("artifact_manifest", "")
        self.declare_parameter("method_config", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("safe_command_topic", "/safe_command")
        self.declare_parameter("command_topic", "/cmd_vel")

        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        safe_command_topic = str(self.get_parameter("safe_command_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)

        self._command_publisher = self.create_lifecycle_publisher(
            Twist, command_topic, 10
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10
        )
        self._odom_subscription = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )
        self._safe_command_subscription = self.create_subscription(
            TwistStamped, safe_command_topic, self._on_safe_command, 10
        )

        self._active = False
        self._method_config: dict[str, Any] | None = None
        self._artifact_manifest: dict[str, Any] | None = None
        self._artifact_paths: dict[str, Path] = {}
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None

    @staticmethod
    def _read_yaml_mapping(path: Path, label: str) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as stream:
            value = yaml.safe_load(stream)
        if not isinstance(value, dict):
            raise ValueError(f"{label} must contain a YAML mapping")
        return value

    @staticmethod
    def _resolve_artifact_uri(artifact_root: Path, uri: str) -> Path:
        prefix = "artifact://"
        if not uri.startswith(prefix):
            raise ValueError("artifact URI must start with artifact://")
        relative_value = uri[len(prefix):]
        if not relative_value or "?" in relative_value or "#" in relative_value:
            raise ValueError("artifact URI must contain one plain relative path")
        relative_path = Path(relative_value)
        if relative_path.is_absolute():
            raise ValueError("artifact URI path must be relative to artifact_root")

        candidate = (artifact_root / relative_path).resolve(strict=True)
        try:
            common = Path(os.path.commonpath([artifact_root, candidate]))
        except ValueError as exc:
            raise ValueError("artifact URI is on a different filesystem") from exc
        if common != artifact_root:
            raise ValueError("artifact URI escapes artifact_root")
        if not candidate.is_file():
            raise ValueError(f"artifact is not a file: {candidate}")
        return candidate

    @staticmethod
    def _sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
        return digest.hexdigest()

    def _validate_artifact_manifest(
        self,
        artifact_root: Path,
        manifest: dict[str, Any],
    ) -> dict[str, Path]:
        artifact_set = manifest.get("artifact_set")
        if not isinstance(artifact_set, dict):
            raise ValueError("artifact_manifest.artifact_set must be a mapping")
        if artifact_set.get("status") != "ready":
            raise ValueError("artifact_set.status must be ready")

        raw_slots = manifest.get("slots")
        if not isinstance(raw_slots, list):
            raise ValueError("artifact_manifest.slots must be a list")
        slots: dict[str, dict[str, Any]] = {}
        for slot in raw_slots:
            if not isinstance(slot, dict) or not isinstance(slot.get("id"), str):
                raise ValueError("every artifact slot must be a mapping with an id")
            slot_id = slot["id"]
            if slot_id in slots:
                raise ValueError(f"duplicate artifact slot id: {slot_id}")
            slots[slot_id] = slot

        resolved_paths: dict[str, Path] = {}
        for slot_name in REQUIRED_ARTIFACT_SLOTS:
            slot = slots.get(slot_name)
            if not isinstance(slot, dict):
                raise ValueError(f"required artifact slot is missing: {slot_name}")
            if slot.get("status") != "verified":
                raise ValueError(f"{slot_name}.status must be verified")
            provenance = slot.get("provenance")
            if not isinstance(provenance, dict) or not all(
                provenance.get(key)
                for key in (
                    "source_type",
                    "training_run_id",
                    "source_revision",
                    "created_at_utc",
                    "dataset_manifest_uri",
                )
            ):
                raise ValueError(f"{slot_name}.provenance is incomplete")
            compatibility = slot.get("compatibility")
            if not isinstance(compatibility, dict) or not all(
                compatibility.get(key)
                for key in (
                    "framework",
                    "framework_version",
                    "architecture",
                    "loader",
                    "input_contract",
                    "output_contract",
                )
            ):
                raise ValueError(f"{slot_name}.compatibility is incomplete")

            uri = slot.get("uri")
            if not isinstance(uri, str):
                raise ValueError(f"{slot_name}.uri must be a string")
            path = self._resolve_artifact_uri(artifact_root, uri)

            expected_bytes = slot.get("bytes")
            if (
                not isinstance(expected_bytes, int)
                or isinstance(expected_bytes, bool)
                or expected_bytes < 0
            ):
                raise ValueError(f"{slot_name}.bytes must be a non-negative integer")
            actual_bytes = path.stat().st_size
            if actual_bytes != expected_bytes:
                raise ValueError(
                    f"{slot_name} byte count mismatch: "
                    f"expected {expected_bytes}, found {actual_bytes}"
                )

            expected_sha256 = slot.get("sha256")
            if (
                not isinstance(expected_sha256, str)
                or len(expected_sha256) != 64
                or any(
                    character not in "0123456789abcdef"
                    for character in expected_sha256
                )
            ):
                raise ValueError(
                    f"{slot_name}.sha256 must be 64 lowercase hexadecimal digits"
                )
            actual_sha256 = self._sha256(path)
            if actual_sha256 != expected_sha256:
                raise ValueError(f"{slot_name} SHA-256 mismatch")

            resolved_paths[slot_name] = path
        return resolved_paths

    def _load_external_configuration(
        self,
    ) -> tuple[dict[str, Any], dict[str, Any], dict[str, Path]]:
        root_value = str(self.get_parameter("artifact_root").value).strip()
        if not root_value:
            raise ValueError("artifact_root is required")
        artifact_root = Path(root_value).expanduser().resolve(strict=True)
        if not artifact_root.is_dir():
            raise ValueError(f"artifact_root is not a directory: {artifact_root}")

        config_value = str(self.get_parameter("method_config").value).strip()
        if not config_value:
            raise ValueError("method_config is required")
        method_config_path = Path(config_value).expanduser().resolve(strict=True)
        if not method_config_path.is_file():
            raise ValueError(
                f"method_config is not a file: {method_config_path}"
            )
        method_config = self._read_yaml_mapping(
            method_config_path, "method_config"
        )
        availability = method_config.get("availability")
        if not isinstance(availability, dict) or availability.get("enabled") is not True:
            raise ValueError("method_config availability.enabled must be true")

        manifest_value = str(
            self.get_parameter("artifact_manifest").value
        ).strip()
        if not manifest_value:
            raise ValueError("artifact_manifest is required")
        manifest_path = Path(manifest_value).expanduser().resolve(strict=True)
        if not manifest_path.is_file():
            raise ValueError(f"artifact_manifest is not a file: {manifest_path}")
        artifact_manifest = self._read_yaml_mapping(
            manifest_path, "artifact_manifest"
        )
        artifact_paths = self._validate_artifact_manifest(
            artifact_root, artifact_manifest
        )
        return method_config, artifact_manifest, artifact_paths

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            (
                self._method_config,
                self._artifact_manifest,
                self._artifact_paths,
            ) = (
                self._load_external_configuration()
            )
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(f"bridge configuration rejected: {exc}")
            self._method_config = None
            self._artifact_manifest = None
            self._artifact_paths = {}
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            "external method configuration and four manifest artifacts verified; "
            "no artifact was loaded"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        result = super().on_activate(state)
        if result == TransitionCallbackReturn.SUCCESS:
            self._active = True
            self.get_logger().info("safe-command forwarding is active")
        return result

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        self._command_publisher.publish(Twist())
        result = super().on_deactivate(state)
        self.get_logger().info("safe-command forwarding is inactive")
        return result

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        self._method_config = None
        self._artifact_manifest = None
        self._artifact_paths = {}
        self._latest_scan = None
        self._latest_odom = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    def _on_scan(self, message: LaserScan) -> None:
        self._latest_scan = message

    def _on_odom(self, message: Odometry) -> None:
        self._latest_odom = message

    def _on_safe_command(self, message: TwistStamped) -> None:
        if not self._active:
            return
        self._command_publisher.publish(message.twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CbfRoverLifecycleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
