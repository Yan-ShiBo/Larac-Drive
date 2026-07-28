"""Spawn deterministic box obstacles through Gazebo's SpawnEntity service."""

from __future__ import annotations

from dataclasses import dataclass
from xml.sax.saxutils import escape

import rclpy
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node


@dataclass(frozen=True)
class BoxObstacle:
    name: str
    x: float
    y: float
    size_x: float
    size_y: float
    size_z: float

    @classmethod
    def parse(cls, value: str) -> "BoxObstacle":
        fields = [field.strip() for field in value.split(",")]
        if len(fields) != 6:
            raise ValueError(
                "expected name,x,y,size_x,size_y,size_z; "
                f"received {len(fields)} fields"
            )
        name = fields[0]
        if not name:
            raise ValueError("obstacle name must not be empty")
        numbers = [float(field) for field in fields[1:]]
        if any(size <= 0.0 for size in numbers[2:]):
            raise ValueError("obstacle dimensions must be positive")
        return cls(name, *numbers)

    def sdf(self) -> str:
        name = escape(self.name)
        return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <collision name="collision">
        <geometry>
          <box><size>{self.size_x} {self.size_y} {self.size_z}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>{self.size_x} {self.size_y} {self.size_z}</size></box>
        </geometry>
        <material>
          <ambient>0.72 0.13 0.12 1</ambient>
          <diffuse>0.82 0.18 0.16 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class ObstacleSpawner(Node):
    """Spawn a ScenarioSpec projection supplied through ROS parameters."""

    def __init__(self) -> None:
        super().__init__("obstacle_spawner")
        self.declare_parameter("obstacles", [])
        self.declare_parameter("service_timeout_sec", 30.0)
        self._client = self.create_client(SpawnEntity, "/spawn_entity")

    def spawn_all(self) -> bool:
        encoded_obstacles = self.get_parameter("obstacles").value
        if not encoded_obstacles:
            self.get_logger().info(
                "no obstacles supplied; waiting for a resolved ScenarioSpec "
                "projection is an application-level responsibility"
            )
            return True

        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"/spawn_entity was unavailable after {timeout:.1f} seconds"
            )
            return False

        success = True
        for encoded in encoded_obstacles:
            try:
                obstacle = BoxObstacle.parse(str(encoded))
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f"invalid obstacle {encoded!r}: {exc}")
                success = False
                continue

            request = SpawnEntity.Request()
            request.name = obstacle.name
            request.xml = obstacle.sdf()
            request.robot_namespace = ""
            request.initial_pose.position.x = obstacle.x
            request.initial_pose.position.y = obstacle.y
            request.initial_pose.position.z = obstacle.size_z / 2.0
            request.reference_frame = "world"

            future = self._client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error(
                    f"failed to call /spawn_entity for {obstacle.name}"
                )
                success = False
                continue

            response = future.result()
            if response.success:
                self.get_logger().info(f"spawned obstacle {obstacle.name}")
            else:
                self.get_logger().error(
                    f"Gazebo rejected {obstacle.name}: {response.status_message}"
                )
                success = False
        return success


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        node.spawn_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()
