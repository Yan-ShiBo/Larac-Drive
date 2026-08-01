"""Self-contained SDF projection for the Gazebo Classic road experiment."""

from __future__ import annotations

from math import pi
from pathlib import Path
from xml.sax.saxutils import escape

from cbf_rover.domain.dynamic_road import DynamicRoadScenario

from .differential_road_contract import GazeboDifferentialRoadContract


def _box_inertia(
    *,
    mass_kg: float,
    length_m: float,
    width_m: float,
    height_m: float,
) -> tuple[float, float, float]:
    return (
        mass_kg * (width_m**2 + height_m**2) / 12.0,
        mass_kg * (length_m**2 + height_m**2) / 12.0,
        mass_kg * (length_m**2 + width_m**2) / 12.0,
    )


def _wheel_inertia(
    *,
    mass_kg: float,
    radius_m: float,
    width_m: float,
) -> tuple[float, float, float]:
    radial = mass_kg * (3.0 * radius_m**2 + width_m**2) / 12.0
    axial = mass_kg * radius_m**2 / 2.0
    return radial, radial, axial


def _obstacle_sdf(
    *,
    obstacle_id: str,
    x_m: float,
    y_m: float,
    radius_m: float,
) -> str:
    name = escape(obstacle_id)
    height_m = 0.50
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x_m:.17g} {y_m:.17g} {height_m / 2.0:.17g} 0 0 0</pose>
      <link name="body">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{radius_m:.17g}</radius>
              <length>{height_m:.17g}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{radius_m:.17g}</radius>
              <length>{height_m:.17g}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.72 0.10 0.08 1</ambient>
            <diffuse>0.86 0.16 0.12 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _indoor_room_sdf(
    *,
    x_min_m: float,
    x_max_m: float,
    half_width_m: float,
    wall_height_m: float,
    wall_thickness_m: float,
) -> str:
    room_length_m = x_max_m - x_min_m
    room_width_m = 2.0 * half_width_m
    center_x_m = (x_min_m + x_max_m) / 2.0
    wall_z_m = wall_height_m / 2.0
    wall_material = """
          <material>
            <ambient>0.34 0.36 0.37 1</ambient>
            <diffuse>0.48 0.49 0.48 1</diffuse>
          </material>"""
    return f"""
    <model name="indoor_room_shell">
      <static>true</static>
      <link name="walls">
        <collision name="north_wall_collision">
          <pose>{center_x_m:.17g} {half_width_m:.17g} {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{room_length_m:.17g} {wall_thickness_m:.17g} {wall_height_m:.17g}</size></box></geometry>
        </collision>
        <visual name="north_wall_visual">
          <pose>{center_x_m:.17g} {half_width_m:.17g} {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{room_length_m:.17g} {wall_thickness_m:.17g} {wall_height_m:.17g}</size></box></geometry>{wall_material}
        </visual>
        <collision name="south_wall_collision">
          <pose>{center_x_m:.17g} {-half_width_m:.17g} {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{room_length_m:.17g} {wall_thickness_m:.17g} {wall_height_m:.17g}</size></box></geometry>
        </collision>
        <visual name="south_wall_visual">
          <pose>{center_x_m:.17g} {-half_width_m:.17g} {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{room_length_m:.17g} {wall_thickness_m:.17g} {wall_height_m:.17g}</size></box></geometry>{wall_material}
        </visual>
        <collision name="west_wall_collision">
          <pose>{x_min_m:.17g} 0 {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{wall_thickness_m:.17g} {room_width_m:.17g} {wall_height_m:.17g}</size></box></geometry>
        </collision>
        <visual name="west_wall_visual">
          <pose>{x_min_m:.17g} 0 {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{wall_thickness_m:.17g} {room_width_m:.17g} {wall_height_m:.17g}</size></box></geometry>{wall_material}
        </visual>
        <collision name="east_wall_collision">
          <pose>{x_max_m:.17g} 0 {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{wall_thickness_m:.17g} {room_width_m:.17g} {wall_height_m:.17g}</size></box></geometry>
        </collision>
        <visual name="east_wall_visual">
          <pose>{x_max_m:.17g} 0 {wall_z_m:.17g} 0 0 0</pose>
          <geometry><box><size>{wall_thickness_m:.17g} {room_width_m:.17g} {wall_height_m:.17g}</size></box></geometry>{wall_material}
        </visual>
      </link>
    </model>
    <light name="room_key_light" type="point">
      <pose>{center_x_m - room_length_m * 0.22:.17g} 0 2.6 0 0 0</pose>
      <diffuse>0.88 0.86 0.79 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <attenuation><range>18</range><constant>0.45</constant><linear>0.03</linear><quadratic>0.004</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
    <light name="room_fill_light" type="point">
      <pose>{center_x_m + room_length_m * 0.22:.17g} 0 2.6 0 0 0</pose>
      <diffuse>0.72 0.78 0.82 1</diffuse>
      <specular>0.08 0.08 0.08 1</specular>
      <attenuation><range>18</range><constant>0.45</constant><linear>0.03</linear><quadratic>0.004</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>"""


def _front_camera_sensor_sdf(
    *,
    contract: GazeboDifferentialRoadContract,
    camera_output_path: str | Path | None,
    camera_queue_directory: str | Path | None,
) -> str:
    camera = contract.front_camera
    if camera is None:
        return ""
    if camera_output_path is None:
        raise ValueError("front camera requires an explicit output path")
    pose = " ".join(f"{value:.17g}" for value in camera.pose_on_base_link)
    queue_directory_sdf = ""
    if (
        contract.camera_recording is not None
        and camera_queue_directory is not None
    ):
        queue_directory_sdf = (
            "\n            <queue_directory>"
            f"{escape(str(Path(camera_queue_directory)))}"
            "</queue_directory>"
        )
    return f"""
        <sensor name="{escape(camera.sensor_name)}" type="camera">
          <pose>{pose}</pose>
          <always_on>true</always_on>
          <update_rate>{camera.update_rate_hz:.17g}</update_rate>
          <visualize>false</visualize>
          <camera name="{escape(camera.sensor_name)}_optics">
            <horizontal_fov>{camera.horizontal_fov_rad:.17g}</horizontal_fov>
            <image>
              <width>{camera.image_width}</width>
              <height>{camera.image_height}</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>{camera.near_clip_m:.17g}</near>
              <far>{camera.far_clip_m:.17g}</far>
            </clip>
          </camera>
          <plugin name="cbf_rover_front_camera_writer" filename="{escape(camera.plugin)}">
            <output_path>{escape(str(Path(camera_output_path)))}</output_path>{queue_directory_sdf}
          </plugin>
        </sensor>"""


def render_gazebo_classic_world(
    *,
    contract: GazeboDifferentialRoadContract,
    scenario: DynamicRoadScenario,
    camera_output_path: str | Path | None = None,
    camera_queue_directory: str | Path | None = None,
) -> str:
    """Render one offline, model-database-independent Gazebo 11 world."""

    runtime = contract.runtime
    rover = contract.rover
    geometry = scenario.geometry
    start = geometry.start
    base_ixx, base_iyy, base_izz = _box_inertia(
        mass_kg=rover.base_mass_kg,
        length_m=rover.base_length_m,
        width_m=rover.base_width_m,
        height_m=rover.base_height_m,
    )
    wheel_ixx, wheel_iyy, wheel_izz = _wheel_inertia(
        mass_kg=rover.wheel_mass_kg,
        radius_m=rover.wheel_radius_m,
        width_m=rover.wheel_width_m,
    )
    wheel_y = rover.wheel_separation_m / 2.0
    base_z = rover.wheel_radius_m + rover.base_height_m / 2.0
    caster_z = rover.rear_caster_radius_m
    road_length = geometry.x_max_m - geometry.x_min_m
    road_width = geometry.physical_y_max_m - geometry.physical_y_min_m
    road_center_x = (geometry.x_max_m + geometry.x_min_m) / 2.0
    road_center_y = (
        geometry.physical_y_max_m + geometry.physical_y_min_m
    ) / 2.0
    scene = contract.visual_scene
    room_x_min = geometry.x_min_m - scene.room_margin_x_m
    room_x_max = geometry.x_max_m + scene.room_margin_x_m
    ground_width = (
        2.0 * scene.room_half_width_m
        if scene.layout == "indoor_room"
        else 10.0
    )
    surround_ambient = (
        "0.18 0.19 0.20 1"
        if scene.layout == "indoor_room"
        else "0.20 0.28 0.20 1"
    )
    surround_diffuse = (
        "0.27 0.28 0.29 1"
        if scene.layout == "indoor_room"
        else "0.24 0.34 0.24 1"
    )
    room_model = (
        ""
        if scene.layout != "indoor_room"
        else _indoor_room_sdf(
            x_min_m=room_x_min,
            x_max_m=room_x_max,
            half_width_m=scene.room_half_width_m,
            wall_height_m=scene.wall_height_m,
            wall_thickness_m=scene.wall_thickness_m,
        )
    )
    camera_sensor = _front_camera_sensor_sdf(
        contract=contract,
        camera_output_path=camera_output_path,
        camera_queue_directory=camera_queue_directory,
    )
    obstacle_models = "".join(
        _obstacle_sdf(
            obstacle_id=obstacle.obstacle_id,
            x_m=obstacle.center.x_m,
            y_m=obstacle.center.y_m,
            radius_m=obstacle.physical_radius_m,
        )
        for obstacle in scenario.obstacles
    )
    state_plugin = (
        ""
        if runtime.state_plugin is None
        else f"""
      <plugin name="cbf_rover_timestamped_state" filename="{escape(runtime.state_plugin)}"/>
"""
    )
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="{escape(runtime.world_name)}">
    <gravity>0 0 -9.81</gravity>
    <physics name="fixed_step_ode" type="ode">
      <max_step_size>{runtime.physics_step_s:.17g}</max_step_size>
      <real_time_update_rate>0</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>{runtime.solver_iterations}</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>
    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.72 0.80 0.90 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -1</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="ground">
        <collision name="collision">
          <pose>{road_center_x:.17g} 0 -0.025 0 0 0</pose>
          <geometry><box><size>{road_length + 2.0 * scene.room_margin_x_m:.17g} {ground_width:.17g} 0.05</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="surround">
          <pose>{road_center_x:.17g} 0 -0.025 0 0 0</pose>
          <geometry><box><size>{road_length + 2.0 * scene.room_margin_x_m:.17g} {ground_width:.17g} 0.05</size></box></geometry>
          <material>
            <ambient>{surround_ambient}</ambient>
            <diffuse>{surround_diffuse}</diffuse>
          </material>
        </visual>
        <visual name="road">
          <pose>{road_center_x:.17g} {road_center_y:.17g} 0.001 0 0 0</pose>
          <geometry><box><size>{road_length:.17g} {road_width:.17g} 0.002</size></box></geometry>
          <material>
            <ambient>0.20 0.22 0.25 1</ambient>
            <diffuse>0.28 0.30 0.34 1</diffuse>
          </material>
        </visual>
        <visual name="center_line">
          <pose>{road_center_x:.17g} 0 0.003 0 0 0</pose>
          <geometry><box><size>{road_length:.17g} 0.025 0.002</size></box></geometry>
          <material>
            <ambient>0.95 0.82 0.20 1</ambient>
            <diffuse>0.98 0.86 0.24 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
{room_model}
{obstacle_models}

    <model name="{escape(runtime.model_name)}">
      <pose>{start.x_m:.17g} {start.y_m:.17g} 0 0 0 {contract.base.initial_heading_rad:.17g}</pose>
      <self_collide>false</self_collide>
      <link name="base_link">
        <pose>0 0 {base_z:.17g} 0 0 0</pose>
        <inertial>
          <mass>{rover.base_mass_kg:.17g}</mass>
          <inertia>
            <ixx>{base_ixx:.17g}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{base_iyy:.17g}</iyy>
            <iyz>0</iyz>
            <izz>{base_izz:.17g}</izz>
          </inertia>
        </inertial>
        <collision name="base_collision">
          <geometry>
            <box>
              <size>{rover.base_length_m:.17g} {rover.base_width_m:.17g} {rover.base_height_m:.17g}</size>
            </box>
          </geometry>
        </collision>
        <visual name="base_visual">
          <geometry>
            <box>
              <size>{rover.base_length_m:.17g} {rover.base_width_m:.17g} {rover.base_height_m:.17g}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.92 0.35 0.08 1</ambient>
            <diffuse>0.96 0.40 0.10 1</diffuse>
          </material>
        </visual>
{camera_sensor}
      </link>

      <link name="left_wheel_link">
        <pose>{rover.wheel_axle_x_m:.17g} {wheel_y:.17g} {rover.wheel_radius_m:.17g} {pi / 2.0:.17g} 0 0</pose>
        <inertial>
          <mass>{rover.wheel_mass_kg:.17g}</mass>
          <inertia>
            <ixx>{wheel_ixx:.17g}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{wheel_iyy:.17g}</iyy>
            <iyz>0</iyz>
            <izz>{wheel_izz:.17g}</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{rover.wheel_radius_m:.17g}</radius>
              <length>{rover.wheel_width_m:.17g}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{rover.wheel_radius_m:.17g}</radius>
              <length>{rover.wheel_width_m:.17g}</length>
            </cylinder>
          </geometry>
          <material><ambient>0.04 0.04 0.05 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
        </visual>
      </link>
      <joint name="left_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>left_wheel_link</child>
        <axis>
          <xyz>0 1 0</xyz>
          <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
      </joint>

      <link name="right_wheel_link">
        <pose>{rover.wheel_axle_x_m:.17g} {-wheel_y:.17g} {rover.wheel_radius_m:.17g} {-pi / 2.0:.17g} 0 0</pose>
        <inertial>
          <mass>{rover.wheel_mass_kg:.17g}</mass>
          <inertia>
            <ixx>{wheel_ixx:.17g}</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>{wheel_iyy:.17g}</iyy>
            <iyz>0</iyz>
            <izz>{wheel_izz:.17g}</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{rover.wheel_radius_m:.17g}</radius>
              <length>{rover.wheel_width_m:.17g}</length>
            </cylinder>
          </geometry>
          <surface>
            <friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{rover.wheel_radius_m:.17g}</radius>
              <length>{rover.wheel_width_m:.17g}</length>
            </cylinder>
          </geometry>
          <material><ambient>0.04 0.04 0.05 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
        </visual>
      </link>
      <joint name="right_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>right_wheel_link</child>
        <axis>
          <xyz>0 1 0</xyz>
          <use_parent_model_frame>true</use_parent_model_frame>
        </axis>
      </joint>

      <link name="rear_caster_link">
        <pose>{rover.rear_caster_x_m:.17g} 0 {caster_z:.17g} 0 0 0</pose>
        <inertial>
          <mass>0.10</mass>
          <inertia>
            <ixx>0.000064</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.000064</iyy>
            <iyz>0</iyz>
            <izz>0.000064</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><sphere><radius>{rover.rear_caster_radius_m:.17g}</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0</mu><mu2>0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><sphere><radius>{rover.rear_caster_radius_m:.17g}</radius></sphere></geometry>
          <material><ambient>0.04 0.04 0.05 1</ambient><diffuse>0.08 0.08 0.09 1</diffuse></material>
        </visual>
      </link>
      <joint name="rear_caster_joint" type="fixed">
        <parent>base_link</parent>
        <child>rear_caster_link</child>
      </joint>

      <plugin name="cbf_rover_diff_drive" filename="{escape(runtime.command_plugin)}">
        <body>base_link</body>
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
      </plugin>
{state_plugin}
    </model>
  </world>
</sdf>
"""
