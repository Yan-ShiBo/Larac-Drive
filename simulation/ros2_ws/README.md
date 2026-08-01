# CBF-Rover ROS 2 simulation workspace

This workspace contains adapters around the Python research core. It does not
define certificate mathematics, synthesize certificates, choose experiment
scenarios, or contain physical-hardware support.

## Packages

- `cbf_rover_description`: geometry-only Xacro model with lidar and RGB-D
  frames. It has no simulator plugins and no hardware interfaces.
- `cbf_rover_sim`: Gazebo Classic adapter containing the base world,
  simulator-only Xacro plugins, RGB-D sensor, inherited visual ground,
  obstacle model/spawner, and simulator launch file.
- `cbf_rover_runtime`: ROS 2 runtime-adapter boundary. It contains the
  disabled-by-default external relay plus the GZ1 polynomial tracker,
  ScenarioSpec visualizer, and a complete monitor-only certificate simulation
  launch. The q-SBC evaluator publishes facts and RViz markers but never
  modifies controller actions.

Do not create empty interface, visualization, or bringup packages in
anticipation of future work. Add a package only when it owns a concrete runtime
responsibility.

## Build

From this directory on the ROS 2 server:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start simulation

```bash
ros2 launch cbf_rover_sim simulation.launch.py
```

The default `worlds/base.world` contains only the inherited ground and sun. The
base launch deliberately does not select or embed obstacle geometry.
`cbf_rover_runtime/certificate_simulation.launch.py` is the application-level
composition root: it resolves the repository's root `ScenarioSpec`, verifies
the P1/D1 geometry, projects physical obstacles into spawner parameters, starts
the monitor-only tracker, and publishes RViz markers.

Base-link contacts are intended to appear on `/collision`; confirm the Gazebo
Classic bumper plugin and topic behavior on the target ROS 2 server before
using that topic as an experiment metric.

The external command relay is an integration adapter and is disabled by
default. Enable it only when intentionally testing an external command source:

```bash
ros2 launch cbf_rover_runtime external_command_relay.launch.py \
  start_external_command_relay:=true
```

It forwards commands without checking certificate compatibility, freshness,
bounds, or feasibility. Its YAML is the only ROS-specific relay
configuration. End-to-end topic behavior still requires confirmation on the
target ROS 2 server.

Once independently validated controller/certificate artifacts exist, launch
the ScenarioSpec-driven empirical run with:

```bash
ros2 launch cbf_rover_runtime certificate_simulation.launch.py \
  scenario_path:=/absolute/source/configs/scenarios/static_obstacles.yaml \
  controller_path:=/absolute/artifacts/frozen_controller.json \
  certificate_path:=/absolute/artifacts/certificate.json
```

Set `rviz:=true` for the live visual display. Headless validation can instead
record `/cbf_rover/certificate/facts`, `/cbf_rover/certificate/identity`,
`/cbf_rover/barrier_value`, `/odom`, and `/collision`.

Generated `build/`, `install/`, and `log/` directories are workspace output and
must not be committed.
