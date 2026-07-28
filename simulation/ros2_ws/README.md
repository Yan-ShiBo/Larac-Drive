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
- `cbf_rover_runtime`: ROS 2 runtime-adapter boundary. Its only current
  executable is a disabled-by-default message relay from an external stamped
  command to `cmd_vel`. It does not validate that command and is not a
  certificate evaluator, controller, action modifier, guard, or complete
  certificate runtime.

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
launch file deliberately does not select or embed obstacle geometry. A future
application-level scenario runner must resolve the repository's root
`ScenarioSpec`, project its geometry into Gazebo obstacle strings, and inject
that projection into `cbf_rover_sim`'s `obstacle_spawner`.

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

Generated `build/`, `install/`, and `log/` directories are workspace output and
must not be committed.
