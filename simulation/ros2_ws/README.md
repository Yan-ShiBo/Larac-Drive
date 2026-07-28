# CBF-Rover ROS 2 simulation workspace

This workspace keeps simulation integration separate from the Python research
core and from external experiment artifacts.

## Packages

- `cbf_rover_description`: geometry-only Xacro model with lidar and RGB-D
  frames. It has no simulator plugins and no hardware interfaces.
- `cbf_rover_gazebo`: Gazebo Classic world, simulator-only Xacro plugins,
  RGB-D sensor, inherited visual ground, obstacle model/spawner, and the
  simulator launch file.
- `cbf_rover_ros`: optional ROS 2 lifecycle bridge. It validates an external
  method configuration and four manifest artifacts, then forwards an already-safe
  command while active. It does not implement a policy or load a model.

## Build

From this directory on the ROS 2 server:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start simulation

```bash
ros2 launch cbf_rover_gazebo simulation.launch.py
```

The default scenario is `worlds/static_obstacles.world`. Base-link contacts are
intended to appear on `/collision`; confirm the Gazebo Classic bumper plugin and
topic behavior on the target ROS 2 server before using that topic as an
experiment metric.

The generic lifecycle bridge is disabled by default. Enable it only when an
external artifact directory and method configuration are available:

```bash
ros2 launch cbf_rover_ros bridge.launch.py \
  start_bridge:=true \
  artifact_root:=/absolute/path/to/artifacts \
  artifact_manifest:=/absolute/path/to/artifact-manifest.yaml \
  method_config:=/absolute/path/to/method.yaml
```

After launch, configure and activate it explicitly through the standard ROS 2
lifecycle services. Configuration succeeds only when the manifest declares a
ready artifact set, all four required slots are verified, every `artifact://`
URI remains inside `artifact_root`, and file byte counts and SHA-256 digests
match. The bridge performs these checks without loading any artifact. This
contract still requires end-to-end confirmation on the target ROS 2 server.

Generated `build/`, `install/`, and `log/` directories are workspace output and
must not be committed.
