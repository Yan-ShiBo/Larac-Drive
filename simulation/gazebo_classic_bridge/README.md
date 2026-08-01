# Gazebo Classic transport bridge

This directory contains the standalone executable and model plugin needed to
connect the Python dynamic-road controller and online CBF to Gazebo Classic
without ROS.

The executable:

- connects to one explicitly isolated Gazebo master;
- waits for the isolated world namespace and bounded plugin-topic connections;
- subscribes to timestamped model state from the companion model plugin and to
  world statistics;
- publishes `[linear_mps, angular_radps]` to Gazebo Classic's built-in
  `DiffDrivePlugin`;
- advances a paused world by an exact number of physics iterations; and
- exchanges line-oriented state and command records with the Python adapter.

The bridge and plugins do not implement a controller, CBF, certificate
evaluator, scenario generator, acceptance gate, dashboard, or certificate
projection. Gazebo/Ogre renders the RGB sensor; the Python adapter in
`src/cbf_rover` constructs the observer-only dashboard and overlay.

Build on the server:

```bash
cmake -S simulation/gazebo_classic_bridge \
  -B /absolute/external/build/directory
cmake --build /absolute/external/build/directory --parallel
```

Protocol version `GZCB1`:

```text
READY GZCB1 <world> <model>
STATE <sim_time_s> <iterations> <x_m> <y_m> <z_m> <roll_rad> <pitch_rad> <yaw_rad>
STEP <linear_mps> <angular_radps> <physics_iterations>
GET
STOP
ERROR <reason>
```

The world must start paused. Each `STEP` publishes one command, advances the
world by the requested iteration count, and emits the measured terminal state.
The companion `libcbf_rover_gazebo_state_plugin.so` returns the measured model
pose stamped with both `World::SimTime()` and the exact
`World::Iterations()` value on request. The iteration is carried on the
private plugin topic as the pose message ID. For each step, the bridge computes
the target from the preceding timestamped pose, advances the world, and polls
the plugin until a pose from that target iteration is received. A stale or
dropped response is retried without advancing physics. The lower-frequency
`world_stats` remains diagnostic telemetry but is not a startup or per-step
completion gate. This prevents an asynchronous callback from exposing a
previous physics pose, avoids missing the low-frequency initial statistics
message during subscription, and does not add the statistics publication
interval to every control step. The plugin defers transport initialization
until the first world update so model loading cannot race world namespace
registration. At startup, the bridge performs exactly one zero-command
bootstrap physics iteration and then obtains its terminal pose through the same
bounded timestamped-pose request/response handshake. All protocol output uses
classic locale-independent decimal notation.

This adapter provides empirical Gazebo behavior only. It does not establish a
formal tracking-error bound or a real-vehicle claim.

## Front RGB camera plugin

`libcbf_rover_gazebo_camera_plugin.so` is a Gazebo `CameraSensor` plugin used by
V14 and V15. It receives actual Gazebo/Ogre RGB frames and publishes each frame as one
24-bit BMP through a private temporary file followed by an atomic rename. The
same BMP contains a small trailer with the Gazebo simulation seconds,
nanoseconds, and a monotonic camera-frame counter, so pixels and capture time
cannot be paired across different files.

The Python observer validates the BMP header and timestamp trailer, converts
the frame to PNG, selects the nearest `0.05 s` measured rover state, and draws
the active lifted distance-CBF boundary with a ground-plane pinhole projection.
The manifest records frame coverage and maximum time-alignment error. One
keyframe is retained only after each newly activated obstacle is also active in
the aligned camera state.

V15 additionally configures a private callback queue. The plugin atomically
publishes one immutable, counter-named BMP for every camera callback while
retaining the latest-frame path for the live preview. The Python observer drains
the queue into a timestamped PNG sequence and JSONL index. The accepted V15
attempt consumed contiguous counters `1..410` with zero missing or invalid
frames. Its downstream renderer maps those callbacks onto a fixed 10 fps
simulation-time grid and produced the accepted 392-frame, 39.2-second delivery
MP4. The camera plugin itself does not encode video.

The verified headless renderer for this path is Xvfb. The filesystem frame is
not a ROS image topic, and neither the image nor the rendered overlay is an
input to the controller or online CBF. This is therefore not a visual-perception
safety loop.

## Step-latency diagnostic

Set `CBF_ROVER_GAZEBO_BRIDGE_TIMING=1` to emit one `TIMING` record per `STEP`
to `bridge.stderr.log`. The record separates velocity publication, world
control publication, terminal-state synchronization, state-request
publication, and protocol emission. The switch is disabled by default and
does not alter `GZCB1` standard output.

For a bounded bridge / physics-only probe, use:

```bash
python simulation/gazebo_classic_bridge/tools/profile_step_latency.py \
  --config configs/experiments/dynamic_road_gazebo_classic_v7.yaml \
  --world /absolute/prior/result/dynamic_road_gazebo.world \
  --bridge /absolute/build/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/diagnostic/directory \
  --master-uri http://127.0.0.1:11470 \
  --steps 20
```

The probe holds a constant differential-drive command and deliberately omits
the controller and CBF. Its JSON output is diagnostic evidence only, never a
safety result.
