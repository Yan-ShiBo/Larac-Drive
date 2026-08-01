# cbf_rover_runtime

This package is the ROS 2 adapter boundary for simulation runtime components.
It does not own certificate mathematics, synthesis, scenarios, or
physical-hardware support.

`external_command_relay` is an integration-only message adapter. It converts an
externally produced `TwistStamped` on
`/external_command` to `Twist` on `/cmd_vel`. It is disabled by default and is
**not** the project's certificate runtime: it does not evaluate a certificate,
validate an action or deadline, select or modify an action, implement a
controller, load an artifact, or establish a safety claim.

The relay's YAML file contains only the two topic parameters it actually
consumes. Model-manifest validation belongs to the learned Python baseline;
certificate-artifact compatibility is implemented by the core artifact
loaders. Neither concern is duplicated in this relay.

The `external_command_relay.launch.py` argument
`start_external_command_relay` defaults to `false`. If explicitly enabled, the
relay forwards every received command without judging whether it is safe. An
orderly shutdown publishes one zero `Twist`, but this is not a watchdog or a
fail-safe guarantee.

`polynomial_tracker` is the GZ1 empirical adapter. It loads one CV1-frozen
polynomial controller, optionally loads a hash-compatible q-SBC certificate,
reads `/odom`, and maps `[u_x,u_y]` to differential-drive `(v,omega)`. The
certificate must carry an independent `validated` status. Its value is
published as monitor-only telemetry and never modifies the action. The node
fails closed on missing artifacts, certificate/controller mismatch, frame
mismatch, stale odometry, terminal states, or action-disk violations.

Live outputs are deliberately redundant and identity-bound:

- `/cbf_rover/barrier_value` publishes the numeric `W(x)` for simple plotting;
- `/cbf_rover/certificate/facts` publishes a per-odometry JSON fact record with
  state, nominal action, differential-drive command, `W`, `q`, `epsbar`,
  validation/readiness, hashes, and the empirical/monitor-only boundary;
- `/cbf_rover/certificate/identity` is a durable identity record;
- `/cbf_rover/certificate/marker` is an RViz text marker attached to the rover.

The marker's certificate-level relation is not an instantaneous state
classifier, and `q` is not an instantaneous collision probability.

`scenario_visualizer` resolves the root ScenarioSpec, verifies it against the
executable P1/D1 geometry, and publishes physical obstacles, inflated unsafe
sets, initial/goal sets, and the domain boundary from that same snapshot.

The complete server launch is:

```bash
ros2 launch cbf_rover_runtime certificate_simulation.launch.py \
  scenario_path:=/absolute/source/configs/scenarios/static_obstacles.yaml \
  controller_path:=/absolute/artifacts/frozen_controller.json \
  certificate_path:=/absolute/artifacts/certificate.json \
  gazebo_gui:=false rviz:=true
```

GZ1 has explicit actuator saturation but no proved tracking-error bound.
Consequently its Gazebo trajectories are empirical evidence and do not inherit
the exact M1 q-SBC probability bound.
