# cbf_rover_runtime

This package is the ROS 2 adapter boundary for simulation runtime components.
It does not own certificate mathematics, synthesis, scenarios, or
physical-hardware support.

Its only current executable, `external_command_relay`, is an integration-only
message adapter. It converts an externally produced `TwistStamped` on
`/external_command` to `Twist` on `/cmd_vel`. It is disabled by default and is
**not** the project's certificate runtime: it does not evaluate a certificate,
validate an action or deadline, select or modify an action, implement a
controller, load an artifact, or establish a safety claim.

The relay's YAML file contains only the two topic parameters it actually
consumes. Model-manifest validation belongs to the learned Python baseline;
certificate-artifact compatibility belongs to the future application runtime.
Neither concern is duplicated in this ROS message adapter.

The `external_command_relay.launch.py` argument
`start_external_command_relay` defaults to `false`. If explicitly enabled, the
relay forwards every received command without judging whether it is safe. An
orderly shutdown publishes one zero `Twist`, but this is not a watchdog or a
fail-safe guarantee.

Future certificate evaluation belongs in this package only after the Python
application/runtime contract exists. Do not treat this message adapter as that
implementation.
