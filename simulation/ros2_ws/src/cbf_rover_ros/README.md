# cbf_rover_ros

This package is a narrow ROS 2 lifecycle adapter around the research core. It
does not implement CBF logic, select a policy, load Torch, or talk to physical
hardware.

On configuration it:

1. parses the external YAML file given by `method_config`;
2. parses the external YAML file given by `artifact_manifest`;
3. requires `artifact_set.status: ready`;
4. requires verified `rgbd_vae`, `rgbd_mdrnn`, `learned_cbf`, and
   `learned_clf` slots;
5. resolves each slot's `artifact://` URI inside `artifact_root`, then verifies
   the file's declared byte count and SHA-256 digest.

The only manifest schema is the repository-level
`artifacts/manifests/schema.yaml`; do not create a ROS-specific variant. Its
slots are a top-level list keyed by each slot's `id`. An artifact URI has the
form `artifact://checkpoints/example.pt`; it is not a filesystem URL. Absolute
paths, missing files, symlink/path escapes, invalid sizes, and digest mismatches
cause lifecycle configuration to fail.

While active, `lifecycle_bridge` forwards `safe_command` (`TwistStamped`) to
`cmd_vel` (`Twist`). This keeps method execution outside the ROS adapter and
makes the safety-command boundary explicit.

The launch argument `start_bridge` defaults to `false`. When enabled, the node
still starts in the standard unconfigured lifecycle state and must be
configured and activated explicitly. Configuration verifies artifact
provenance but deliberately does not load a model.
