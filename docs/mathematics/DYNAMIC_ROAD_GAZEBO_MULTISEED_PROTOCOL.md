# Dynamic-road Gazebo Classic multi-seed protocol

## Material Passport

- Protocol ID: `GZ5_DISCOVERY_CALIBRATED_MULTISEED_V4`
- Bound episode contract: `GZ5_DISCOVERY_CALIBRATED_AUDITED_STARTUP_CBF_LIVE_DISPLAY_V13`
- Replay contract: `GZ5_DISCOVERY_CALIBRATED_REPLAY_VALIDATION_V8`
- Authoritative config: `configs/experiments/dynamic_road_gazebo_classic_multiseed_v4.yaml`
- Freeze status: frozen before V4 execution
- Execution status: discovery `32/32`; held-out validation `32/32`; replay `accepted`
- Claim class: empirical Gazebo Classic finite-catalog validation
- Explicit exclusions: no sampling-probability, q-SBC, ROS2, or real-vehicle claim

## 1. Objective

This protocol tests whether one frozen measured-state online CBF and
differential-drive controller pass every five-obstacle lane pattern in Gazebo
Classic 11 ODE while publishing a read-only runtime certificate display.

The protocol also separates three concerns that must not be conflated:

1. discovery may calibrate an empirical tracking envelope;
2. held-out validation may evaluate that frozen envelope but may not tune it;
3. replay may assess bounded physical repeatability but may not turn empirical
   Gazebo evidence into a formal theorem.

## 2. Catalog construction

Each road contains five seeded obstacles whose lane choices form one five-bit
pattern. The 32 patterns range from `00000` through `11111`.

Two non-overlapping catalogs are explicit in the V4 YAML:

- discovery: the first ascending seed for each lane pattern in the frozen
  differential-drive discovery interval;
- validation: the first ascending seed for each lane pattern in the frozen
  held-out interval.

Selection reads only `seed` and `lane_pattern`. It never reads collision,
barrier, tracking, controller, termination, or display outcomes. Each source
`episode_metrics.jsonl` SHA-256 is bound in the config, and the 32 members are
stored explicitly in ascending pattern order.

This is stratified finite-catalog coverage, not independent random sampling.
No Clopper-Pearson or other success-probability bound is permitted.

## 3. Discovery-calibrated tracking support

V10 showed that the earlier `0.04 m / 0.10 rad` tracking support could cause a
safe 100-second deadlock. V12 then evaluated a tighter `0.016 m / 0.055 rad`
support over the complete 32-pattern discovery catalog. Its worst heading
residual was `0.05563104052672907 rad`, so the V12 batch was retained as a
31/32 diagnostic rather than accepted.

V13 freezes the outward-rounded empirical envelope:

```text
coordinate position half-width = 0.016 m
heading error bound             = 0.065 rad
lookahead heading displacement  = 2 * 0.15 * sin(0.065 / 2)
                                = 0.009748283684395426 m
base support                    = 0.005421809086151014 m
effective coordinate support   = 0.03117009277054644 m
```

No V13 threshold was changed after validation observations. This envelope is
calibrated and hard-gated empirical support, not a formal tracking-error bound.

## 4. Per-episode contract

Every member uses V13 and the same Release bridge/plugin build:

- Gazebo Classic 11, ODE, headless paused fixed-step;
- `0.001 s` physics step and 50 iterations per `0.05 s` control step;
- timestamped measured pose and exact physics iteration from the state plugin;
- frozen nominal controller and measured-state lifted online CBF;
- no action clipping after CBF projection;
- V13 empirical support `0.016 m / 0.065 rad`;
- loopback-only, observer-only live dashboard;
- wall-clock pacing at 1x simulated time;
- no GIF rendering during the batch.

Worlds, obstacle plans, traces, runtime logs, dashboard snapshots, startup
attempts, and manifests are retained with hashes.

## 5. Audited startup policy

Gazebo may be attempted at most three times before an initial state exists.
Each attempt has its own bridge and `gzserver` logs. `startup_attempts.json`
records status, elapsed time, log hashes, selected attempt, and whether any
control frame had started.

The retry boundary is strict:

- `failed_before_initial_state` may start the next bounded attempt;
- after initial state selection, the control trajectory is executed once;
- no failure after control begins may be retried or replaced;
- the batch gate `no_control_retries_after_initial_state` must be true.

This policy handles Gazebo namespace/bootstrap failures without selecting on
controller outcomes.

## 6. Aggregate gates

Held-out validation is `provisional_success` only if:

1. all 32 episodes reach the finish line and pass every V13 single-run gate;
2. all 32 lane patterns occur exactly once;
3. collision, road-departure, and timeout episode counts are zero;
4. lifted barrier, CBF residual, and measured contraction residual are at
   least `-1e-9` in their declared units;
5. mapping residual is at most `1e-12 m/s`;
6. position and heading residuals are at most `0.016 m` and `0.065 rad`;
7. absolute roll and pitch are at most `0.05 rad`;
8. `v <= 0.5000000001 m/s` and `|omega| <= 1.5000000001 rad/s`;
9. no post-projection clipping or post-initial-state control retry occurs;
10. every observer-only live display completes;
11. every episode is wall-clock paced with achieved real-time factor in
    `[0.95, 1.01]`;
12. manifest identities and referenced plan/world/trace/display/startup hashes
    are valid.

## 7. Validation infrastructure failure rule

An incomplete batch never counts as validation success. The first V4
validation invocation was retained with `status=crashed` after its SSH stdout
pipe closed at 2/32. No parameter changed. A complete second invocation used
server-local stdout/stderr redirection and the identical frozen protocol.

Infrastructure reruns must use a new result directory and preserve the failed
attempt. They may not overwrite partial evidence or silently resume at a
favorable member.

## 8. Independent replay

After held-out validation, the member with the largest measured position
tracking residual is selected from the frozen batch records and rerun in a new
directory. Replay V8 validates two manifests, all referenced hashes, scenario
identity, runtime identity, every single-run gate, and bounded terminal spread.

Gazebo ODE traces need not be byte identical. This post-hoc challenge is a
repeatability diagnostic and cannot change batch acceptance.

## 9. Interpretation

A successful result supports only this statement: the frozen V13 Gazebo
Classic implementation passed the 32 explicitly listed held-out lane-pattern
cases under its empirical discovery-calibrated tracking envelope, while
publishing a real-time observer-only obstacle-certificate display.

The executed evidence is recorded in
`docs/experiments/DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md`.
Neither this protocol nor its successful execution is a formal
differential-drive safety proof, a random-geometry q-SBC theorem, a ROS2 result,
or evidence about a real rover.
