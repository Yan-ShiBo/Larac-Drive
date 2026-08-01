# Differential-road multi-seed server run (2026-07-31)

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-07-31
- Verification Status: VERIFIED
- Version Label: `differential_road_multiseed_result_v1`
- Experiment ID: `DR2_V2_MS1_MULTISEED_STRESS_V1`
- Exact source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_differential_multiseed_20260730T203821Z_776e2636fa95`
- Source archive SHA-256:
  `776e2636fa954be034bd61da81c669a6f0c07e57a26010e4edc07774380d10cf`
- Read-only server result root:
  `/data/users/csynth/cbf_rover_runs/results_differential_multiseed_20260730T203821Z_776e2636fa95`
- Read-only local delivery copy:
  `E:\6_Engineering_Projects\CBF-Rover\runs\results_differential_multiseed_20260730T203821Z_776e2636fa95`

## Result

The frozen differential-drive V2 implementation passed the complete
predeclared protocol:

- discovery: `256 / 256` episodes reached the goal;
- held-out validation A: `1024 / 1024` reached the goal;
- independent held-out validation B: `1024 / 1024` reached the goal;
- all `32` five-obstacle lane patterns occurred;
- collision, road departure, timeout, model breach, and CBF infeasibility:
  all zero;
- validation A and B produced byte-identical per-episode metrics.

Validation B status is `accepted`. For the frozen seeded sampling procedure,
the exact one-sided 95% Clopper-Pearson lower bound is
`0.997078755364612`.

## Frozen catalogs

The protocol was frozen before execution in
[`DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_PROTOCOL.md`](../mathematics/DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_PROTOCOL.md).

| Catalog | Seed interval | Episodes | Role |
|---|---:|---:|---|
| Discovery | `7320000..7320255` | `256` | implementation/controller discovery |
| Validation | `7321000..7322023` | `1024` | held-out empirical validation |

No V2 source, controller, threshold, or seed changed after discovery. No
validation seed was used in earlier DR1 or differential V1/V2 work.

## Server gates

```text
ruff                                                   passed
focused differential/batch/DR1/projection/config       21 passed
fast suite                                             93 passed, 3 deselected
research_slow suite                                     3 passed, 93 deselected
```

Every server command used a hard timeout and
`PYTHONDONTWRITEBYTECODE=1`. Batches ran serially in ascending seed order.
PID, CPU, RSS, atomic progress, output growth, and stderr were monitored.
There were no silent retries and no saved failure traces because no episode
failed.

## Aggregate metrics

| Metric | Discovery | Held-out validation |
|---|---:|---:|
| Episodes passed | `256 / 256` | `1024 / 1024` |
| Lane patterns | `32 / 32` | `32 / 32` |
| Episodes with projection intervention | `252` | `996` |
| Total intervention steps | `6992` | `28493` |
| Minimum lifted certificate `h` | `0.13408728819016116 m` | `0.12064749104095798 m` |
| Minimum lifted physical clearance | `0.1840872881901612 m` | `0.17064749104095803 m` |
| Minimum body-center physical clearance | `0.33528330300204146 m` | `0.32049885141849355 m` |
| Minimum CBF residual | `0.32759147088633367 m/s` | `0.31889975126232717 m/s` |
| Minimum realized contraction residual | `0.021132004722605013 m` | `0.01927937496354122 m` |
| Maximum mapping residual | `1.6883057536160649e-16 m/s` | `2.0014830212433605e-16 m/s` |
| Maximum linear speed | `0.45999999999847196 m/s` | `0.4599999999961843 m/s` |
| Maximum absolute angular speed | `1.5000000000000007 rad/s` | `1.5000000000000007 rad/s` |

The angular excess over the declared `1.5 rad/s` boundary is floating-point
roundoff below `1e-15`; it is inside the frozen `1e-10` comparison tolerance.
No command was clipped or modified after mapping.

## Deterministic batch replay

Validation A and B produced the same `episode_metrics.jsonl` SHA-256:

```text
75d874b85f4fee6460ae84e464ba115c4b5bd6653e8bff696127e6c181b429a5
```

Both stderr logs are empty. Validation B's replay gate and all other
acceptance gates are `true`.

## Worst held-out seed replay

Seed `7321609`, lane pattern `01010`, had the minimum lifted certificate
barrier and body-center physical clearance. Its batch trace SHA-256 was

```text
4d0dbfcb7f49ff58b41b84a29deca3b589fcb19cefa2e8f2d9b6f579c98afef7
```

The 1× wall-clock run supplied that identity as a hard gate and was
`accepted`:

| Fact | Result |
|---|---:|
| Termination | `goal_reached` |
| Transitions / frames | `811 / 812` |
| Simulated / paced wall time | `40.55 / 40.550376744009554 s` |
| Activated obstacles | `5 / 5` |
| Collision / road departure | `0 / 0` |
| Projection intervention steps | `55` |
| Minimum lifted `h` | `0.12064749104095798 m` |
| Minimum body-center physical clearance | `0.32049885141849355 m` |
| Minimum realized contraction residual | `0.021617381364381336 m` |
| Batch/live trace identity | exact match |

The animation displays the body heading, center and lookahead paths, inflated
certificate boundaries, online `h`, linear/angular commands, and projection
intervention. Its title explicitly excludes q-SBC and Gazebo claims.

## Artifact identities

| File | Bytes | SHA-256 |
|---|---:|---|
| `discovery/episode_metrics.jsonl` | `285275` | `af7cc900689a4cd050d457e453f31f54e9fa5ee9d0177af62d8f9f1041ac5b47` |
| `validation_a/episode_metrics.jsonl` | `1140948` | `75d874b85f4fee6460ae84e464ba115c4b5bd6653e8bff696127e6c181b429a5` |
| `validation_b/episode_metrics.jsonl` | `1140948` | `75d874b85f4fee6460ae84e464ba115c4b5bd6653e8bff696127e6c181b429a5` |
| `worst_seed_7321609_1x/differential_road_cbf_1x.gif` | `4241201` | `e5b0e241bd37ebe3f95bf25ba936344827ffd78583870edb873066f33fb5dd7f` |
| `worst_seed_7321609_1x/differential_road_cbf_final.png` | `273394` | `55b27317088540afb78f961c791e9f5770933945977f0264ddb853cdfebd57bd` |
| `worst_seed_7321609_1x/live_trace.jsonl` | `3451697` | `4d0dbfcb7f49ff58b41b84a29deca3b589fcb19cefa2e8f2d9b6f579c98afef7` |
| `worst_seed_7321609_1x/run_manifest.json` | `4292` | `78155a24897a35aee01826f34c91ca99d13b30bfe3d9f0816134ecdb08266ec7` |

All `22` local delivery files match the server result root by relative path
and SHA-256.

## Resource note

Batch RSS remained near `54 MB`. The worst-seed Matplotlib/Pillow GIF process
rose from about `43 MB` during paced simulation to a sampled peak of about
`953 MB` while retaining frames, then released memory on exit. This is not
server capacity and was not an out-of-memory event.

## Claim boundary and next stage

This is reproducible empirical evidence for the declared stochastic Euler
differential-drive model, forward-only command set, lookahead lifting, body
inflation, and seeded obstacle/noise procedure.

It is not a q-SBC theorem and does not establish Gazebo, wheel-dynamics,
latency, slip, ROS2 scheduling, or real-vehicle safety. The next meaningful
gate is a physics/runtime tracking-error experiment. Because the current
server has no ROS2 Humble/Gazebo installation, that gate cannot be honestly
claimed from this exact backend; the accepted exact model and inflation
contract are now ready to be connected when that runtime is available.
