# Dynamic-road differential-drive server run (2026-07-31)

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-07-31
- Verification Status: VERIFIED
- Version Label: `dynamic_road_differential_v2_result_v1`
- Experiment ID: `DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V2`
- Exact source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_differential_v2_20260730T202451Z_4e2054356c2b`
- Source archive SHA-256:
  `4e2054356c2bce19f75f27300169994537cc5d6805d847581324fb8e6174969a`
- Read-only server result root:
  `/data/users/csynth/cbf_rover_runs/results_differential_v2_20260730T202451Z_4e2054356c2b`
- Read-only local delivery copy:
  `E:\6_Engineering_Projects\CBF-Rover\runs\results_differential_v2_20260730T202451Z_4e2054356c2b`

## Accepted result

The first exact-backend nonholonomic slice is `accepted` on discovery seed
`7311127`. The rover uses bounded differential commands and continuous heading
instead of directly applying independent world-frame velocities.

| Fact | Result |
|---|---:|
| Termination | `goal_reached` |
| Transitions / frames | `778 / 779` |
| Simulated / paced wall time | `38.9 / 38.900394608965144 s` |
| Activated obstacles | `5 / 5` |
| Collision / road departure | `0 / 0` |
| CBF/reachability projection steps | `44` |
| Maximum projection intervention | `0.22594547268568915 m/s` |
| Maximum linear speed | `0.4599999657332198 m/s` |
| Maximum absolute angular speed | `1.5000000000000002 rad/s` |
| Minimum lifted certificate barrier | `0.1607467695600762 m` |
| Minimum lifted physical clearance | `0.21074676956007624 m` |
| Minimum body-center physical clearance | `0.3649215647109396 m` |
| Minimum CBF constraint residual | `0.4551272240576545 m/s` |
| Minimum realized contraction residual | `0.028447104103039855 m` |
| Maximum exact mapping residual | `1.1443916996305594e-16 m/s` |
| Post-projection command clipping | none |

The sampled angular value exceeds `1.5` only by about
`2.2e-16 rad/s` floating-point roundoff and lies inside the declared `1e-10`
feasibility tolerance. The command value itself was not clipped or changed.

## Model and safety construction

The authoritative V2 configuration is
[`dynamic_road_differential_v2.yaml`](../../configs/experiments/dynamic_road_differential_v2.yaml).
Its main contracts are:

- stochastic explicit-Euler differential drive at `dt=0.05 s`;
- `0 <= v <= 0.5 m/s`, `|omega| <= 1.5 rad/s`, no reverse;
- N1 additive position noise and no heading noise;
- a `0.15 m` lookahead point whose reachable velocity halfspaces enter the
  online projection;
- an additional `0.15 m` obstacle/road inflation protecting the complete body;
- a worst-case Euler heading remainder of `0.000421809086151014 m`, added to
  the N1 disturbance support;
- exact velocity-to-command mapping with no post-projection clipping.

The V2 lane supervisor starts a lane change at `2.4 m`, below the declared
`2.8..3.2 m` obstacle appearance range. It therefore reacts only to visible
obstacles.

## Preserved discovery failures

Failures were not silently retried:

1. Snapshot `05c34dd9ec40...` stopped at Ruff because of one unused import.
2. V1 snapshot `c4fe52b91a64...` passed Ruff but stopped safely at step `146`
   with `cbf_action_set_infeasible`. Its controller waited until `0.6 m` to
   change lanes; the inflated forward-only CBF required a backward component
   before that point. Minimum lifted `h` was still
   `0.02548614157750076 m`, with zero collision and road departure.
3. Initial V2 snapshot `40694b548c35...` reached the angular boundary
   `-1.5000000000000002`; a zero-tolerance duplicate dynamics check rejected
   it even though the declared projection tolerance passed.

The final implementation passes the already-declared `1e-10` tolerance into
that duplicate check. It does not clip the command, alter the transition, or
relax any certificate acceptance threshold.

V1 failure behavior is retained as an automated regression test.

## Server gates

```text
ruff                                             passed
focused differential/DR1/projection/config       16 passed
fast suite                                       90 passed, 3 deselected
research_slow suite                               3 passed, 90 deselected
```

All commands had hard timeouts and used `PYTHONDONTWRITEBYTECODE=1`.

Two independent unpaced CLI runs wrote complete traces. Replay B supplied
Replay A's trace identity as a hard gate:

```text
1ca92cad1bd4f1c6a8d6dbe7a191b170511476a4bccf0585849b3a243ccfc870
```

The 1× wall-clock run produced the same trace hash and status `accepted`.
Official stderr is empty.

## Visual artifacts

The animation shows:

- the rover body, heading, and body-center path;
- the lookahead CBF evaluation point and its path;
- random obstacle appearances and inflated lifted-certificate boundaries;
- online minimum `h`;
- linear speed, absolute angular speed, and projection intervention;
- live mapping residual and realized contraction residual.

| File | Bytes | SHA-256 |
|---|---:|---|
| `differential_road_cbf_1x.gif` | `4106326` | `196e20989b1b05ccf1cfc456496e7b4395f2e6308be1811c1d6ae2c9b5571cef` |
| `differential_road_cbf_final.png` | `265879` | `a9a08e192fb003c673ebc553d38a7ded02de98d5433b5048b671c8e4580b60fa` |
| `live_trace.jsonl` | `3310623` | `1ca92cad1bd4f1c6a8d6dbe7a191b170511476a4bccf0585849b3a243ccfc870` |
| `obstacle_plan.json` | `2451` | `77be4512141f7fecfa7c481e91f1a0ff61f7a91e8ded22c2732ee37575a0c0ff` |
| `run_manifest.json` | `4274` | `e2f240fdea2ead12aabde1a892cf2cee8316bf1995d74d6478caec99ee94efff` |

All `13` files in the local delivery copy match the server result root by
relative path and SHA-256.

## Resource note

The Python process used about `41 MB` RSS during paced simulation. Matplotlib
and Pillow retained animation frames during GIF encoding, increasing sampled
RSS to about `919 MB`; memory was released when the process exited. This is
the renderer process peak, not server capacity.

## Claim boundary and next experiment

This run establishes reproducible empirical behavior only for the declared
stochastic Euler differential-drive model. It is not a q-SBC theorem and does
not claim Gazebo safety, wheel dynamics, slip, latency, ROS2 scheduling, or a
real vehicle.

Seed `7311127` was selected from prior DR1 results and is discovery-only.
Before a differential-drive generalization statement, V2 must face a new
predeclared discovery catalog, then a previously unused held-out catalog with
an exact batch replay gate.
