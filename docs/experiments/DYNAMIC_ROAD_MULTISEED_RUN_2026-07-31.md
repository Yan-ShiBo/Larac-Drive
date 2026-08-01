# Dynamic-road multi-seed server run (2026-07-31)

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-07-31
- Verification Status: VERIFIED
- Version Label: `dynamic_road_multiseed_result_v1`
- Experiment ID: `DR1_MS1_MULTISEED_STRESS_V1`
- Exact source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_multiseed_20260730T194038Z_5fe191600ff2`
- Source archive SHA-256:
  `5fe191600ff2b035c796ceaf41e5b2d62b67f3a06462137032916f0260905ad5`
- Server result root:
  `/data/users/csynth/cbf_rover_runs/results_multiseed_20260730T194038Z_5fe191600ff2`
- Local delivery copy:
  `E:\6_Engineering_Projects\CBF-Rover\runs\results_multiseed_20260730T194038Z_5fe191600ff2`

## Result

The frozen online-CBF implementation passed the complete predeclared protocol:

- discovery: `256 / 256` goal-reaching episodes;
- held-out validation A: `1024 / 1024` goal-reaching episodes;
- held-out validation B: `1024 / 1024` goal-reaching episodes;
- collision, road departure, timeout, model breach, and CBF-infeasibility
  episodes: all zero;
- all `32` five-obstacle lane patterns occurred;
- validation A and B produced byte-identical episode metrics.

Validation B is therefore `accepted`. Its exact one-sided 95% Clopper-Pearson
lower confidence bound is `0.997078755364612` for the fixed seeded sampling
procedure.

## Frozen protocol

The protocol was written before execution in
[`DYNAMIC_ROAD_MULTISEED_PROTOCOL.md`](../mathematics/DYNAMIC_ROAD_MULTISEED_PROTOCOL.md).
The seed catalogs are non-overlapping:

| Catalog | Seed interval | Episodes | Role |
|---|---:|---:|---|
| Discovery | `7310000..7310255` | `256` | diagnostic only |
| Validation | `7311000..7312023` | `1024` | held-out empirical claim |

No implementation, configuration, or acceptance threshold changed after the
held-out run began.

## Server gates

The exact uploaded snapshot passed:

```text
ruff                                             passed
focused dynamic-road/config/projection tests     13 passed
fast tests                                       85 passed, 3 deselected
research_slow tests                               3 passed, 85 deselected
```

Every command had a hard timeout and ran with
`PYTHONDONTWRITEBYTECODE=1`. The discovery and validation processes were
serial, ordered by ascending seed, and monitored through their wrapper PID,
Python child PID, RSS, CPU use, growing JSONL, and atomic `progress.json`.
There were no silent retries.

The held-out commands were:

```bash
timeout --signal=TERM --kill-after=60s 1800s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=SOURCE/src python \
  -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-batch \
  --batch-config SOURCE/configs/experiments/dynamic_road_multiseed.yaml \
  --catalog validation --output-dir RESULT/validation_a

timeout --signal=TERM --kill-after=60s 1800s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=SOURCE/src python \
  -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-batch \
  --batch-config SOURCE/configs/experiments/dynamic_road_multiseed.yaml \
  --catalog validation --output-dir RESULT/validation_b \
  --expected-metrics-sha256 \
  aa086a39fc47f6c340299cfefcb7a3ffe02ef3a4129bf9ea35b46e9eef2eecda
```

Both validation stderr logs are empty.

## Aggregate metrics

| Metric | Discovery | Held-out validation |
|---|---:|---:|
| Episodes passed | `256 / 256` | `1024 / 1024` |
| Lane patterns | `32 / 32` | `32 / 32` |
| Episodes with CBF intervention | `244` | `999` |
| Total intervention steps | `3593` | `15653` |
| Minimum barrier `h` | `0.023201321177394374 m` | `0.02247359551050765 m` |
| Minimum physical clearance | `0.07320132117739442 m` | `0.0724735955105077 m` |
| Minimum CBF residual | `-1.1102230246251565e-16 m/s` | `-1.3877787807814457e-16 m/s` |
| Maximum filtered speed | `0.4609360889381668 m/s` | `0.4611009937290335 m/s` |

The residuals are floating-point roundoff inside the frozen `-1e-9 m/s`
tolerance. Every action remained inside the S1 disk without post-filter
clipping.

Validation A and B have the same metrics-file SHA-256:

```text
aa086a39fc47f6c340299cfefcb7a3ffe02ef3a4129bf9ea35b46e9eef2eecda
```

## Worst held-out seed live replay

Seed `7311127`, lane pattern `01101`, had the smallest held-out barrier and
physical clearance. It was replayed at 1× simulated time with the batch trace
hash supplied as a hard acceptance gate:

```bash
timeout --signal=TERM --kill-after=60s 1200s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=SOURCE/src python \
  -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-live \
  --config SOURCE/configs/scenarios/dynamic_road_random_obstacles.yaml \
  --output-dir RESULT/worst_seed_7311127_1x \
  --seed 7311127 \
  --expected-trace-sha256 \
  4b1bfaffb99a022d5d74c1711b1cd851c40173e04a0035b0cfb5cf99a47acbad
```

The replay was `accepted` and its stderr is empty:

| Fact | Result |
|---|---:|
| Termination | `goal_reached` |
| Transitions / frames | `767 / 768` |
| Simulated / paced wall time | `38.35 / 38.35037902905606 s` |
| Obstacles activated | `5 / 5` |
| CBF intervention steps | `47` |
| Maximum intervention | `0.3421573542809597 m/s` |
| Minimum barrier `h` | `0.02247359551050765 m` |
| Minimum physical clearance | `0.0724735955105077 m` |
| Collision / road departure | `0 / 0` |
| Batch/live trace hash | exact match |

The animation displays the seeded road geometry, obstacle inflation boundaries,
rover path, live minimum barrier, CBF residual, and filter intervention. The
title explicitly identifies the method as an online CBF and not a q-SBC.

## Artifact identities

| File | Bytes | SHA-256 |
|---|---:|---|
| `discovery/episode_metrics.jsonl` | `215244` | `98464968d3c75dec9ea859030d5caa0852813d51739bf91a1c1a02d6610cd3f3` |
| `validation_a/episode_metrics.jsonl` | `861997` | `aa086a39fc47f6c340299cfefcb7a3ffe02ef3a4129bf9ea35b46e9eef2eecda` |
| `validation_b/episode_metrics.jsonl` | `861997` | `aa086a39fc47f6c340299cfefcb7a3ffe02ef3a4129bf9ea35b46e9eef2eecda` |
| `worst_seed_7311127_1x/dynamic_road_cbf_1x.gif` | `3522046` | `2c98c222989a439655a9bcc3d491bc1c1e0f316f9ef1c091a9a8cbd934b5ab75` |
| `worst_seed_7311127_1x/dynamic_road_cbf_final.png` | `230992` | `9adcef3c84c8bc38b045335dca1b735f3df98dc131f315ff881debc89b0d9b49` |
| `worst_seed_7311127_1x/live_trace.jsonl` | `2208801` | `4b1bfaffb99a022d5d74c1711b1cd851c40173e04a0035b0cfb5cf99a47acbad` |
| `worst_seed_7311127_1x/run_manifest.json` | `3196` | `61b3fbb0ba1b188078e06827eca833fce873a92885874a32927af6eb2980a885` |

All `22` files in the local delivery copy were compared against the server
result root by relative path and SHA-256; mismatch count was zero.

## Resource note

Batch RSS remained near `50 MB`. During GIF composition, Matplotlib/Pillow
retained frames and sampled RSS rose from about `40 MB` to about `880 MB`,
then was released when encoding completed. This is per-process peak memory,
not total server capacity and not an out-of-memory event.

## Claim boundary and next gate

This result is strong empirical evidence for the frozen M1 stochastic
single-integrator, seeded obstacle generator, lane supervisor, and exact online
robust distance-CBF projection. It is not a random-geometry q-SBC theorem and
does not yet establish differential-drive or Gazebo tracking safety.

The next experiment must introduce a separately declared nonholonomic tracking
contract. It will measure how accurately bounded linear/angular velocity
commands track the accepted holonomic safe vector, then inflate obstacles by a
measured tracking-error budget before any Gazebo safety claim is considered.
