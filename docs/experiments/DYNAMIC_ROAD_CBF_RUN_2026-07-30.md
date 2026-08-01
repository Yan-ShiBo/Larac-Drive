# Dynamic-road online CBF server run (2026-07-30)

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-07-30
- Verification Status: VERIFIED
- Version Label: dynamic_road_cbf_result_v1
- Experiment ID: `DR1_TWO_LANE_RANDOM_OBSTACLES_V1-seed-7302026`
- Exact source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_dynamic_road_20260730T143821Z_1691c55c5c9f`
- Source archive SHA-256:
  `1691c55c5c9f4f1f65cde343dcc2eea02cdcd0e2256aa83fca60211fedd7b68c`
- Read-only result root:
  `/data/users/csynth/cbf_rover_runs/results_dynamic_road_final_20260730T143821Z_1691c55c5c9f`
- Local read-only delivery copy:
  `E:\6_Engineering_Projects\CBF-Rover\runs\20260730T143821Z_dynamic_road_official_1x`

## Experiment result

- Type: exact stochastic simulation with live rendering
- Status: `accepted`
- Seed: `7302026`
- System: M1, `dt=0.05 s`, `||u||_2 <= 0.5 m/s`, N1 noise
- Online method: robust distance CBF plus exact two-dimensional projection
- Termination: `goal_reached`
- Simulated duration: `36.5 s`
- Wall-clock duration: `36.500255163991824 s`
- stderr: empty

The authoritative configuration is
[`dynamic_road_random_obstacles.yaml`](../../configs/scenarios/dynamic_road_random_obstacles.yaml).
Five obstacles are generated from the recorded seed. The first frame has zero
active obstacles; activation events occur later in the trace, so obstacles
actually appear while the rover is moving.

## Commands and hard timeouts

Static gates:

```bash
timeout --signal=TERM --kill-after=30s 600s \
  python -m ruff check --no-cache SOURCE

timeout --signal=TERM --kill-after=30s 600s \
  env -C SOURCE PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src \
  python -m pytest -q -m "not research_slow and not ros2"

timeout --signal=TERM --kill-after=60s 900s \
  env -C SOURCE PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src \
  python -m pytest -q -m research_slow
```

Two independent no-wait replays used:

```bash
timeout --signal=TERM --kill-after=30s 600s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=SOURCE/src python \
  -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-live \
  --config SOURCE/configs/scenarios/dynamic_road_random_obstacles.yaml \
  --output-dir RESULT/replay_a \
  --seed 7302026 --no-wall-clock --no-gif
```

The same command was run into `replay_b`. The official command then added
wall-clock pacing, GIF rendering, and the independently obtained expected
trace hash:

```bash
timeout --signal=TERM --kill-after=60s 1200s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=SOURCE/src python \
  -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-live \
  --config SOURCE/configs/scenarios/dynamic_road_random_obstacles.yaml \
  --output-dir RESULT/official_1x \
  --seed 7302026 \
  --expected-trace-sha256 \
  fafd851ff8c8481484d71ec7fd0e5d683b206f253f884e682654fbd01eada5a1
```

## Verification gates

```text
ruff                                      passed
fast tests                               82 passed, 3 deselected
research_slow tests                       3 passed, 82 deselected
same-seed replay A/B trace SHA-256        exact match
official 1x / replay trace SHA-256        exact match
all five obstacles activated              true
goal reached                              true
collision count                           0
road departure count                      0
post-filter clipping                      false
online CBF intervention observed           true
```

An intentionally overbroad diagnostic invocation passed the whole `tests/`
tree instead of the declared `tests/unit` and `tests/math` testpaths. It
reported `85 passed, 2 failed`: the two failures require the local immutable
handover archive directory, which is deliberately excluded from server source
snapshots. The archive was not uploaded to make those tests pass. Local
read-only verification still found exactly the original three files with
their unchanged SHA-256 values.

## Trajectory metrics

| Metric | Result |
|---|---:|
| Transitions / frames | `730 / 731` |
| Activated obstacles | `5 / 5` |
| Filter intervention steps | `15` |
| Maximum intervention | `0.3162580827512036 m/s` |
| Maximum filtered speed | `0.4600000000000001 m/s` |
| Minimum active obstacle CBF margin | `0.03360764347796319 m` |
| Minimum physical clearance | `0.08360764347796323 m` |
| Minimum CBF constraint residual | `-5.551115123125783e-17 m/s` |
| Collision / road departure | `0 / 0` |

The tiny negative residual is floating-point roundoff and lies well inside the
declared `-1e-9 m/s` acceptance tolerance.

## Controller diagnostic retained

The initial `3.4 m` lane-switch lookahead avoided every obstacle without CBF
intervention. A diagnostic `0.4 m` lookahead produced a safe CBF equilibrium
(`h_min=0.014428 m`) but timed out because the nominal planner waited to switch
lanes until after the CBF stopping distance. The accepted `0.6 m` lookahead
lies outside that equilibrium distance: it triggers visible CBF intervention
and still completes the task. These diagnostics are preserved as evidence,
not promoted as accepted results.

## Output artifacts

| File | Size | SHA-256 |
|---|---:|---|
| `dynamic_road_cbf_1x.gif` | `3,387,587` | `65c4b341f3a2b50623bf9a7a6387aae48e54a8365b9e9cc0c6d964c562bb5007` |
| `dynamic_road_cbf_final.png` | `220,611` | `1ff8ba24353efff52c56e9b603565be4d679f37938f168da8392dda5a98103ce` |
| `live_trace.jsonl` | `2,104,782` | `fafd851ff8c8481484d71ec7fd0e5d683b206f253f884e682654fbd01eada5a1` |
| `obstacle_plan.json` | `2,451` | `6ad51ea8958e55e142b866372726a0a0791b1dec5980221319056c241149e492` |
| `run_manifest.json` | `3,200` | `1904a30c20778be61b4689efdd173599d48298956a2491f5d4cf4dbf06637f6e` |

The obstacle-plan content identity is
`c1472d0c99646d867358118918acf119115e90160f39a0925861b6cf63e9dda2`.
The run-manifest content identity is
`3df747dd7f7ed44ac056b4f9d75b437037b3ba6c2cfa6adde6f333f3bc58dd54`.

## Monitoring and resource note

The official process was monitored by timeout-wrapper PID and Python child
PID. CPU remained active and trace/output growth continued. Python RSS rose
from about `43 MB` during wall-clock simulation to a sampled peak of about
`873 MB` while Matplotlib/Pillow held GIF frames for encoding. This crossed the
generic `3x initial RSS` advisory threshold but was not an out-of-memory event
and does not mean the server has only 1 GB of RAM. The process completed
naturally, wrote a 3.39 MB GIF, and emitted no stderr.

## Claim boundary

The displayed certificate is an online robust distance-CBF action filter. Its
barrier values, constraint residuals, and interventions are evaluated from the
measured state at runtime. The full episode is one reproducible empirical
stochastic trajectory.

This run does not inherit the fixed P1/D1 polynomial q-SBC, does not provide a
random-geometry reach-avoid probability bound, and does not establish Gazebo
or differential-drive tracking safety. Those require separate parameterized
certificate synthesis and a verified tracking-error contract.
