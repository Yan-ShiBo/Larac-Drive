# Gazebo Classic V13 dynamic-road multi-seed validation

## Material Passport

- Date: `2026-07-31`
- Episode contract: `GZ5_DISCOVERY_CALIBRATED_AUDITED_STARTUP_CBF_LIVE_DISPLAY_V13`
- Batch contract: `GZ5_DISCOVERY_CALIBRATED_MULTISEED_V4`
- Replay contract: `GZ5_DISCOVERY_CALIBRATED_REPLAY_VALIDATION_V8`
- Backend: Gazebo Classic 11, ODE, headless paused fixed-step
- Final empirical status: validation `provisional_success`; independent replay `accepted`
- Evidence class: finite-catalog empirical Gazebo differential-drive validation
- Explicit exclusions: no q-SBC probability theorem, no formal tracking-error bound,
  no ROS2 result, and no real-vehicle claim

## 1. Outcome

The frozen V13 implementation passed both 32-member lane-pattern catalogs.
Every episode reached the end of the two-lane road, activated all five seeded
random obstacles, retained a positive measured-state lifted CBF margin, and
completed the observer-only live certificate display. No accepted episode
used post-projection command clipping or retried a control trajectory after an
initial state had been obtained.

The held-out validation result is therefore accepted only as an empirical,
finite-catalog Gazebo result. The fixed-geometry q-SBC research chain remains
`not_ready`; this experiment does not change that status.

## 2. Frozen identity

| Item | Frozen identity |
|---|---|
| Local source archive | `source_gazebo_multiseed_v4_candidate_20260731T220648.tar.gz` |
| Source archive SHA-256 | `bb7730ec15af122c7144e5064dcf4d9eb98b8e498dd9284bc9dec1b529482920` |
| Server source | `/data/users/csynth/cbf_rover_runs/source_gazebo_multiseed_v4_candidate_bb7730ec` |
| Server build | `/data/users/csynth/cbf_rover_runs/build_gazebo_multiseed_v4_candidate_bb7730ec` |
| Bridge SHA-256 | `a786944da7fd169dbca6c2c46116004d2caa004fe8d80704aced408010cbbf7e` |
| State plugin SHA-256 | `888a8e4b746adec1090886128c7a98a23fd574f3174e4c6e10d9e4805efb5f1b` |
| V13 config SHA-256 | `7f1b141b8a0efafe9e64c5a4ae92fa01223940d41e126fb8e646f18ab92bfc0a` |
| V4 batch config SHA-256 | `b49090f39b38289eee91e25cd83428687c53b2ae2089c256a36f42bb1b7ab3f7` |
| Replay V8 config SHA-256 | `a96694426255d5ad5da71a79488bd5737d7f1e0dce5a17f28ad2c37a56944423` |

The discovery and validation catalogs were selected from the already frozen
differential-drive exact-model evidence using only `seed` and the five-bit
lane pattern. Their source metrics SHA-256 values are respectively
`0748e44621d62f52960f764a6fc4e1407757b771795b1aa082b57d29a4818e65`
and `604293e75ea4c29c684e74566a153aa785517051f41846c44a41a67691105517`.
No Gazebo outcome metric participated in seed selection.

After this report and the documentation indexes were updated, a cross-platform
ordinal manifest rehashed 247 executable-critical files: source, tests,
simulation/build sources, all YAML configs, artifacts, and project metadata.
The local workspace and the tested read-only server source both produced
`437afd3cd66ab7e88313e60c2bde04b7fc8f6ac3fef32974fa2706af16059898`.
Documentation and Markdown indexes were deliberately excluded from this
identity, so the comparison proves that the tested program/config did not
drift while the report was written.

## 3. Repair history retained as evidence

| Version | Result | Interpretation |
|---|---|---|
| V10 | seed `7320014` timed out safely after 2000 transitions and 100 s; 19/20 gates passed, with only `goal_reached=false` | The earlier `0.04 m / 0.10 rad` empirical tracking support was too conservative for this route and produced a safe deadlock, not a collision. |
| V11 | `timed out waiting for the Gazebo world namespace`; no control state or trajectory was produced | Gazebo startup itself required a bounded, auditable infrastructure policy. |
| V12 | repair seeds passed; complete discovery was 31/32 | All safety gates passed, but `lane=10101, seed=7330000` measured `0.0556310405 rad`, exceeding the frozen `0.055 rad` heading envelope. |
| V13 | position envelope retained at `0.016 m`; heading envelope rounded outward to `0.065 rad` from the complete V12 discovery | Both repair seeds passed before any V4 catalog run. |

The V12 discovery metrics and summary hashes are:

- `episode_metrics.jsonl`: `48166004b9929443e777805f20a3c561ca3f878904159e0de8fd0e0328e68070`;
- `batch_summary.json`: `0e0ba769b718a015bc6a36f5afe1443a0cf4b0594876745f21384cb9a62ef22e`.

The V13 repair runs used seeds `7320014` and `7320004`. Both reached the goal
with 20/20 gates. Their minimum lifted barriers were `0.176267 m` and
`0.178187 m`; maximum position residuals were `0.012692 m` and `0.012985 m`;
maximum heading residuals were `0.045245 rad` and `0.046320 rad`.

## 4. Audited startup boundary

V13 allows at most three Gazebo startup attempts, but only before the first
measured initial state. Every attempt writes separate bridge and `gzserver`
logs plus SHA-256 identities to `startup_attempts.json`.

Once an initial state exists, the control trajectory is never retried. A
failure after that boundary is an episode failure. The V4 batch aggregate gate
`no_control_retries_after_initial_state` enforces this rule.

This policy is an infrastructure reliability boundary, not control-result
selection. It cannot discard an unsafe or unsuccessful trajectory.

## 5. Discovery result

Server result:
`/data/users/csynth/cbf_rover_runs/results_gazebo_multiseed_v4_discovery_bb7730ec`

| Metric | Result |
|---|---:|
| Episodes passed | `32 / 32` |
| Lane patterns covered | `32 / 32` |
| Success rate | `1.0` |
| Collision / road departure / timeout episodes | `0 / 0 / 0` |
| Maximum position tracking residual | `0.01415336398 m` |
| Maximum heading tracking residual | `0.05167955739 rad` |
| Minimum lifted certificate barrier | `0.17434167616 m` |
| Minimum measured contraction residual | `0.03394713352 m` |
| Achieved real-time factor range | `[0.99977844, 0.99993653]` |
| Episodes with pre-control startup retry | `5` |
| Total / maximum startup attempts | `38 / 3` |
| Control retries after initial state | `0` |
| Live display updates | `6191` |

All 21 aggregate gates were true. Discovery remains ineligible for the final
empirical claim because its complete V12 predecessor calibrated the V13
heading envelope.

Evidence hashes:

- `batch_summary.json`: `455611a14a75698a93f5f1feca0d16b7613484396db594cdee903b24d3b3396c`;
- `episode_metrics.jsonl`: `3fda0b9514ee839b95550eff8ae241624c3e9df4bebb2ebc6604ccbaea6fcb46`.

## 6. Held-out validation result

The first validation invocation wrote two passing episodes and then crashed
with `BrokenPipeError` because its SSH stdout consumer had been closed. It is
preserved at
`/data/users/csynth/cbf_rover_runs/results_gazebo_multiseed_v4_validation_bb7730ec`
with `progress.json status=crashed`; it is not an accepted result.

No research parameter changed after that infrastructure failure. The complete
attempt redirected stdout/stderr to a server-local log before detaching from
SSH and used the same frozen source, binaries, config, seeds, and acceptance
gates:

`/data/users/csynth/cbf_rover_runs/results_gazebo_multiseed_v4_validation_bb7730ec_attempt2`

| Metric | Result |
|---|---:|
| Status | `provisional_success` |
| Episodes passed | `32 / 32` |
| Lane patterns covered | `32 / 32` |
| Success rate | `1.0` |
| Collision / road departure / timeout episodes | `0 / 0 / 0` |
| Maximum position tracking residual | `0.01298520713 m` |
| Maximum heading tracking residual | `0.04631885939 rad` |
| Minimum lifted certificate barrier | `0.17329978044 m` |
| Minimum measured contraction residual | `0.03376840774 m` |
| Achieved real-time factor range | `[0.99971759, 0.99992971]` |
| Episodes with pre-control startup retry | `4` |
| Total / maximum startup attempts | `36 / 2` |
| Control retries after initial state | `0` |
| Live display updates | `6193` |

All 21 aggregate gates were true. Evidence hashes:

- `batch_summary.json`: `0472fceefa31f017dbb1273df71c5712b84ee688015c41d0954153e2dc610478`;
- `episode_metrics.jsonl`: `e7e8728118fbed4e5552c9e2556526573d3a7c28490be49c1969713c2936871a`;
- detached execution log: `e78bba5eca1ea98e215898145fbf302ae8a85e337fc964f5bdaecb826264f621`.

## 7. Independent repeat and replay

The frozen selection rule chose the validation member with the largest
position tracking residual: `lane=11111, seed=7331100`. It was run again in a
new directory, reached the goal, passed 20/20 gates, started on its first
attempt, and had zero control retries.

The V8 replay validator accepted all 19 gates:

- both runs: `677` transitions and `33.85 s` simulated time;
- terminal center-position spread: `6.4925983e-6 m`;
- terminal heading spread: `5.4501204e-10 rad`;
- transition-count and simulated-duration spreads: `0`;
- both manifests and all referenced world, plan, startup, and trace hashes:
  valid;
- byte-identical traces: not required and not observed.

Replay report:
`/data/users/csynth/cbf_rover_runs/results_gazebo_multiseed_v4_selected_replay_bb7730ec/gazebo_replay_validation.json`

SHA-256: `fba2eb097b222ec9f4af0a370d17159ab86a81bf161b353070cd78c957daea8a`.

## 8. Runtime display and reproducible rendering

Each discovery and validation episode published a loopback-only dashboard
during control. The display was a downstream observer: the certificate
evaluator and display never altered `[v, omega]`.

For a useful visual challenge, the post-run renderer used the validation
member with the smallest lifted barrier, `lane=01011, seed=7331005`. It
generated 207 frames at a `0.2 s` simulated interval and 1x simulated-time
playback:

| Artifact | SHA-256 |
|---|---|
| `gazebo_classic_cbf_1x.gif` | `99ff273f9ea9e2f0eb299a0b93bac76e5da5892318b1924a22dae20558a2b14f` |
| `gazebo_classic_cbf_final.png` | `0d2bef5068ef988df7159a9dbb9a06c3de0a0f42cdf04366ae51d94b69e83c15` |
| `render_manifest.json` | `7cdc34b96611d3285c0755de96ac5f4c722e9f7d3d7a6724213e7e08ee26f280` |

These are renderings of the hash-validated measured Gazebo trace, not a Gazebo
GUI screen capture and not a second control run.

## 9. Final server code gates

All final checks used the frozen V13/V4 source on the server:

| Gate | Result |
|---|---|
| Ruff | `All checks passed` with `--no-cache` because the source tree is read-only |
| Fast pytest suite | `114 passed, 3 deselected` |
| MOSEK/SOS slow suite | `3 passed` |
| Gazebo bridge integration | `2 passed` |

The first Ruff invocation attempted to create `.ruff_cache` inside the
read-only source tree and failed before code analysis. The successful
`--no-cache` invocation ran the same rules without writing to the source.

## 10. Claim boundary

This experiment establishes only that the frozen measured-state online CBF,
controller-to-differential-drive mapping, empirical tracking envelope, audited
startup boundary, and observer-only certificate display passed the explicitly
listed Gazebo Classic finite catalogs.

It does not establish:

- a q-SBC reach-avoid probability bound;
- a formal differential-drive tracking-error bound;
- safety for random geometries outside the two frozen catalogs;
- ROS2 scheduling or RViz integration;
- equivalence between Gazebo ODE and a real rover.
