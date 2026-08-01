# 2026-07-31 Dynamic-road Gazebo Classic measured-state CBF run

## Material Passport

- Experiment family: `GZ2_DYNAMIC_ROAD_LIFTED_CBF`
- Accepted run contract: `GZ2_DYNAMIC_ROAD_LIFTED_CBF_CONFIRMATION_V6`
- Replay contract: `GZ2_DYNAMIC_ROAD_LIFTED_CBF_REPLAY_VALIDATION_V2`
- Seed: `7321609`
- Backend: Gazebo Classic `11.15.1`, ODE, headless paused fixed-step
- Host: Ubuntu `20.04.6 LTS`, 80 logical CPUs, 125 GiB RAM
- Verification status: `accepted`
- Claim class: empirical Gazebo differential-drive closed loop
- Formal exclusions: no q-SBC probability result, no ROS2 result, no
  real-vehicle result

## 1. Outcome

The Rover reached the end of the two-lane physical Gazebo road in two
independent runs. In both runs:

- all five seeded random cylinders became visible to the controller;
- collision and road-departure counts were zero;
- the measured-state CBF was evaluated before every differential command;
- command limits were inside the projection and no post-CBF clipping occurred;
- the lifted barrier, projection residual and measured contraction gates
  remained nonnegative within the declared `1e-9` numerical tolerance.

The separate replay validator returned `accepted` with all 19 gates true.
Every returned pose was sampled on demand after the target physics iteration
and was required to carry a simulation timestamp at least as recent as the
corresponding world-statistics sample.

## 2. Server capability and isolation

Observed environment:

```text
Gazebo multi-robot simulator, version 11.15.1
Ubuntu 20.04.6 LTS
80 logical CPUs
125 GiB RAM, 113 GiB available during final evidence collection
ROS2/colcon absent
```

The experiment did not install or emulate ROS2. It used Gazebo Classic's native
transport API through the standalone `GZCB1` C++ bridge and its timestamped
state model plugin. Every V6 run used the non-default loopback master
`http://127.0.0.1:11466` and explicitly set `GAZEBO_IP=127.0.0.1`, so the
experiment neither attached to nor stopped the unrelated Gazebo/ROS Noetic
process on the default master.

The bridge and model plugin were compiled on the server against the installed
Gazebo headers and libraries. Their responsibilities were restricted to
timestamped pose/world-statistics transport, differential command publication
and fixed physics stepping. Controller and CBF mathematics remained in Python.

## 3. Failure-preserving calibration sequence

All failed runs remain separate evidence. No result was overwritten and no
metric or trace was edited.

| Version | Frozen source archive SHA-256 | Result | Observed cause |
|---|---|---|---|
| V1 | `4ba99b431af049b82f5c36893e56ffe672f7a421982efcebf5139c0cb55b1564` | failed, timeout at 2000 steps | wheel axle at the body center left the center of mass on the support boundary; maximum pitch was `0.4378627884 rad` |
| V2 | `bdbc2c622356ea007239cdbb4c9490ae310a1b4e8c54204111897b93a755bf3` | failed, timeout at 2000 steps | axle shift stabilized pitch, then a transport angular-sign mismatch made the rover turn opposite to the mathematical convention |
| V3 | `c66670532c2f7cb132152e0892538b46bbb4a3851cf1b72b5bf57ee7a519d08d` | reached goal but failed the predeclared heading gate | calibrated plugin sign `-1`; observed heading residual `0.0752329408 rad` exceeded the original `0.05 rad` envelope |
| V4 | `0b733868fdbbcce0224c130b2c12696ce883e915e3d9da8b329c9a5dbf5d275e` | first run provisional; independent replay failed | replay position residual `0.0301104751 m` exceeded the frozen `0.025 m` envelope during a sharp angular-command transient |
| V5 | `a495c31748accf620865fad4fb14d4d80820d555c758be42352ae4f6aeb5bd62` | both independent runs provisional; replay validator accepted | V4 replay evidence was rounded outward to `0.04 m / 0.10 rad` before V5 execution |
| V6 | `bf863c7cc41422d9891df31c01ad6559cc7c6b9020a8fc31fdde969984472067` | both independent runs provisional; replay-v2 accepted | dedicated state plugin plus terminal request/response eliminated stale pose samples; loopback transport and deferred plugin initialization closed startup races |

Two direct physical calibration probes were also retained:

```text
/data/users/csynth/cbf_rover_runs/gazebo_physical_calibration_straight_20260731
/data/users/csynth/cbf_rover_runs/gazebo_physical_calibration_turn_20260731
```

The straight probe confirmed positive forward motion. The turn probe showed
that a raw plugin angular command of `+0.5` produced approximately
`-0.46094 rad` yaw in one second, establishing that transport convention
correction was necessary.

## 4. V5/V6 frozen tracking support

V5 introduced, and V6 retained unchanged, the base DR2/N1 support plus the
outward-rounded V4 replay envelope:

```text
base support                         0.005421809086151014 m
position coordinate half-width      0.04 m
heading bound                        0.10 rad
lookahead displacement bound         0.014993750781203498 m
effective CBF support                0.060415559867354516 m
```

The full derivation and single-run/replay gates are frozen in
[`DYNAMIC_ROAD_GAZEBO_CLASSIC_CONTRACT.md`](../mathematics/DYNAMIC_ROAD_GAZEBO_CLASSIC_CONTRACT.md).

## 5. Timestamp-aligned bridge hardening

V5 passed its declared empirical gates, but a later bridge audit found that
Gazebo's asynchronous pose topic could expose a pose older than the terminal
world-statistics sample. A deterministic regression reproduced
`target_iterations=51`, `stats_time=0.051 s` and stale
`pose_time=0.042 s`.

The final V6 transport boundary therefore:

1. waits for the isolated world namespace and bounded plugin-topic
   connections;
2. defers state-plugin transport initialization until the first
   `WorldUpdateBegin`, avoiding a namespace race during `ModelPlugin::Load`;
3. performs exactly one zero-command bootstrap physics iteration;
4. waits for each target physics iteration before requesting the current model
   pose from the companion plugin;
5. retries a dropped response without advancing physics and returns only when
   `pose_sim_time >= world_stats_sim_time`;
6. pins both `GAZEBO_MASTER_URI` and `GAZEBO_IP` to loopback and closes child
   processes even when startup fails.

The feedback-loop regression asserts initial iteration/time `1 / 0.001 s`,
then exactly `+50 / +0.05 s`. It passed 10 consecutive diagnostic cold starts
and 5 additional starts from the final frozen source and rebuilt binary, with
no port or process leak. All temporary diagnostic instrumentation was removed
before the final source archive was created.

The rejected `e078e01b...e0b228f` candidate remains preserved separately. It
was never used for accepted V6 evidence.

## 6. V6 physical-run evidence

| Metric | V6-A | V6-B |
|---|---:|---:|
| status | `provisional_success` | `provisional_success` |
| termination | `goal_reached` | `goal_reached` |
| single-run gates | 15/15 | 15/15 |
| control transitions | 830 | 831 |
| simulated duration | 41.50 s | 41.55 s |
| wall time | 166.1131 s | 166.2259 s |
| activated obstacles | 5 | 5 |
| collisions / departures | 0 / 0 | 0 / 0 |
| minimum lifted barrier | 0.3027851506 m | 0.3027923170 m |
| minimum CBF constraint residual | `-8.33e-17 m/s` | `-5.55e-17 m/s` |
| minimum measured contraction residual | 0.0602608982 m | 0.0602603536 m |
| maximum position tracking residual | 0.0126918719 m | 0.0126918719 m |
| maximum absolute heading residual | 0.0452608635 rad | 0.0462751865 rad |
| maximum absolute roll | `3.71e-5 rad` | `3.71e-5 rad` |
| maximum absolute pitch | 0.0063162852 rad | 0.0063162852 rad |

Frozen identity evidence:

```text
source archive SHA-256
bf863c7cc41422d9891df31c01ad6559cc7c6b9020a8fc31fdde969984472067

run config SHA-256
903a617079d7f0303a7e1be9503ee47cc5d9c06a30a9b6d4ce64715202cc4052

obstacle-plan content SHA-256
69a84355c38d50e490a2a342a17c1f1d293307351fae2ca049d36aebbd0afe4f

world SDF SHA-256
a8fae1a13ffedec58d7176c9753b4817c8bc27292cb168dd35fc9fca2d801eb0

bridge / state-plugin SHA-256
8b2cd458da327715ea88998448d4c87d31186ea4ddc5ca2d30b7b77681116b15
beda8448525cd13173454d0abda3740a490c9f6c73daf2f169ede16510ad24a0

V6-A / V6-B trace SHA-256
4caccb197f6d4fa4a9fb59525695c8ba9724e13df2bc1f7bcc5d94056feac656
275cef7abcc965d378eb1ddc2cfe085d029c316873a2da697094291e7170d702
```

## 7. Independent replay-v2 report

The validator ran from the same final source archive and independently checked
both manifest content hashes, every referenced file hash, run/config/scenario
identity, all single-run gates and the predeclared terminal spreads.

```text
/data/users/csynth/cbf_rover_runs/results_gazebo_final_v6_aligned_replay_v2/
  gazebo_replay_validation.json

report content SHA-256:
94570ba704e5f770ecda996376f09322a52cf9ae47b384f1c49e91621ee62007
```

Aggregate replay facts:

```text
transition-count spread             1          <= 2
simulated-duration spread           0.05 s     <= 0.10 s
terminal center-position spread     0.0033361 m <= 0.05 m
terminal heading spread             2.1154e-6 rad <= 0.10 rad
byte-identical trace replay         false
all replay gates                    19/19 true
```

The trace hashes are intentionally allowed to differ. Each run must
independently satisfy its safety and tracking contract, while physical
repeatability is bounded at the terminal trajectory level. This is an empirical
repeatability contract, not bitwise determinism or a formal probability bound.

## 8. Validation and reproducible commands

Final frozen-source validation:

```text
Ruff                                      passed
focused Gazebo/replay unit tests          8 passed
fast unit + mathematics suite             101 passed, 3 deselected
research_slow mathematics suite           3 passed, 101 deselected
final bridge cold-start regression        5/5 passed
```

Physical run:

```bash
PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v6.yaml \
  --bridge-binary build/gazebo_classic_bridge/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/result/directory \
  --seed 7321609 \
  --no-gif
```

Independent acceptance:

```bash
PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  validate-gazebo-classic-road-replay \
  --config configs/experiments/dynamic_road_gazebo_classic_replay_v2.yaml \
  --run-manifest /absolute/first/run_manifest.json \
  --run-manifest /absolute/replay/run_manifest.json \
  --output-dir /absolute/new/validation/directory
```

Verified source-trace rendering:

```bash
PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  render-gazebo-classic-road-cbf-run \
  --config configs/experiments/dynamic_road_gazebo_classic_v6.yaml \
  --run-manifest /absolute/accepted/run_manifest.json \
  --output-dir /absolute/new/render/directory
```

## 9. Verified obstacle-certificate visualization

The render command revalidated the V6-A manifest and trace hashes before
producing 209 frames at a `0.2 s` frame interval:

```text
1x GIF SHA-256
5e06bef820dc230ba648101c00e0ebebc50c55c62ff62b880733b6efaed6780f

final PNG SHA-256
306ef9cbfc631056a24db1dbcaba24dc16da849a7efb98c0577915bd1db7bb13

render report content SHA-256
98989322341b6115714d7c066c399f5d397a4da27568ec183278b194b989bd89
```

The animation shows the physical road, measured Rover body and lookahead
point, newly activated random obstacles, inflated certificate boundaries,
minimum lifted barrier, realized contraction, `[v, omega]` and online CBF
intervention. It is a 1x playback of the measured headless Gazebo trace, not a
Gazebo GUI screen capture.

## 10. Result locations

```text
server V6-A:
/data/users/csynth/cbf_rover_runs/results_gazebo_final_v6_aligned_a

server V6-B:
/data/users/csynth/cbf_rover_runs/results_gazebo_final_v6_aligned_b

server replay-v2:
/data/users/csynth/cbf_rover_runs/results_gazebo_final_v6_aligned_replay_v2

server render:
/data/users/csynth/cbf_rover_runs/results_gazebo_final_v6_aligned_render

local verified copy:
E:\6_Engineering_Projects\CBF-Rover\runs\
  gazebo_final_v6_aligned_bf863c7c
```

The accepted results live outside the Git working tree. The local copy passed
SHA-256 verification against both run manifests, the replay report and render
manifest. No staging, commit, push or deployment was performed.

## 11. Claim boundary

Accepted means the declared physical Gazebo experiments and bounded replay
gates passed. It does not mean:

- the empirical tracking envelope is a proved invariant set;
- the runtime online CBF is a q-SBC probability certificate;
- Gazebo contact dynamics equal real hardware;
- ROS2 integration has been built or tested;
- a real rover has been tested.

The fixed-geometry q-SBC chain remains `not_ready`. The accepted Gazebo slice
is a separate empirical obstacle-avoidance result.
