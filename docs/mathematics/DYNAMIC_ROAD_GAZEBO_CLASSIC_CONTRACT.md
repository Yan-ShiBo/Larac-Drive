# Dynamic-road Gazebo Classic measured-state CBF contract

## Material Passport

- Contract family: `GZ2_DYNAMIC_ROAD_LIFTED_CBF`
- Active run contract: `GZ5_DISCOVERY_CALIBRATED_AUDITED_STARTUP_CBF_LIVE_DISPLAY_V13`
- Multi-seed contract: `GZ5_DISCOVERY_CALIBRATED_MULTISEED_V4`
- Replay contract: `GZ5_DISCOVERY_CALIBRATED_REPLAY_VALIDATION_V8`
- Backend: Gazebo Classic 11, ODE, headless paused fixed-step
- Claim class: empirical Gazebo differential-drive closed loop
- Explicit exclusions: no q-SBC probability claim, no ROS2 result, no real-vehicle
  claim

## 1. Purpose

This slice answers one narrow question: can the dynamic-road online obstacle
CBF be evaluated from the measured pose of a physical differential-drive model
at every control step, while the rover reaches the end of the seeded road with
randomly placed obstacles and without post-CBF command clipping?

It does not convert the online CBF into the fixed-geometry q-SBC certificate.
It also does not prove a differential-drive tracking-error bound. Gazebo
evidence remains empirical even when every runtime gate passes.

## 2. Measured state and protected point

Gazebo supplies the measured body state

```text
s_k = [x_k, y_k, theta_k]
```

after each fixed physics interval. The protected lookahead point is

```text
p_k = [x_k + ell cos(theta_k), y_k + ell sin(theta_k)]
ell = 0.15 m.
```

Its nominal kinematic velocity under command `[v, omega]` is

```text
u = A(theta) [v, omega]^T

A(theta) = [[cos(theta), -ell sin(theta)],
            [sin(theta),  ell cos(theta)]].
```

The action projection includes the reachable differential-drive halfspaces,

```text
0 <= v <= 0.5 m/s
|omega| <= 1.5 rad/s
||u||_2 <= 0.5 m/s.
```

The projected lookahead velocity is mapped once to `[v, omega]`. The Gazebo
adapter sends that command directly to the transport plugin; it does not clip
or repair the command afterward.

## 3. Online distance barriers

Only obstacles inside their seeded appearance range are known to the
controller. All five cylinders exist in the physical world from startup, but
an inactive obstacle cannot influence the nominal controller or CBF.

For an active obstacle `j`, the displayed and evaluated lifted barrier is

```text
h_j(p) = ||p - o_j|| - r_certificate,j - ell.
```

The road edges use the same additional `ell` body inflation. Let `n` be the
outward-safe unit normal of a barrier. At every measured state the filter
enforces

```text
n^T u >= -(kappa / dt) h
           + (delta_eff / dt) (|n_x| + |n_y|)

kappa = 0.20
dt = 0.05 s.
```

The minimum-change feasible `u` is obtained by exact two-dimensional projection
onto the CBF halfspaces, differential reachable set and speed disk.

## 4. Gazebo tracking support

The base DR2 support already contains N1 and the bounded Euler heading
remainder:

```text
delta_base = 0.005421809086151014 m.
```

The earlier V5-V10 envelope (`0.04 m`, `0.10 rad`) was conservative enough to
create a safe deadlock on repair seed `7320014`. V12 therefore evaluated a
tighter support over the complete 32-pattern discovery catalog. Its position
maximum remained below `0.016 m`, while one heading residual reached
`0.05563104052672907 rad` and correctly failed the then-frozen `0.055 rad`
gate.

V13 freezes the following outward-rounded discovery-calibrated envelope:

```text
coordinate position half-width = 0.016 m
heading error bound             = 0.065 rad
lookahead heading displacement  = 2 ell sin(0.065 / 2)
                                = 0.009748283684395426 m

delta_eff = delta_base + 0.016 + 0.009748283684395426
          = 0.03117009277054644 m.
```

The V12 source trace and catalog-metrics hashes are bound in the V13 config.
The envelope is checked on every run, but it is still calibrated from finite
Gazebo evidence. It must be labeled empirical, not formal.

## 5. Physical and time contract

```text
base box                 0.48 x 0.38 x 0.12 m
base mass                8.0 kg
wheel radius             0.10 m
wheel separation         0.44 m
wheel axle x             0.10 m
rear caster x           -0.19 m
conservative body disk   0.31 m
ODE physics step         0.001 s
physics steps / command  50
control period           0.05 s
maximum control steps    2000
```

The base half diagonal is approximately `0.3061 m`, so the declared `0.31 m`
scenario body disk covers the planar box. Wheels, joints, caster, cylinder
obstacles and contacts are physical SDF entities.

The standalone C++ bridge owns only Gazebo transport and stepping. Python owns
the CBF/controller mathematics. The bridge protocol is:

```text
READY GZCB1 <world> <model>
STATE sim_time iterations x y z roll pitch yaw
STEP v omega physics_iterations
GET
STOP
```

Each experiment uses an isolated non-default loopback master URI and terminates
its own `gzserver` and bridge processes. It must never attach to or stop another
user's default Gazebo master.

V6 onward requires `GAZEBO_IP=127.0.0.1` and the companion timestamped-state
model plugin. The plugin defers transport initialization until the first world
update. The bridge performs exactly one zero-command bootstrap physics
iteration.

V6 and V7 gated every step on the lower-frequency `world_stats` topic before
requesting the terminal pose. That ordering was correct but added about
`0.20 s` wall time to every `0.05 s` control step. V8 retains `world_stats` as
a startup health check and carries the exact `World::Iterations()` value as
the pose message ID on the private state topic. For each command, the bridge
computes the target iteration from the preceding timestamped pose, advances
exactly 50 physics iterations, and polls without advancing physics until it
receives:

```text
pose_iterations >= target_iterations
```

V9 removes the remaining startup dependency on a potentially missed
`world_stats` message: readiness is established by the bounded timestamped
pose-iteration response itself. The returned protocol state uses simulation
time and iteration from that same pose sample.

V13 adds a bounded infrastructure policy around this startup only. At most
three attempts are allowed before an initial state exists. Every failed or
selected attempt preserves bridge and `gzserver` logs with hashes in
`startup_attempts.json`. After an initial state is selected, no control step,
trajectory, stale response, unsafe state, or timeout may be retried. The
control command, ODE settings, controller, CBF, and public `GZCB1` records are
otherwise unchanged.

## 6. Single-run acceptance

Every V13 run must satisfy the physical gates:

- termination is `goal_reached`;
- all five seeded obstacles activate;
- collision and road-departure counts are zero;
- minimum lifted barrier, CBF constraint residual and measured contraction
  residual are at least `-1e-9`;
- mapping residual is at most `1e-12 m/s`;
- command bounds hold with no post-projection clipping;
- maximum position and heading tracking residuals are at most `0.016 m` and
  `0.065 rad`;
- absolute roll and pitch remain at most `0.05 rad`.
- `startup_attempts.json` is valid and no control retry occurs after the
  initial state.

It must also satisfy five live-display gates:

- the runtime display reaches `completed`;
- the display update count exactly matches the configured stride plus the
  terminal frame;
- the terminal display step and simulated time match the Gazebo trace;
- display observations neither modify actions nor feed data back to control;
- bind host, port, stride, refresh interval and pacing policy match the V13
  configuration.

A passing single run is `provisional_success`, not final acceptance.

## 7. Independent replay acceptance

Replay-v8 requires two
separate result directories and validates:

- both manifest content hashes and every referenced plan/world/trace file hash;
- both startup-attempt files and their retry-scope identities;
- identical run config, base config, scenario, seed, obstacle plan, SDF world
  and runtime contract;
- both runs pass every single-run gate and reach the goal;
- transition-count spread is at most 2 and simulated-duration spread at most
  `0.10 s`;
- final center-position spread is at most `0.05 m` and final-heading spread at
  most `0.10 rad`.

Gazebo ODE traces are not required to be byte identical. Exact trace equality
would confuse empirical simulator repeatability with a formal determinism
claim. Final status becomes `accepted` only when the separate replay report
passes every gate.

## 8. Visualization contract

V13 publishes an atomic `snapshot.json` while the Gazebo episode is running and
serves an external-dependency-free Canvas dashboard on
`http://127.0.0.1:11469/`. Every displayed state comes from the already
evaluated measured-state CBF frame. The display sink is downstream of control
and has no control-feedback path.

Every runtime view shows:

- physical rover footprint and measured body/lookahead trajectories;
- only currently active random obstacles;
- physical obstacle disks and the live inflated certificate boundary;
- minimum lifted barrier, realized contraction residual;
- commanded `[v, omega]`, nominal command and intervention state;
- run status, sequence, step, simulated time and newly activated obstacle ids.

The server is loopback-only and stops after a terminal-snapshot grace interval.
The viewer, terminal snapshot, incremental event log and their hashes remain
in the result directory. A separate renderer can still verify the source run
manifest, trace hash, config hash and seeded scenario before producing a
post-run GIF and final PNG.

“Live” means values are displayed while the controlling simulation process is
alive. The V4 batch makes wall-clock pacing and an achieved real-time factor in
`[0.95, 1.01]` aggregate gates. The executed V13 discovery and held-out
validation catalogs both passed `32/32`; replay-v8 accepted the selected
held-out member. The full evidence and rendering hashes are recorded in
`docs/experiments/DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md`.
This is performance and runtime-display evidence, not a formal safety claim.
