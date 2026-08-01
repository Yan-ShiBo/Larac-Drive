# Dynamic-road differential-drive multi-seed protocol

## Material Passport

- Protocol ID: `DR2_V2_MS1_MULTISEED_STRESS_V1`
- Differential contract: `DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V2`
- Status: frozen before execution
- Claim class: exact-backend empirical nonholonomic validation
- Explicit exclusions: no q-SBC, Gazebo, or real-vehicle claim

## 1. Objective

The accepted V2 run uses a deliberately difficult but previously inspected
seed. It establishes model functionality, not generalization. This protocol
tests the frozen forward-only differential controller, lifted online CBF, and
Euler stochastic transition on new seed catalogs.

## 2. Catalog separation

The authoritative catalog is
`configs/experiments/dynamic_road_differential_multiseed.yaml`:

- discovery: 256 seeds `7320000..7320255`;
- held-out validation: 1,024 seeds `7321000..7322023`.

Neither interval was used by DR1 or the differential V1/V2 discovery runs.
Discovery failures may guide a V3 implementation or controller. Any change
after held-out validation begins invalidates the held-out run and requires a
new catalog.

## 3. Per-seed evidence

Each deterministic record contains:

- seed, five-obstacle lane pattern, obstacle-plan identity, and trace identity;
- termination, transition count, and obstacle activation count;
- body collision, road departure, and physical clearance;
- minimum lifted barrier, projected CBF residual, and realized one-step
  contraction residual;
- maximum mapping residual, linear speed, and absolute angular speed;
- projection intervention and post-command clipping facts.

Up to 32 failing full traces are retained. Successful full traces are
represented by their canonical SHA-256, with the worst accepted seed replayed
later at 1× for rendering.

## 4. Batch gates

Held-out validation passes only if:

1. all 1,024 episodes reach the finish line;
2. collision, road departure, timeout, model breach, and CBF infeasibility are
   absent;
3. lifted `h >= -1e-9 m`;
4. projected CBF residual is at least `-1e-9 m/s`;
5. realized successor contraction residual is at least `-1e-9 m`;
6. mapping residual is at most `1e-12 m/s`;
7. `v <= 0.5000000001 m/s` and `|omega| <= 1.5000000001 rad/s`;
8. no command is clipped after projection;
9. all 32 lane patterns occur;
10. an independent replay has the identical episode-metrics SHA-256.

The tiny allowances above command limits are numerical comparison tolerances,
not applied clipping or larger physical command bounds.

## 5. Statistical interpretation

If all `n=1024` held-out seeds pass, the one-sided 95% exact lower confidence
bound is `0.05^(1/n)`. It applies only to the frozen seeded sampling procedure
and implementation. It is not the `q` of a stochastic barrier certificate and
is not a worst-case geometry guarantee.

## 6. Monitoring

Execution is serial and ascending by seed. The partial JSONL grows once per
episode and `progress.json` is updated every 16 episodes and on every failure.
The server wrapper enforces a 30-minute hard timeout. PID, CPU, RSS, progress,
output growth, and stderr are monitored; failures are never silently retried.
