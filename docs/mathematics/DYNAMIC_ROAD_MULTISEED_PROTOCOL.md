# Dynamic-road multi-seed stress protocol

## Material Passport

- Protocol ID: `DR1_MS1_MULTISEED_STRESS_V1`
- Scenario: `DR1_TWO_LANE_RANDOM_OBSTACLES_V1`
- Status: frozen before execution
- Claim class: exact-backend empirical validation
- Explicit exclusion: not a random-geometry q-SBC theorem

## 1. Objective

The accepted seed `7302026` proves only that one deterministic random plan and
noise stream can be reproduced. This protocol tests whether the same frozen
online-CBF implementation completes a broad, predeclared seed catalog without
collision, road departure, CBF infeasibility, or timeout.

## 2. Seed separation

Two non-overlapping contiguous catalogs are frozen in
`configs/experiments/dynamic_road_multiseed.yaml`:

- discovery: 256 seeds starting at `7310000`;
- held-out validation: 1,024 seeds starting at `7311000`.

Discovery results may guide implementation fixes and therefore cannot support
the final empirical success-rate statement. No implementation or scenario
parameter may change after inspecting the held-out validation catalog. A code
change after validation begins invalidates that validation run and requires a
new held-out catalog.

## 3. Per-seed facts

Every seed produces one deterministic summary row containing:

- seed and five-bit obstacle lane pattern;
- termination reason and transition count;
- activated-obstacle, collision, and road-departure counts;
- minimum certificate barrier and physical clearance;
- minimum online-CBF constraint residual;
- intervention steps and maximum intervention;
- maximum filtered speed;
- obstacle-plan content SHA-256;
- canonical trace SHA-256.

Full traces are retained automatically for failed episodes up to the declared
limit. Successful traces are represented by their canonical hashes; the
worst successful seed is replayed separately for full live rendering.

## 4. Batch gates

The held-out validation batch passes only when:

1. all 1,024 seeds reach the finish line;
2. collision, road departure, timeout, model breach, and CBF-infeasible episode
   counts are zero;
3. every episode satisfies `h_min >= -1e-9 m`;
4. every evaluated CBF residual is at least `-1e-9 m/s`;
5. every filtered action remains inside `0.5000000001 m/s`;
6. all 32 possible five-obstacle lane patterns occur;
7. an independent replay produces an identical `episode_metrics.jsonl`
   SHA-256.

The intervention count is reported but is not a per-seed pass condition. A
safe nominal trajectory need not be modified merely to make the CBF appear
active.

## 5. Statistical interpretation

If all `n=1024` held-out seeds succeed, the exact one-sided 95% Clopper-Pearson
lower confidence bound is

```text
p_lower = 0.05^(1/n).
```

This interval describes success probability under this fixed seeded sampling
procedure and implementation. It is not a worst-case guarantee over every
possible obstacle geometry or disturbance, and it is not the q parameter of a
stochastic barrier certificate.

## 6. Reproducibility and monitoring

Batch execution is serial and ordered by ascending seed. A partial JSONL file
grows once per episode, and `progress.json` is atomically updated every 16
episodes. The server command has a 30-minute hard timeout. Process liveness,
RSS, CPU, progress count, and output growth are monitored; crashes and failed
episodes are not silently retried.
