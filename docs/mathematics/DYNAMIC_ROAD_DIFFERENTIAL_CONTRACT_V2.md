# Dynamic-road differential-drive CBF contract V2

## Material Passport

- Contract ID: `DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V2`
- Status: frozen before V2 server execution
- Parent: `DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V1`
- Change class: discovery-driven controller timing revision
- Claim class: exact-backend empirical nonholonomic validation

## 1. V1 failure retained

V1 stopped safely at step `146` on discovery seed `7311127` with
`cbf_action_set_infeasible`. At the terminal state:

- only the first two obstacles had activated;
- collision and road-departure counts were zero;
- minimum lifted barrier was `0.02548614157750076 m`;
- minimum physical clearance was `0.3754525532360031 m`;
- all completed-step CBF and realized-contraction residuals passed.

The first obstacle was about `0.72 m` ahead of the lookahead point, but the
inherited DR1 lane supervisor waited until `0.6 m` to request a lane change.
The robust inflated CBF already required a backward velocity component while
the forward-only differential contract prohibited reverse motion. The
intersection was therefore genuinely empty.

V1 remains immutable failure evidence. Its safety thresholds, inflation,
command limits, and solver were not relaxed.

## 2. Single V2 change

V2 adds an explicit differential-specific deterministic controller:

```text
controller id                 DR2_EARLY_LANE_SUPERVISOR_V2
lane-switch lookahead         2.4 m
forward speed                 0.42 m/s
maximum nominal speed         0.46 m/s
lateral gain                  1.6 1/s
obstacle pass margin          0.45 m
finish slowdown distance      0.75 m
```

The `2.4 m` threshold starts nonholonomic lane changes while the reachable
lookahead-velocity set still has turning authority. It remains below the
declared `2.8..3.2 m` obstacle appearance range, so the controller does not use
hidden obstacles.

All other V1 terms remain fixed:

- `dt=0.05 s`, N1 position noise, no heading noise;
- `0 <= v <= 0.5 m/s`, `|omega| <= 1.5 rad/s`, no reverse;
- `ell=0.15 m` lookahead and equal body inflation;
- Euler heading-remainder support added to N1;
- no post-projection command clipping;
- identical acceptance thresholds and claim exclusions.

## 3. V2 discovery gate

V2 must first pass deterministic replay on seed `7311127`. If it passes,
additional discovery seeds may be used to tune the controller. Only after the
implementation and controller are frozen will a new, previously unused
held-out seed catalog be declared.
