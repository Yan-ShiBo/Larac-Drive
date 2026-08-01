# Dynamic-road online CBF contract

## Material Passport

- Contract ID: `DR1_ROBUST_DISTANCE_CBF_V1`
- Scenario ID: `DR1_TWO_LANE_RANDOM_OBSTACLES_V1`
- System: M1 stochastic single integrator
- Status: implementation contract
- Claim class: runtime online CBF plus empirical trajectory evidence
- Explicit exclusion: this contract is not the static P1/D1 q-SBC theorem

## 1. Research question

Can the M1 rover travel along a two-lane road while seeded random obstacles
become visible during the run, with the active obstacle certificate and its
effect on the commanded action shown at every `0.05 s` simulation step?

## 2. Scope boundary

The existing P1/D1 certificate is bound to three fixed unsafe boxes and a
frozen monitor-only controller. Random obstacle geometry changes the unsafe
set, so neither its barrier polynomial nor its q bound is inherited here.

DR1 is a separate runtime action-filter experiment:

- the obstacle plan is deterministic for a recorded seed;
- only revealed obstacles enter the online constraint set;
- the distance CBF is recomputed from the measured state every step;
- the filter may modify the nominal action;
- the returned action already belongs to the S1 speed disk, so no later
  clipping is allowed;
- the rendered `h` values and CBF residuals are runtime facts, not
  instantaneous collision probabilities and not a q-SBC probability bound.

## 3. Geometry and random appearance

The rover center travels in the road-center safe strip obtained by shrinking
the physical road edges by the rover circumscribed radius `r_rover = 0.31 m`.
Five circular obstacles are sampled from the declared x anchors, x jitter,
lane centers, radius interval, and appearance-distance interval. An obstacle
becomes active when

```text
x_rover >= x_obstacle - appearance_distance.
```

The complete hidden plan is written before the trace and bound by SHA-256.
Rendering hides inactive obstacles and shows each obstacle, its physical
radius, and its certificate radius only after activation.

## 4. Online robust distance CBF

For an active obstacle with center `c` and certificate radius

```text
R = r_obstacle + r_rover + safety_margin,
```

define

```text
d = x - c
n = d / ||d||_2
h(x) = ||d||_2 - R.
```

For `x+ = x + dt*u + w`, convexity of the Euclidean norm gives

```text
h(x+) >= h(x) + dt*n^T*u + n^T*w.
```

N1 has `|w_x|, |w_y| <= a`, hence

```text
n^T*w >= -a (|n_x| + |n_y|).
```

The online filter therefore imposes the sufficient one-step condition

```text
n^T*u >= -(kappa/dt) h(x)
         + (a/dt) (|n_x| + |n_y|),
```

with `kappa = 0.20`. If `h(x) >= 0` and the constraint is met, this lower
bound implies `h(x+) >= (1-kappa)h(x) >= 0` for every disturbance in the
declared N1 support. The two road-edge barriers use the same derivation.

At each step the filter solves

```text
minimize_u  ||u - u_nom||_2^2
subject to  A u >= b
            ||u||_2 <= 0.5.
```

Because the decision is two-dimensional, the implementation enumerates the
finite set of possible Euclidean projections: the nominal point, individual
constraint boundaries, pairwise line intersections, the speed-circle
projection, and line-circle intersections. The feasible candidate closest to
`u_nom` is selected deterministically. There is no post-filter clipping.

## 5. Live evidence

Every trace frame records:

- measured state and N1 noise;
- newly activated and currently active obstacle identities;
- nominal and filtered actions and intervention norm;
- every obstacle and road-edge barrier value;
- every CBF constraint residual;
- the minimum active barrier and minimum residual;
- collision, road-boundary, goal, and termination facts.

The live figure shows the road, lane markings, rover path, visible physical
obstacles, certificate-radius circles, minimum `h`, minimum CBF residual, and
filter intervention. It must be rendered at `1x` simulated time.

## 6. Acceptance gates

A single-seed demonstration is accepted only when:

1. all five planned obstacles activate;
2. the finish line is reached within 2,000 steps;
3. collision count and road-departure count are zero;
4. minimum certificate barrier is at least `-1e-9 m`;
5. minimum constraint residual is at least `-1e-9 m/s`;
6. no post-filter clipping occurs;
7. the online CBF changes the nominal action on at least one step;
8. two no-wait replays with the same seed produce identical trace SHA-256;
9. the `1x` trace has the same SHA-256 as the no-wait replay.

Passing these gates establishes a deterministic, reproducible empirical
simulation result for the recorded seed. It does not establish a
random-geometry q-SBC theorem or Gazebo tracking guarantee.
