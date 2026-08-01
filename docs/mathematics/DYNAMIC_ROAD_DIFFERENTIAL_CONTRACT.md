# Dynamic-road differential-drive CBF contract

## Material Passport

- Contract ID: `DR2_LOOKAHEAD_DIFFERENTIAL_CBF_V1`
- Status: frozen before first server execution
- Base geometry: `DR1_TWO_LANE_RANDOM_OBSTACLES_V1`
- Model: stochastic Euler differential drive
- Claim class: exact-backend empirical nonholonomic validation
- Explicit exclusions: no q-SBC, Gazebo, or real-vehicle claim

## 1. Why this stage exists

The DR1 result directly applies a safe world-frame velocity to a stochastic
single integrator. A differential-drive rover cannot independently realize
world-frame `u_x` and `u_y`; it accepts bounded linear and angular velocity.
DR2 introduces that nonholonomic constraint without changing the accepted road
geometry or random-obstacle generator.

This is a new experiment family. DR1 evidence is retained, but its safety claim
is not inherited automatically.

## 2. State and transition

The body state is

```text
z = [x, y, theta].
```

At `dt=0.05 s`, the declared exact-backend model is the explicit Euler
transition

```text
x+     = x + dt v cos(theta) + psi_x
y+     = y + dt v sin(theta) + psi_y
theta+ = wrap(theta + dt omega),
```

where `0 <= v <= 0.5 m/s`, `|omega| <= 1.5 rad/s`, and position noise is the
same N1 independent symmetric triangular process used by DR1. Heading noise is
absent in this first nonholonomic slice. Reverse motion and post-controller
clipping are forbidden.

## 3. Lookahead-point lifting

For lookahead distance `ell=0.15 m`, define

```text
p = [x + ell cos(theta), y + ell sin(theta)].
```

Its instantaneous velocity is

```text
p_dot = e(theta) v + ell e_perp(theta) omega.
```

At every step, the admissible lookahead velocity set is represented explicitly
by four heading-dependent halfspaces for the `v` and `omega` bounds, intersected
with the inherited `||u||_2 <= 0.5 m/s` disk. The online CBF projection solves
inside this reachable set. The selected velocity is then inverted exactly:

```text
v     = e(theta)^T u
omega = e_perp(theta)^T u / ell.
```

The command is not clipped after this mapping. A round-trip mapping residual is
recorded at every transition.

## 4. Conservative body geometry

The CBF is evaluated at `p`, not at the rover center. To protect the complete
body disk, every obstacle certificate radius and each road-edge contraction
receive an additional `ell=0.15 m` inflation.

If the inflated lookahead point is safe, the triangle inequality implies that
the original rover disk is safe. Physical body clearance and road departure
are still measured independently at the actual center.

## 5. Discrete remainder and stochastic robustness

The Euler lookahead transition differs from its instantaneous linear map by

```text
r = ell (
  e(theta + dt omega)
  - e(theta)
  - dt omega e_perp(theta)
).
```

The CBF disturbance support adds the worst-case coordinate bound

```text
ell * norm([
  cos(dt omega_max) - 1,
  sin(dt omega_max) - dt omega_max
])
```

to the N1 half-width. This is conservative because each coordinate magnitude
is no greater than the vector norm. Each trace also records the realized
one-step contraction residual for the actual successor, rather than relying
only on the projected linear inequality.

## 6. First-run gates

The discovery run passes only when:

1. the center reaches the finish line within 2,000 steps;
2. all five obstacles activate;
3. collision and road-departure counts are zero;
4. every lifted barrier is at least `-1e-9 m`;
5. every projected CBF residual is at least `-1e-9 m/s`;
6. every realized successor contraction residual is at least `-1e-9 m`;
7. every command satisfies the declared `v, omega` bounds;
8. the maximum lookahead mapping residual is at most `1e-12 m/s`;
9. no post-projection command clipping occurs;
10. an independent same-seed replay has the identical trace SHA-256.

Seed `7311127` is a deliberately difficult discovery seed selected from the
previous DR1 held-out set. It cannot support a new held-out generalization
claim. A separate, predeclared seed catalog is required after discovery fixes
are complete.

## 7. Claim boundary

Passing DR2 establishes reproducible empirical behavior only for the declared
Euler differential-drive model and position-noise process. It does not include
wheel dynamics, slip, latency, ROS2 scheduling, Gazebo physics, or real
hardware. Those effects require a measured tracking-error envelope and another
obstacle-inflation gate.
