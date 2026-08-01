# Polynomial SBC method boundary

This package implements the frozen Rover-first polynomial stochastic barrier
certificate chain after the research contract fixed:

- the discrete-time system and sampling period;
- state and action ordering, units, and frames;
- initial, unsafe, target, and applicability regions;
- the noise model or moment assumptions;
- the exact certificate inequalities and the meanings of `q` and `epsilon`;
- artifact compatibility fields and independent validation requirements.

The active C2 controller is a fixed three-piece polynomial map. Runtime uses
ordered first-match semantics at shared x boundaries; CV1, SOS synthesis, and
independent replay cover both adjacent closures. The controller is immutable
during synthesis, so the decision variables remain W, q, epsbar, and SOS Gram
matrices.

The active G10 first-slice route instead fixes q strictly below exact 1/10 and
epsbar at its frozen binary64 value, then maximizes a common rho over Chebyshev W
coefficients and nonnegative local Bernstein multiplier coefficients. Its finite
W14/M2 -> W16/M2 -> W16/M4 schedule cannot publish without V3 zero-tolerance
replay and barrier/controller/proof-tree hash bindings.

G10R is an independent, projection-equivalent reduction of G10. It removes only
the two nonnegative rectangle-guard multipliers that were attached to the same
box already used for Bernstein positivity. It retains every initial-disk and
goal-exterior shape multiplier and leaves the schedule, scalars, controller, G4
obligations, and V3 gate unchanged. The legacy `solve-qsbc-g10` command remains
available; the reduced route is `solve-qsbc-g10r`.

G10E is the independent, finite next-order fallback. It reuses the exact G10R
projection argument and every frozen theorem input, but searches only
W18/M4 -> W20/M4. It stops at the first V3-validated certificate; two failed
stages produce `not_ready`, with no automatic degree or multiplier escalation.
Its route is `solve-qsbc-g10e`.

G10A is a separate active-set fallback. A hash-bound G9 diagnostic barrier is
used only for warm-start and point-row ranking. Every fourth outer iteration
runs the unchanged V3 validator; only a bound `proved_violation` or `unresolved`
exact leaf box may add hard Bernstein rows for the original residual. Solver
and active-row results remain non-formal. Its route is `solve-qsbc-g10a`.

G10A2 is an independent W16 pre-separation fallback. It fixes `rho=0`, projects
the Chebyshev coefficients toward the hash-bound G9 seed with a two-phase
L-infinity/L1 objective, and adds only selected exact-negative coefficient
cuts over the frozen D3/D4/D5 catalog. A complete D5 gate may permit exactly
one unchanged V3 call, but neither the preseparator nor its ledger is formal
evidence. Its route is `solve-qsbc-g10a2`.

G10A3 is a separate exact-rational grid multipoint fallback for the same
global W16 subject. Its fixed G17/G33/G65 catalogs select spatially distributed
zero-rho point cuts, then run one complete G65 diagnostic gate. Grid and LP
results remain non-formal; only the unchanged V3 can publish. Its route is
`solve-qsbc-g10a3`.

G11P49 is an independent continuous piecewise-barrier route. A shared control
net makes the 7-by-7 obstacle-aligned partition C0 by construction, while 148
quadrant roof witnesses upper-bound the stochastic next-state barrier. It uses
schema 3 and the independent V3P validator; legacy V3 cannot validate or
publish this subject. Its route is `solve-qsbc-g11p49`.

`certificate.py` is a pure monitor-only evaluator. It never modifies an action.
No artifact is formal merely because a solver returned a result: hash identity,
controller validation, theorem scalars, and solver-independent Bernstein replay
must all pass. Gazebo differential tracking remains empirical.
