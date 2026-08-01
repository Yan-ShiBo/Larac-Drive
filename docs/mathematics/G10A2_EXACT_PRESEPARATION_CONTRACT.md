# G10A2 exact-Bernstein pre-separation contract

Status: **implemented locally; server execution and V3 proof pending**
Version: `G10A2-v1`
Date: `2026-08-01`

## Boundary

G10A2 is an independent W16 candidate route. It does not replace or mutate
G10A, G10E, the frozen C2 controller, the 42 q-SBC obligation families, `q`,
`epsbar`, or the exact-rational V3 validator. The G9 bundle is hash-bound and
supplies only the frozen Chebyshev seed and the same depth-3 diagnostic
geometry. The G9 barrier is never a proof subject.

The preseparator and every LP result are non-formal. A complete D5 scan can
reject a candidate or permit the single V3 call, but it cannot publish a
certificate.

## Projection LP

The fixed solver layout is:

```text
[c_0,...,c_152, rho, t, z_0,...,z_152]
```

`rho` is a dummy fixed to canonical `+0.0`; its objective coefficient is zero.
All G9 anchor rows and selected coefficient cuts are first constructed in the
core `[c,rho]` layout and then bitwise-zero-extended through every `t/z`
column. Selected cuts also have a bitwise-zero `rho` column.

For the hash-bound G9 seed `c_seed`, candidate generation uses two phases:

```text
phase 1: minimize t,  |c_i-c_seed_i| <= t
phase 2: minimize sum(z_i), |c_i-c_seed_i| <= z_i
         t <= nextafter(t_star,+inf) + 1e-9
```

There are four candidates and exactly eight LP solves: initial anchors, then
the results after D3, D4, and D5 cuts. Pure L1 minimization is not an allowed
first objective because server diagnostics showed that it can select a severe
unsafe oscillatory candidate. The formal validator reads only the resulting
barrier, never `rho`, `t`, `z`, the objective, anchors, or cuts.

## Exact finite catalog

The split rule is widest axis, with x on a tie, at the exact rational midpoint.
Every pass traverses the entire fixed-depth binary catalog for all 42 families:

| pass | depth | maximum new negative coefficients per active box |
|---|---:|---:|
| D3 | 3 | 1 |
| D4 | 4 | 2 |
| D5 | 5 | 4 |

Every visited leaf, including an outside-domain leaf, receives a canonical
identity containing the family, depth, domain classification, exact rational
box, and split-rule ID. The geometry catalog freezes the expected aggregate
leaf-set SHA-256 independently for D3, D4, and D5. Each runtime pass and the
final D5 gate recompute the scanned leaf-set hash and require exact equality;
matching counts alone are insufficient and any mismatch fails closed.

Only individual coefficient inequalities are added. Whole-box coefficient
blocks are forbidden. The maximum selected-row count is therefore:

```text
42*8*1 + 42*16*2 + 42*32*4 = 7056
```

Each selected constraint has two identities. `constraint_definition_id` binds
the family, exact box, coefficient index, target degree, numeric margin, and
zero-rho semantics. `selection_event_sha256` additionally binds the candidate
barrier hash, exact negative coefficient, and classification. Before float
projection, the exact coefficient is recomputed from the source-bound family;
the float row is then replayed against the same candidate.

The ledger self-validates its exact box identity, selection event, and content
hash. Its numeric-row digest records a source-bound rebuild performed during
ledger creation; it is provenance, not independently sufficient proof. The
unchanged V3 validator does not read this ledger.

After the fourth candidate, a separate full D5 gate scans the current V3
subject at its actual residual degrees. It must visit exactly `42*32 = 1344`
leaves before it can pass. Any active-domain negative Bernstein coefficient
returns `not_ready` without invoking V3.

## Fixed budgets and fail-closed rules

- aggregate LP wall time: 1800 s;
- aggregate exact preseparator wall time: 3600 s;
- complete stage wall time, including V3: 10800 s;
- maximum process RSS high-water mark: 1 GiB, measured as `VmHWM` from
  `/proc/self/status` before and after the unique V3 call and throughout exact
  traversal;
- master rows: 7710; one ephemeral lexicographic row gives 7711;
- decision variables: 308;
- CSR nonzeros: 1,087,873.

Deadline checks occur between families and throughout each box traversal.
Rows, nonzeros, RSS, and wall time are checked at runtime. Missing RSS evidence,
partial scans, solver errors, late solver/V3 returns, ledger mismatches, and all
unexpected exceptions fail closed. There is no automatic degree, pass, cut-cap,
or time-budget escalation.

## Sole publication authority

At most one unchanged V3 call is allowed, and only after the complete D5 gate.
If that V3 call is nonvalidated, late, malformed, or fails any subject/tree
binding, the route returns `not_ready` and never calls V3 again. Publication
requires all of the following:

```text
V3 report schema_version = 2
V3 validator_id = G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3
V3 status = validated
V3 first_slice_ready = true
V3 formal_acceptance_tolerance = 0
barrier/controller/q/epsbar/model/report/tree bindings all pass
unchanged V3 release gate = passed
```

Only then may G10A2 write a schema-v2 certificate. Runtime remains
`monitor_only`; the certificate evaluator never modifies the rover action.

Publication is transactional and remains inside the complete stage budget.
After the V3/tree/hash/release checks pass, G10A2 samples peak RSS and checks
the stage deadline again before writing `certificate.pending.json`. It repeats
both gates after the pending write, atomically renames that file to
`certificate.json`, and repeats the gates after the rename and immediately
before the ready manifest. Any late return, RSS breach, write error, rename
error, or manifest error enters the fail-closed handler, which removes the
route-owned pending, final, and internal temporary certificate filenames. A
`not_ready` result therefore cannot retain `certificate.json` or
`certificate.pending.json` as a misleading publication artifact.

Server command (use a new output directory):

```text
python -m cbf_rover.entrypoints.research solve-qsbc-g10a2 \
  --controller <frozen-controller.json> \
  --g9-seed <canonical-g10a-g9-seed-bundle.json> \
  --output-dir <new-empty-output-directory>
```

No local import, static inspection, or small-matrix test is a research result.
The first current claim remains `not_ready` until the server executes the route
and the unique unchanged V3 report passes every release gate.
