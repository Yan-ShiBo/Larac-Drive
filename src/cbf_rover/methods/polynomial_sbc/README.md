# Polynomial SBC method boundary

This package is reserved for polynomial stochastic barrier-certificate work.
It intentionally contains no synthesizer, theorem implementation, evaluator,
or formal-safety claim.

Implementation begins only after the research contract fixes:

- the discrete-time system and sampling period;
- state and action ordering, units, and frames;
- initial, unsafe, target, and applicability regions;
- the noise model or moment assumptions;
- the exact certificate inequalities and the meanings of `q` and `epsilon`;
- artifact compatibility fields and independent validation requirements.

Until those items are explicit and validated, code in this package must not
label a result as a formal certificate.

The current episode runner's no-modifier path executes nominal actions without
certificate evaluation. A true `monitor_only` mode remains out of scope until
this package provides a theorem-specific evaluator.
