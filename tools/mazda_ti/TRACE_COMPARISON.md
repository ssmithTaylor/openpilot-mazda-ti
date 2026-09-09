# Portable trace comparison

`python -m tools.mazda_ti.trace_compare` compares three retained version-1 adapter bundles: the original reference, candidate, and equivalent current-control arm. Each bundle needs `result.json`, `trace.json`, and `trace.jsonl`; the command writes a new portable `result.json` and `report.md`.

Trace capabilities are optional. A metric absent from an active trace is `unavailable`; a trace with no active rows is `inactive`. Neither condition is reported as zero. The unified optional row fields are request/retained-reference/integral/compensation/carryover counts, clipping, constant-command exposure, and TI/stock command counts.

Each arm records normalized evidence-root-relative references and content hashes for its retained result and trace artifacts. Without `--data-root` full-bundle verification, the aggregate is `content_bound_unqualified`: useful for comparison but not release qualification. When an arm is a complete adapter bundle, pass `--data-root` to require its normal source/input/raw verification; only then can the result record `verified_full_bundle` identity validation.

Declare entry, sustained, unwind, and recovery windows in `trace.json`. When absent, the command applies its documented chronological four-way rule and records inferred uncertainty. Result sections keep invariant failures, recorded-motion command effects, and rider physical observations separate. It reports per-metric worst deltas and no scalar score; an earlier release is not a handling improvement claim.
