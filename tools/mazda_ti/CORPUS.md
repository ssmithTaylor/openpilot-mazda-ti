# Versioned mixed corpus

`python -m tools.mazda_ti.corpus` runs the historical and instrumented adapters serially,
qualifies each original baseline, evaluates one pinned candidate, and writes per-case
bundles plus a corpus coverage report. It does not change controller policy or operate
a device. The original baseline is the control arm in version 1; a distinct policy
control arm, parallel execution, caches and resume are separate extensions.

```text
python -m tools.mazda_ti.corpus --manifest tools/mazda_ti/examples/corpus-v1.json --candidate-revision ad14961a29f59635d5f657a43000ca2a1ade62fb --profile command-regression --data-root PATH/TO/rlogs --evidence-root PATH/TO/curve-review-20260907 --exposure-ledger PATH/TO/exposure-ledger.json --output NEW/evidence
```

The versioned reference contains historical VW, ordinary-road and hard-right cases,
instrumented TI loss/recovery with engagement boundaries, the outward-crossing rider
case, and the retained broad04 latest-history one-count failure. The latter is an
explicit exclusion referencing its original failed result; it never runs a candidate.
No adjacent-Float32 exception or integer tolerance is introduced. New failed baselines
remain in their case bundles and cannot contribute completed coverage. In this reference,
the outward-crossing case reproduces both integer histories but fails exact serialized
controller-output equality under its original 834-second anchor. Its rider evidence is
retained, and no candidate result is supplied for that unqualified case.

`command-regression` requires the listed command cases and categories. The stricter
`representative-validation` profile also requires every case, successful physical
corner coverage and an unexposed withheld case. The supplied corpus explicitly lacks
rider-confirmed successful matched passes and untouched withheld cases, so that profile
must fail. A command profile passing does not fill those gaps or establish cornering
success. Missing raw bytes or annotation evidence also prevents required coverage.

The manifest declares `format_version`, stable `id` and `version`, `cases`, `sources`,
`unavailable` categories and named `profiles`. Each case embeds the existing version 1
case contract: full original/warmup/limiter revisions, settings whitelist, raw relative
paths and SHA-256 hashes, one history, initialization and scoring windows. It additionally
declares its original `control_revision`, categories, development/withheld group,
prior exposures, annotations, expected minimum sends/stock coverage, and any exclusion.

Annotations carry a kind, value, confidence, source identity and monotonic interval.
The source table supplies a relative evidence path, SHA-256 and evidence kind. Confirmed
contact and physical outcomes require rider provenance, not an analysis of TI signals.
The outward-crossing source preserves the rider's corrected intended left lane and
outward crossing into the right lane, no intervention during the crossing, subsequent
catches near 859 and 868 seconds, approximate timing, and unknown contact ends. Historical
right-corner contact near 2879 seconds remains approximate. Unknown lane positions are
not filled with a lane-center assumption or an invented corridor width.

Speed/entry annotations in the reference were calculated from raw carState over each
scoring interval: speed min/median/max, then median speed and steering angle in its first
second. Their source points to a raw segment; the case pins the complete adjacent input
set used in that calculation. These are recorded entry context, not matched-condition
A/B comparisons or independent body-clearance measurements. Sources and unrestricted
raw data stay local. Supply the original campaign annotations and failed result under
the declared relative paths; only their identities and curated annotations enter Git.

Keep one durable exposure ledger across runs. It records input-content hashes before
attempting each case, even when checks fail, and carries across corpus IDs and versions.
Any overlap with previously evaluated input bytes prevents an unexposed withheld claim.
Cases with declared prior inspection belong to development. The output retains ledger
before/after snapshots and marks current-run exposure; it never calls an evaluated case
untouched afterward. Preserve this ledger and incorporate other human/tuning exposure
into the manifest: deleting a ledger cannot prove that data was previously unseen.
Use a single writer for a ledger; concurrent invocations are unsupported in this slice.

The output preserves each adapter's four statuses and integer rules. Required-profile
failure is `failed_check`, with per-case execution failures, exclusions and unsupported
coverage visible. Case reports retain fixed recorded motion and availability limitations.
The corpus report keeps annotations separate from candidate command changes and reports
no weighted physical score. Each completed case is independently bundle-verified during
the invocation. Source/runtime/raw provenance remains in its bundle; manifest and
exposure identities are retained alongside the aggregate result.

Default tests exercise both real adapters using deterministic synthetic serialized logs,
required raw/annotation gaps, exposure reuse, retained exclusions and rejection of
TI-derived confirmed contact. Existing GitHub Actions full-suite discovery includes them.
