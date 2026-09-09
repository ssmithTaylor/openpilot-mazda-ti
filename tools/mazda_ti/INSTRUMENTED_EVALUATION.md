# Instrumented TI loss and recovery

The common [evaluation command](EVALUATION.md) accepts `method: instrumented`,
`history: exact`, and `fpcs_sample_at: diagnostic`. It promotes the full-controller loop
from the local `post-drive-290/replay_keepout_history.py` caller into `instrumented.py`.
It executes pinned original and candidate controller source with the shared state
migration and `RecordedTiFeedback`; it changes no steering policy or vehicle settings.

```text
python -m tools.mazda_ti.evaluate --request tools/mazda_ti/examples/cutout-evaluation.json --data-root PATH/TO/rlogs --output PATH/TO/evidence/cutout-01
python -m tools.mazda_ti.evaluate --verify-bundle PATH/TO/evidence/cutout-01 --data-root PATH/TO/rlogs
```

The reference pins original/warmup/candidate/limiter revision
`ad14961a29f59635d5f657a43000ca2a1ade62fb`, settings and raw SHA-256 identities for
route `00000290--9c1e61afa1`, segments 15–17. The raw data remains private and outside
Git. Warmup uses all supplied leading events, the once-only integral/friction anchor
starts at 990 seconds, activation is 1000, and scoring is [1000, 1070) monotonic seconds.
Expected original and same-source candidate: 7,000 controller updates, 6,999 simulated
applies, zero TI differences and zero stock differences. The first scored output
publication represents a pre-activation apply and remains recorded; it is not counted
as a simulated apply. Recorded selection contains one TI loss, one re-entry, 185 active
stock-fallback applies and 3,968 inactive applies.

Every controller update resolves all eight nested input identities, health snapshots
and the recorded model/camera clocks. Applied carControl identity, output and active
state must match its controller publication. Missing adjacent inputs, absent/version-zero
or unsupported instrumentation, future references, inconsistent validity, duplicate
identities and sequence/history corruption fail before candidate scoring. Health flags
remain distinct from identity completeness. No nearest-publication fallback is used.
Leading updates with unavailable identities may be skipped only before the declared
anchor, and their timestamps are retained in `trace.json`.

State initialization follows the original caller: recorded live-torque parameters and
runtime flags, exact raw curvature request, recorded active Kp, one post-update integral
and friction-gate anchor, and recorded previous TI/stock integer seeds. Controller gains
from CarParams retain their serialized precision. Candidate activation migrates the
existing controller state; inactivity resets the controller and both limiter histories.
TI requests reset during bypass and restart on re-entry; the candidate stock history
continues even while TI supplies feedback. Consumed feedback selects the exact previous
apply, and limited-state history compares before appending the current request.

Baseline qualification requires nonempty active coverage, exact Float32-serialized
controller output equality on every scored update (including inactivity), controller command residual
below one count, matching plant/limited state, and **exact integer equality on both TI
and stock paths**. The controller residual allowance cannot excuse a wire-count mismatch.
The actual pinned limiters validate individual transitions, including legitimate bypass
and inactive reset jumps. Stock wire requests use normalization 600; the measured EPS
response limit of about ±308 is a separate physical observation.

Bundles preserve the common four statuses, readable report and top-level provenance.
`trace-stock-sends.json` is required alongside TI sends; baseline qualification of both
paths blocks candidate execution if either fails. Raw hashes, source revisions/file
hashes, analyzer/dependency/schema hashes, runtime package/native hashes, initialization,
exact consumed identities and both clocks are retained. A source or runtime change
requires fresh qualification. Bundle verification checks these identities and rebuilds
the command comparison; it does not re-execute the recorded runtime.

TI availability, planner/sensor observations, apply schedule and physical motion remain
recorded. Candidate commands cannot predict whether a TI loss would be avoided, its
counterfactual recovery, a new vehicle path or successful cornering. These limitations
are present in the report even when both command histories reproduce exactly.

### Inspect compensation and integral carryover

New instrumented traces retain `recorded_components` and `replay_components` for active
version-1 controller diagnostics. `trace.json.component_trace` versions the selection and
declares native units. Inactive or unavailable components are null, rather than measured
zeros. These snapshots include the request, effective references, measurement, PID terms,
inverse command, compensation and final command. `limiter_feedback_limited` and
`consumed_feedback_steer` retain the replay's actual pre-update limiter state and the exact
consumed software feedback. TI/stock apply traces remain separate, keyed by apply time;
they must not be substituted for controller components by nearest-timestamp matching.

The shared trace comparison recognizes acceleration-domain request/reference/integral
metrics in `m/s^2` and inverse/final/compensation commands in TI counts. It aligns consumed
input identities and model/camera clocks as well as controller publication time. Integral
differences are not converted into counts through an assumed linear plant. Compare both
original and otherwise-equivalent current-control arms before attributing a difference to
a policy. A changed compensation mapping can change limiter feedback, freeze eligibility,
and subsequent integral history even after compensation itself returns to its old value.
Use declared physical phase evidence; chronological quarters alone cannot label corners.

The default suite includes a serialized synthetic zero-command transition, adjacent
segment joins and corrupted controller/apply/stock evidence. Independent nonzero
`test_stock_feedback.py` expectations exercise candidate stock evolution under TI,
bypass, re-entry, old consumed feedback and inactive reset. Set `MAZDA_EVAL_DATA_ROOT`
for the separate recorded reference test. Both existing GitHub Actions Mazda test jobs
discover the new module through the full `tools/mazda_ti` suite.
