# Mazda TI replay evidence tools

For every replay, batch or large test run, follow [scratch and retention](STORAGE.md).
Full outputs and caches are disposable; durable records remain compact and state
when full evidence must be rebuilt before verification.

## Fresh-session acceptance

[`FRESH_SESSION.md`](FRESH_SESSION.md) is the setup, run, and diagnosis guide for
the public end-to-end acceptance. It generates deterministic fixtures, relocates
their bytes into two fresh roots, exercises ingestion through release
qualification, measures cold and all-cache runs separately, and demonstrates
that incomplete process coverage remains unqualified.

## Isolated startup process boundary

[`STARTUP_EVAL.md`](STARTUP_EVAL.md) documents the isolated real `controlsd`
process-replay startup profile, its generated diagnostic timestamp checks, and
the narrower `carState` schema/subscriber check.

[`TRANSITION_EVAL.md`](TRANSITION_EVAL.md) documents the complementary active,
disengage/re-engage, TI bypass/re-entry, and unhealthy-message process evidence
profile. It retains fixed availability limits when a recorded segment lacks a
required transition.

[`RELEASE.md`](RELEASE.md) documents the versioned release qualification
boundary. It consumes hash-bound corpus, scenario, process, and comparison
results, preserves concerns and a specific physical question, and emits device
prerequisites as pending records. It never deploys or actuates.

For one command that prepares a pinned historical or instrumented case, qualifies its baseline,
evaluates a candidate, and writes a structured bundle plus readable report, see
[EVALUATION.md](EVALUATION.md). The supplied VW reference includes explicit input hashes,
settings, revisions, history, and scoring window.

For versioned mixed recorded cases, rider annotations, exposure and required coverage
profiles, see [CORPUS.md](CORPUS.md).

[TRACE_COMPARISON.md](TRACE_COMPARISON.md) documents the optional, versioned portable
trace comparison report and its equivalent current-control arm.

[SAMPLING_SENSITIVITY.md](SAMPLING_SENSITIVITY.md) compares a policy's incremental
effects under both compatible historical request timings, with four verified bundles.
Use it before attributing a plateau-related feedback or integral change to the policy.

## Raw-evidence ingestion

On Windows, `.ti-local/raw` may be a junction or other reparse point. The coarse
geographic inventory accepts an explicitly supplied final root junction or
symlink, records both the caller spelling and resolved target, and rejects
reparse ancestors and nested reparse entries. A route is unavailable only after
its exact requested segment paths have been checked in the resolved root.
Preserve the configured root and exact paths in the compact evidence.

Before attributing a candidate-only effect, run `candidate_attribution.py`'s
validator (or the equivalent library call). A historical recorded baseline may
use a different controller than the current candidate. In that case an
explicitly simulated equivalent-control arm is mandatory and must match the
candidate's controller parent, dependency hashes, preparation, warmup, settings,
history, and window. Mismatched arms are rejected; direct historical-to-candidate
deltas are not candidate-only evidence.

### Camera timing provenance

Use `camera_timing.py` to map a generated clip to exact `roadEncodeIdx` rows.
The input JSON contains only extracted `fullHEVC` rows with `segment_id`,
`frame_id`, `encode_id`, and SOF/EOF nanoseconds. The tool validates contiguous
frames, preserves before/after EOF brackets, and generates PTS from the declared
frame rate. Camera SOF/EOF clocks remain separate from generated clip PTS; the
tool does not infer lane position, rider contact, causality, or physical effect.
It records source hashes and any visibility transformation description. Keep
video and extracted rows in C: scratch under the storage policy; retain only
the compact mapping JSON in durable study storage.

### Rider review bundle

Use `rider_review.py build` with a source-locked request, byte-hashed manifest,
existing local video, and a camera mapping carrying matching route, segment, and
video identities to create a small static review page. Labels are
explicit observations (`smooth`, `scallop`, `held_inward`, `over_release`,
`intervention`, or `uncertain`); they never infer inside/outside, driver contact,
or TI causality. `rider_review.py validate` requires matching request/manifest
hashes, exact interval IDs and mapping brackets, confidence, and rider
provenance for physical labels. The tool refuses ambiguous or changed inputs
and never copies video into the repository.

### Rider outcome/controller-feature intersection

For a compact cross-case audit of retained Mazda rider labels and controller
settings, run:

```powershell
python tools/mazda_ti/feature_intersection_audit.py `
  --root F:/repos/openpilot-mazda-ti `
  --json F:/repos/openpilot-mazda-ti/.ti-local/compensation-study-20260909/feature-intersection-audit-v1.json `
  --md F:/repos/openpilot-mazda-ti/.ti-local/compensation-study-20260909/FEATURE-INTERSECTION-AUDIT-v1.md
```

The output is an evidence index, not an event detector or controller
qualification. It retains source and input hashes, leaves unrecorded settings
as `null`, and keeps rider labels, command/telemetry, and road behavior as
separate evidence levels. Route-level labels must not be read as exact event
labels. Use the feature findings to identify exclusions and confounding (for
example, a setting common to both smooth and problematic modern cases), then
return to source-locked replay and physical evidence before selecting a
controller change. The compact 28f initData extract records the exact source
rlog hash and only fields explicitly read from its first initData event.

### Plan-to-motion phase audit

Use plan_feedback_audit.py to examine the narrow observational question
whether as-of model demand has a stable phase relationship with measured yaw
motion across rider-labelled symptom and smooth-control windows:

~~~powershell
python tools/mazda_ti/plan_feedback_audit.py --request .ti-local/compensation-study-20260909/plan-feedback-audit-request-v1.json --data-root . --output NEW_COMPACT_JSON --markdown NEW_COMPACT_MD
~~~

The request freezes raw paths, monotonic windows and rider labels. The tool
hashes the rlogs and source, rejects stale joins, reports its full +/-1 s
time-domain scan, and only exposes 0.4-3 Hz Welch phases when at least three
windows exist. A maximum at the lag-search edge is explicitly edge_limited.
The model request is an as-of modelV2 publication, not an exact legacy
consumed-input identity; phase, correlation and coherence never establish
plan-to-motion causality. Keep only the compact outputs; rebuild from the
request and raw rlogs when reviewing later.

### Closed-loop command-response audit

Use `closed_loop_id_audit.py` only with a compact, source-locked set of
independently named rider-labelled windows. It tests whether a published-TI
command proxy adds reproducible *one-step* prediction of measured wheel-angle
or fused-yaw increments beyond causal response state and speed, using
leave-case-out scoring and train-only lag selection:

```powershell
python tools/mazda_ti/closed_loop_id_audit.py `
  --input F:/owned-study/closed-loop-id-input.json `
  --output F:/owned-study/closed-loop-id-audit.json `
  --markdown F:/owned-study/CLOSED-LOOP-ID-AUDIT.md
```

Rows may contain only `time_s`, `ti_command_counts`, `wheel_angle_deg`,
`yaw_lateral_accel_mps2`, and `speed_mps`. The audit rejects malformed or
nonmonotonic rows. It excludes controller reference/actual fields, lane/model
values, and future response values from predictors, so it cannot use a
post-treatment controller signal to create a command-to-motion claim. Its
reported gains, lag choices, and outcome-separation gate are predictive
associations, not rack parameters, delivered torque, or proof of a physical
path. A failed gate is an explicit falsifier of this corpus as a basis for a
delay/gain/phase controller change; a passing gate still needs current,
instrumented exact-applied-command evidence and rider-confirmed road behavior.

### Lateral event evidence

`lateral_event_detector.py` scores a pre-joined normalized row stream for
command decline while motion remains in the prior direction. It requires an
explicit monotonic window and rlog paths plus explicit identity, lane-health
and health fields; missing gates are insufficient evidence. It rejects invalid
rows and gaps over 100 ms, keeps camera EOF separate from MONOTONIC, and grades results as
`command-only`, `motion-supported candidate`, or `insufficient evidence`.
Motion-supported requires steering-angle/rate unwind signs and timing to agree
with the command, in addition to lane/health and yaw-rate × speed checks;
missing or disagreeing steering remains command-only when other gates are usable.
Model lane geometry is a validity gate and remains model-derived; yaw-rate ×
speed and lateral acceleration are a separate motion check. The output never
asserts rider contact, surveyed lane truth, causality, or physical success.

### Observed-drive measurement qualification

`observed_drive_rows.py`, `maneuver_metrics.py` and `measurement_qualification.py`
measure settling, lane-motion cycles, inward-hold/late-wide, path quality and a
jerk proxy on already supplied drives with per-group health and curvature phase
anchors, then report false alarms, missed symptoms and unscorable cases against
retained rider labels. Holdout traversals are sealed outcome-blind. See
[OBSERVED_DRIVE_METRICS.md](OBSERVED_DRIVE_METRICS.md). This qualifies the
measurement, not a controller or a physical acceptance threshold.

Use [geographic matching](GEOGRAPHIC_MATCH.md) to locate a previously identified
road window in another recorded drive. It retains direction, coverage and
localization checks; matching a location does not transfer rider annotations.
For a compact first pass over extracted NPZ files, use the coarse inventory
documented there. It derives geometry from a hash-verified reference extract;
its qualification is always `coarse_candidate_only`, and raw float64 geographic
matching is required before naming or using a hit.

[`INGESTION.md`](INGESTION.md) defines the separate versioned inventory and explicit bounded collection command for raw drive evidence. It is offline-testable and does not evaluate a candidate, contact a device, or expose unrestricted Params.

For structured rlog fields, the identity-coverage audit command, sign conventions and logging tests, read [DIAGNOSTICS.md](DIAGNOSTICS.md). Verify the recorded diagnostic versions and vehicle source separately from the analyzer's current checkout.

For a suspected steering-angle measurement error, use the field guide's
[exact consumed-yaw comparison](DIAGNOSTICS.md#check-the-angle-model-measurement-against-consumed-yaw).
It emits compact descriptive evidence with measurement ages and uncertainty,
without replaying commands or predicting a vehicle path.

For instrumented-drive candidate replay, that guide also documents `RecordedTiFeedback`,
the shared library for coupling candidate requests to software TI feedback through exact
consumed-output and applied-command identities. It is separate from the historical runner
below. The [full-controller adapter](INSTRUMENTED_EVALUATION.md) resolves every consumed
input and recorded clock and qualifies both command paths through TI loss and recovery.
Both adapters expose the typed `PreviousAppliedFeedback` boundary before a controller
update. `candidate` is absent until the consumed output represents an apply sourced by a
post-activation candidate update. When present, it carries TI and stock counts, selection,
availability, limiter-freeze state and the known request/controller/apply/publication
identities. Callers must not reconstruct this state by joining a later send or output.

`verify_replay.py` checks that an integrated controller reproduces an already-qualified reference replay, and that the source files still match the integration run's recorded hashes. It runs with Python3.11+ and the standard library on Windows or Linux. It reads local files and performs no network, vehicle or deployment operations.

The shared runner now prepares input identities and runs baseline/candidate replay directly from raw rlogs. It has been checked against the qualified VW window under both compatible request histories and a second window containing an actuator-state transition. Preparation and candidate artifacts reproduced byte-for-byte after relocating the raw inputs. This does not yet qualify every older corpus case or predict improved lane tracking.

Version2 runner records cover relevant repository dependencies (including PID, plant, limiter, schemas, controlsd and card), every shared Python tool, loaded repository modules, Python version/executable, and numpy/pycapnp versions plus Python/native runtime files. Candidate execution requires an exact recorded-command baseline from the same preparation, history and environment, with unchanged source and artifact hashes. These are reproducibility checks, not signed attestations or a hermetic machine image. Legacy integration locks still cover only their listed three runtime files; they cannot qualify changes elsewhere.

## Prepare and replay

For older NNFF recordings, use the separate [historical NNFF/PID probe](LEGACY_NNFF.md).
Then use the [historical actuation audit](LEGACY_NNFF_ACTUATION.md) for original
TI/stock scaling, limiter history and previous-apply feedback checks.
For neural feedback-rate contributions, use [rate-input isolation](LEGACY_NNFF_FEEDBACK.md);
it preserves the full baseline while evaluating instantaneous expression changes.
It covers normalized controller-output reconstruction on pinned source, with
explicit legacy receipt assumptions. The full actuator/candidate adapters below
remain distinct and do not inherit that qualification.

For hidden retained targets in the older plant controller, use the
[legacy feedback-reference audit](LEGACY_REFERENCE.md). It separates current,
delayed and retained targets using source-verified log equations, without
turning that algebraic evidence into a physical handling claim.

Before using fitted road curvature as an independent steering target, run the
[lane-motion consistency check](LANE_MOTION_BALANCE.md). It compares recorded
lane motion with yaw/speed at original model rate and preserves sensitivity to
the10m/20m fits, missing geometry and observation timing.

Run these commands from the repository root. The runner needs numpy and pycapnp compatible with the chosen Python runtime; the verifier itself remains standard-library-only. It loads real controller/PID/filter/plant/limiter code with constant-only facades for hardware imports. It does not open sockets, invoke vehicle hardware or modify the checkout.

Create an experiment JSON with these fields:

| Field | Meaning |
| --- | --- |
| `format_version` | `1` for the experiment specification |
| `name` | Descriptive experiment name |
| `rlogs` | Distinct paths relative to the data root, such as `route--3/rlog`; uncompressed Cap'n Proto events |
| `window` | `anchor_i_at`, `activation`, `start`, `end`, all monotonic seconds; anchor precedes activation, activation is at/before scoring start |
| `baseline_controller`, `warmup_controller`, `limiter` | Explicit Git revisions; preparation resolves them to full commits |
| `candidate_controller` | Explicit revision or `worktree` |
| `fpcs_sample_at` | `carState` or `controlsState`; a declared publication-based sampling assumption |
| `force_offset` | Explicit boolean for the historical live-torque offset policy |
| `settings` | Numeric values for the required recorded parameters below |

Required settings are `TiSteerMax`, `TiSteerDeltaUp`, `TiSteerDeltaDown`, `TiSteerDriverAllowance`, `TiSteerDriverMultiplier`, `TiSteerDeltaUpKnee`, `TiSteerDeltaUpHigh`, `LatDamping`, `LatCommitSetpoint`, `LatFrictionComp`, `LatOutputFilter`, `LatNoFrictionRelay`, `TorqueInterceptorEnabled`, and `SteerKP`. Read these from raw `initData.params.entries`, not an extraction summary that may omit toggles. Only these whitelisted settings enter the report; other recorded Params can contain private information.

[Synthetic controller scenarios](SCENARIOS.md) run fixed controller/limiter inputs locally. Their outcomes cover software command invariants only, not a vehicle path or handling result.

[Resumable corpus execution](BATCH.md) schedules independently isolated common-adapter cases with validated cache reuse. Its timing metadata is host-specific and separate from canonical evidence.

The current runner requires TI600, interceptor enabled, output smoothing off, and the friction relay disabled. It checks the expected initial settings in every supplied segment. That does not prove those values persisted at runtime; baseline reproduction and recorded flags remain necessary, and newly instrumented drives can provide direct runtime settings. Configure enough preceding segments for reference, friction, integral and actuator-state history. An integral anchor is a declared state initialization, not a fitted correction on every frame.

```text
python -m tools.mazda_ti.run prepare --spec experiment.json --data-root PATH/TO/rlogs --output evidence/prepared
python -m tools.mazda_ti.run replay --prepared evidence/prepared --data-root PATH/TO/rlogs --variant baseline --history earliest --output evidence/baseline-earliest
python -m tools.mazda_ti.run replay --prepared evidence/prepared --data-root PATH/TO/rlogs --variant candidate --history earliest --baseline-result evidence/baseline-earliest/result.json --output evidence/candidate-earliest
```

Repeat baseline and candidate with `latest` and distinct output directories. These are two compatible sampling histories, not bounds on all possible histories. Candidate execution uses its own commands and software TI feedback after activation; recorded vehicle motion remains fixed. The schema's newly logged input identities are not yet consumed by this historical-log runner.

The historical runner records exact candidate controller and paired-request identities in
`previous_applied_feedback` once a candidate-generated send has produced the consumed
output. Older recorded requests have no qualified producing-controller identity, so the
typed candidate value remains null during warmup instead of assigning a nearest update.

At activation, separate controller computation, consumed-input time and command publication.
`publish_paired_request` publishes each paired carControl inside the feedback window even
when its controller consumed an older input. A computation before activation retains its
recorded request; one after activation uses the replay request. Feedback reads still use
the audited input cutoff. Gating publication on that cutoff can omit the first request
needed by card. Serialized-message tests cover both sides of this boundary.

If baseline differences cluster immediately after activation, test whether the integral
anchor leaves enough settling time before scoring. Declare the earlier anchor before
rerunning, preserve the failed output, and keep activation, scoring, settings and the
exact-send requirement unchanged. In two older hard-corner cases, moving the anchor from
one to eleven seconds before activation removed the initial one-count differences under
both histories. This result does not justify accepting other one-count failures.

Preparation writes the resolved spec, input audit, identity mapping, request sampling maps and `preparation.json`. Replay writes `trace.json`, `trace.jsonl`, `trace-sends.json` and `result.json`. The baseline requires non-empty coverage, identical integer sends, matching plant flags, and less than one count of controller-output residual. The residual allowance is separate from the exact integer-command requirement. Failed evidence is retained, and candidates are refused when their baseline is unqualified. The isolated historical Float32 rounding exception is not implemented in this shared runner; it must fail rather than silently accept one-count mismatches.

Use a new output directory for every run. Changed source, runtime, prepared artifacts or raw-log bytes require fresh preparation and qualification. Paths are relative in reports, which allows identical bytes at a different data root. Raw logs, generated traces and large result manifests should remain outside Git.

## Compare an integration

```text
python tools/mazda_ti/verify_replay.py --baseline PATH/TO/reference --candidate PATH/TO/integration --source-lock PATH/TO/integration-result.json --start 230 --end 327 --additional-log-flag 4096 --output verification.json
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti/test_verify_replay.py
```

Each prefix identifies a `.jsonl` control trace and a `-sends.json` integer-send trace. Times are route monotonic seconds; send timestamps are integer nanoseconds. `--source-lock` is the result captured by the integration run with a `production_sha256` mapping for the controller, lane helper and controlsd. Preserve that record with the traces. It is not a request to hash today's source and assign it to old results.

The optional test command requires pytest. Its explicit configuration boundary excludes hardware-dependent repository fixtures and plugins; the verifier itself needs no openpilot runtime or third-party packages. The full shared checks are:

```text
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti
```

The verifier requires identical active frame identities, controller/reference/geometry fields, integer sends and their coverage. The optional4096 allowance verifies the newly added lane-release log flag on precisely the reference-removal frames. It cannot waive a torque or reference difference. The TI600-count and15-count adjacent-send bounds are checked for this campaign's configuration.

Reports contain byte hashes and no wall-clock timestamps or absolute machine paths, so the same input bytes produce the same report. Existing output files are preserved. Version2 verification also checks the broader repository source map and the candidate's saved artifact hashes; it does not execute or compare the current runtime environment. Changed source, missing frames, changed commands or non-finite JSON produce a nonzero exit. Source hashes deliberately cover actual file bytes; line-ending conversion may require a fresh integration run rather than reusing an old source lock.

Keep raw drives and large generated traces outside Git. Share the tool, tests and skill; provide evidence artifacts separately when another reviewer needs to reproduce a particular comparison.

When replay changes controller classes at activation, `switch_controller` preserves existing
state and initializes newly introduced fields. For the plant measurement observer, legacy
history is valid only when the immediately preceding replayed update was active. Otherwise
the new observer starts without a derivative and clears the stale rate filter. An already
initialized newer observer retains its history through inactive updates. Tests exercise these
three cases against real controller implementations; initialization must not create or hide a
candidate transient. The optional trace fields `friction_withdrawal` and
`friction_episode_completed` expose the new friction policy during active updates. Inactive
rows lack active-loop fields; absence is not a measured zero. Source locks include the new
friction helper, so older preparations require fresh qualification before candidate execution.
