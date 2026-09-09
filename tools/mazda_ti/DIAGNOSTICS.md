# Mazda rlog diagnostics

The working-tree instrumentation adds structured records to existing `controlsState`, `carControl` and `carOutput` events. It adds no service or stdout stream. Schema changes require a complete device build and parameter-sync check before reboot. A local test pass does not establish device timing or road performance.

## Follow one command

1. Read `carOutput.appliedCarControlMonoTime` to identify the `carControl` event represented by that output.
2. Read that command's `controlsStateMonoTime` to identify its `controlsState` event.
3. Read `controlsState.lateralControlState.torqueState.mazdaDiagnostics.inputs` to identify the messages used for that controller update. Seven rows come from SubMaster and record event validity, liveness, frequency status, updated/seen status, and `all_checks` after configured ignore rules. The carState row comes from its dedicated socket, as described below.
4. Compare controller references and command contributions with `carOutput.mazdaDiagnostics`, then use the recorded CAN and vehicle/lane observations to assess what happened physically.

`card` publishes its previous applied output before its next apply. The identity fields deliberately follow that previous output. `applySequence` increments only after `CI.apply` returns; repeated sequence numbers describe the same apply. `appliedAtMonoTime` is the timestamp passed to `CI.apply`, not a hardware receipt timestamp. The existing event `valid` refers to current publication health; `appliedCarControlChecksPassed` preserves health at the represented apply. Neither should be substituted for the other.

At a segment boundary, referenced inputs or commands can be in the preceding segment. Load adjacent segments before declaring a reference missing. Do not substitute nearest timestamps when an explicit identity is absent. Old logs have zero/default identity fields and need the older, explicitly qualified sampling reconstruction.

`controlsd` receives carState through a separate blocking socket; it is absent from its SubMaster. Its snapshot preserves the exact last received event identity and validity when `CS_prev` is reused on timeout. `seen` means an event has arrived; `updated` and `alive` mean an event arrived on this iteration. `frequencyOk` is always false because this socket has no SubMaster frequency estimate; it does not indicate measured low frequency. `checksPassed` means a new event arrived with event validity and `carState.canValid` both true. These are diagnostic definitions, not additional control gates. Before first receipt, identity is zero and `seen` is false.

The original instrumentation in `1f56328` incorrectly looked up carState in SubMaster and crashed controlsd before publishing the first controller diagnostic. The regression test now obtains the real subscriber list from controlsd's constructor and exercises its real receive/fallback method; an invented all-services fixture cannot qualify this boundary.

## Controller record

### Check the angle-model measurement against consumed yaw

Before attributing a release delay to inflated steering-angle acceleration, use:

```text
python -m tools.mazda_ti.audit_measurement --data-root PATH/TO/rlogs --rlogs ROUTE--SEG/rlog --start-ns START --end-ns END --output NEW-measurement.json
```

This joins the original active torque diagnostics to exactly consumed carState and
liveLocationKalman identities. It compares gross right-positive `measurement` to
`angularVelocityCalibrated.value[2] * carState.vEgo`. Do not subtract roll or learned
offset on just one side. Analyze stable and unwind windows separately, with windows
declared before inspecting disagreement. A passing command means all supplied
active measurements have valid identities, finite values and healthy inputs; it
does not qualify a controller or establish physical handling.

The compact report retains min/median/max, both publication ages, mixed-time
separation, distinct localizer-message count, issue counts, bounded samples and
source/raw/runtime hashes. `--sample-count` defaults to12 and accepts0–100; no full
trace or cache is written. Metrics weight controller publications, including
repeated localizer inputs. Inactive defaults and unhealthy measurements are excluded
from statistics. Missing exact inputs are failures, never replaced with nearest
messages. Maximum publication gap and actual first/last publication remain visible;
coverage does not certify missing time or sensor exposure timing.
Supply one original route: the CLI checks distinct paths, not common route naming.
Health failures have aggregate counts; bounded unhealthy samples indicate rejection,
but the individual flag at every rejected timestamp requires re-reading raw inputs.

Yaw times speed approximates rotational acceleration; lateral velocity transients,
mounting, speed and timing can contribute to disagreement. The reported yaw-only
standard deviation times speed is not total uncertainty or a physical error bound.
The fused localizer offers a route beyond steering angle, not independent ground
truth. Do not fit lag to improve agreement or divide uncertainty by sqrt(sample count).
Preserve this distinction when interpreting a small difference or selecting a new
measurement source. Serialized tests cover wrong identities, malformed/invalid data,
unhealthy inputs, inactive defaults, repeated localizer samples and compact output.

### Recorded fields

When testing a short lane-motion forecast as a possible release signal, freeze
horizons and scoring windows first. Predict from currently consumed geometry and
motion only; keep later observations exclusively in scoring. Camera-EOF horizons
leave only `horizon - cameraAge` lead at issue. Report this mixed-time assumption
and reject nonpositive lead. Transport3D points with an explicitly compatible
coordinate frame, rejecting missing or nonmonotonic support rather than extrapolating.
Compare persistence and simple heading prediction on exactly the same available
rows. Future lane estimates share the visual system's errors and are not surveyed
boundaries. Preserve unavailable rows, active-episode boundaries and scoring gaps.
Require improvement before the relevant permission decision; aggregate forecast
accuracy alone cannot justify an earlier release or revive retired raw-plan FF.
If full rows are pruned, retained aggregates/hashes require row reconstruction
before independent recomputation. A forecast is not a candidate-driven vehicle replay.

`mazdaDiagnostics.version == 1` identifies this format. Version zero means absent, including non-plant controllers and old recordings. If the surrounding torque state is inactive, only request/filter/context/settings fields are populated; zero active-loop fields do not represent measured zero torque or acceleration.

The reference sequence is:

`modelV2.action.desiredCurvature` → curvature limiting in controlsd → `rawRequest` → `filteredRequest` → delayed and committed references → `effectiveFeedforward` / `effectiveSetpoint`.

`rawRequest` is the curvature passed into the controller multiplied by speed squared; it is **not** the network's untouched output. `committedFeedforward` and `committedSetpoint` include the commitment blend and precede lane-supported removal. Separate ratchet states, gate state, blend, removal amounts, dwell counters, lane validity/reason, and motion/position permissions explain the reference transition. Lane geometry is the actual context used, including its camera age. The newest model data are available through the explicit input identity.

Acceleration fields use m/s², measurement rate uses m/s³, lane offset uses metres, headings use radians, and curvature uses 1/metres. Positive means right in this record. Command contributions use TI counts in that same right-positive controller frame. The final `command / tiMax` is negated before publication as `actuators.steer`.

`inverseCommand`, proactive `frictionCompensation`, `frictionRelay`, `breakerBoost`, `commandBeforeSmoothing`, and the output filter/blend describe successive command stages. Each stage retains its existing clipping. Summing every contribution without those clips is incorrect. The historical `frictionTorque` field combines relay and breaker, normalized by TI maximum; it does not include proactive friction compensation.

### Friction release state extension

Read `frictionReleaseVersion` independently of the parent version. Zero means this state
was not recorded, even when `mazdaDiagnostics.version == 1`; it cannot establish that no
withdrawal or completed episode existed. Version1 records the helper's post-update state:

| Field | Meaning |
| --- | --- |
| `frictionWithdrawal` | Nonnegative TI counts removed from the existing proactive compensation |
| `frictionReleaseCompleted` | The helper observed unwind after withdrawal began and latched completion |
| `frictionReleaseDirection` | Right-positive direction: -1 left, +1 right, 0 after reset |

The signed contribution that existed before withdrawal is `frictionCompensation +
frictionReleaseDirection * frictionWithdrawal`. This is a command decomposition, not
delivered motor torque. A completed episode prevents repeated withdrawal until a fresh
tightening request rearms it; it does not identify a road corner or successful recovery.
Direction reversal, inactive updates and disabled/ineligible compensation reset the helper.
On inactive plant updates, extension version1 explicitly represents reset state: zero
withdrawal/direction and false completion. Other inactive active-loop fields retain their
unavailable meaning. Completion may remain true after withdrawal has returned to zero.

`test_friction_diagnostics.py` serializes actual controller results in both directions,
compares existing fields and outputs against the pinned pre-extension implementation,
and exercises completion, rearming and nonzero-to-reset transitions. Original drive28f
logs have extension version0. Local replay qualification covers22,700 updates/applies,
including2,022 inactive updates: only the four extension fields differ. This is not a
device-build or physical-handling qualification; verify new recorded fields after deployment.

Check the state extension in a new recording with:

```text
python -m tools.mazda_ti.audit_friction_release --data-root PATH/TO/rlogs --rlogs ROUTE--SEG/rlog --start-ns START --end-ns END --output NEW-friction-state.json
```

The auditor reconstructs pre-withdrawal compensation from the logged inverse, friction gate,
authority and settings. It checks decomposition, bounds and reset state, then replays the
actual helper from each preceding observed state using the exact consumed carState steering
rate and logged lane permissions/dwell. The first complete state is an anchor; the check
does not reconstruct history before it. Publications more than30ms apart fail explicitly.
There is no controller sequence counter, so even a passing check cannot prove no smaller
missing interval occurred. Use the separate input/apply and lane-context audits as well.

Exit zero requires version1, consistent state/contribution checks, active coverage and at
least one checked transition. Absent/unsupported extension, missing or mismatched inputs,
nonfinite values, altered compensation or latch, reset errors and publication gaps fail.
The1e-9 tolerance applies only to these Float64 count/state comparisons; it cannot excuse
an integer-command mismatch. Source/runtime/raw hashes and all failed rows are retained.

Qualification includes twelve serialized production-controller/failure tests and22,697
adjacent transitions across22,700 generated replay-fixture updates, with their recorded
carState identities preserved. These generated fixtures are not new vehicle recordings.
The CLI correctly rejects the original28f version0 records. Qualify actual state coverage
again when a new drive is available; a state-consistency pass is not a lane-performance score.

`integralBefore`, `integralAfter`, the PID input error/feedforward, gains and output permit inspection of accumulated error and anti-windup. `freezeReasons` bits are: 0 limiter feedback, 1 steeringPressed, 2 speed below 5 m/s. The steeringPressed input can be TI-contaminated and is not evidence of hand contact. `plantLimit` is the controller model's current limit, not a measured tire-grip ceiling.

`settings` bits are: 0 commitment, 1 damping, 2 proactive friction compensation, 3 output smoothing, 4 friction relay disabled. These report the toggles actually seen on that update. Float64 values preserve controller precision where Float32 rounding can change an eventual integer command.

`modelContextNow` uses CLOCK_MONOTONIC, matching Python event identities. `cameraContextNow` uses CLOCK_BOOTTIME, matching camerad EOF timestamps. Freshness compares each timestamp within its own clock domain; suspend can separate those clocks. Historical replay without explicit clock samples assumes they were aligned. A numeric lane-fit failure is recorded as `fit_failure` and withdraws geometry permission.

## Actuator record

`carOutput.mazdaDiagnostics.version == 1` identifies a TI-equipped Mazda apply. It records stock and TI requests, limited integer commands, previous commands, the limiter's torque input, TI permission and effective TI limiter settings. A TI request of zero while `tiAllowed` is false is the gated request, not evidence that the controller asked for zero.

Here the counts use the wire convention, **positive left**. `stockLimited` and `tiLimited` are arguments to the CAN packer, not measured EPS/motor delivery. In particular, stock request counts must not be interpreted as the stock EPS's measured ±308-count response. Existing received CAN/FrogPilot state and video remain necessary for the vehicle response.

The records distinguish a late model request, retained reference, persistent integral/friction state, and an actuator-limited request. They do not by themselves prove that a different request would improve the path. Match speed, entry position and rider-confirmed motion when judging handling.

## Local validation

For candidate controller replay with these explicit identities, `recorded_feedback.py`
provides the software TI feedback boundary. It is a library; the historical `run replay`
CLI still uses its declared sampling histories. After auditing the input chain, construct
`RecordedTiFeedback(outputs, commands, activation_ns, end_ns, limiter)` with serialized
carOutput events paired with event timestamps, a carControl event map keyed by timestamp,
and the pinned production limiter. Times are integer nanoseconds.

Before each controller update, use `output_for(consumed_carOutput_mono)` for the exact
output identity in its diagnostic inputs. Publish the resulting normalized steering with
`publish(controlsState_mono, steer)`. Use `history_request(mono, replayed, recorded)` for
the controller's request-history comparison, preserving recorded requests and feedback
together before activation. Call `finish()` after all required controller outputs are
published to obtain sequential apply results. Retain the controller's actual limiter-feedback
update order; the helper does not implement controlsd's surrounding state machine.

Construction verifies original serialized requests, sequential TI limits, previous-command
continuity, repeated-apply payloads and feedback values. Simulation retains its own previous
limited command. A post-activation publication can still represent a pre-activation apply;
the represented apply determines which feedback is used. Missing identities, changed repeated
applies, nonfinite signals and unsupported settings fail explicitly. Active stock fallback
requires the explicit stock configuration below; the default TI-only mode still rejects it.
Recorded physical observations, driver torque, actuator permission
and the apply schedule remain fixed; software feedback is not a vehicle-motion prediction.

The helper reproduced the local exact-identity prototype's controller traces byte-for-byte
and its TI results on three original-drive windows with baseline and four individual probes
(76,000 scored rows total). Fourteen serialized-event tests cover activation boundaries,
candidate-owned feedback, delayed consumed identities, and malformed/incomplete inputs.
This qualifies the feedback component, not an arbitrary new controller or full replay runner.
Preserve caller, dependency, runtime and raw-log hashes and independently reproduce the
recorded baseline before interpreting a candidate.

### TI bypass and stock feedback

GEN1 continues calculating the stock command while the TI operates. When TI permission is
lost, `carOutput.actuatorsOutput.steer` selects the stock limited request divided by **600**.
The EPS's measured ±308 response is a different quantity. Replacing unavailable TI feedback
with zero loses the stock limiter history and can change the controller's anti-windup behavior.

Use `pure_stock_limiter(pinned_revision)` from `feedback.py` to load the actual stock limiter,
GEN1 constants and their source hashes without hardware imports. Pass its first two results
as `stock_limiter=` and `stock_limits=` to `RecordedTiFeedback`. Record the returned source
hashes with the caller's provenance. Only the explicit GEN1 stock600 settings are supported.

This mode validates both recorded request/previous/limited sequences on every apply, including
when the other actuator supplies the published feedback. Candidate requests advance both
histories; `output_for` selects the appropriate normalized command using recorded TI permission.
`counts` and `finish()` retain their existing TI-count meaning; `stock_counts` contains the
separate stock counts keyed by apply sequence. Neither is motor-delivery measurement.

Seven serialized-event tests cover the distinct rate limits, continuously retained stock
history, TI reset/re-entry, inactive reset, old consumed identities, corrupt stock evidence
and rejection of EPS308 normalization. A 7,000-update original-drive baseline covering a TI
cutout, active stock fallback and recovery reproduces all TI and stock commands. Recorded
availability still forces the same TI dropout in candidate replay; this cannot predict
whether changed commands would avoid it or how the subsequent vehicle trajectory would differ.

After checking applied-output identities, reproduce the lane observer from the actual consumed
model events and both recorded clocks:

```text
python -m tools.mazda_ti.audit_lane_context --data-root PATH/TO/rlogs --rlogs ROUTE--13/rlog --start-ns 835000000000 --end-ns 855000000000 --output NEW-lane-context.json
```

This executes the current production `LaneObserver` and compares its reason, validity, age,
offset, headings and curvatures with logged diagnostics. The absolute comparison tolerance
is 1e-10 for those geometry/age fields only. Reports include the exact consumed model identity,
lane probabilities, separate input health, reference signals and source/raw/runtime hashes.
Invalid geometry produces a null offset in the report, rather than a measured lane-center zero.
Missing consumed events, invalid clocks, duplicate identities and changed recorded results
fail the audit. Existing evidence is preserved. Input file order does not select a newer model
in place of the consumed one; age is rechecked even when a model identity repeats.

The auditor is qualified on 15,200 recorded controller publications spanning the drive28f
approach and both VW directions, plus serialized clock/identity/failure cases. This establishes
lane-observer reproduction only. A correct fit of low-confidence or incorrectly selected lane
boundaries is not proof of a correct physical path. Controller command reproduction, release
state history and rider-confirmed lane outcomes remain separate requirements.

Audit explicit identities in original uncompressed rlogs before interpreting a new drive:

```text
python -m tools.mazda_ti.audit_diagnostics --data-root PATH/TO/rlogs --rlogs ROUTE--3/rlog ROUTE--4/rlog --start-ns 262000000000 --end-ns 285000000000 --output NEW-coverage.json
```

Use integer monotonic nanoseconds and a half-open scoring window. Supply one route and continuous card lifetime, with adjacent segments containing preceding inputs. The auditor joins each scored carOutput publication to its actual carControl, controlsState and eight input identities. It preserves repeated prior-apply publications and reports sequence gaps/resets, changed repeated payloads, missing events, version-zero/unsupported diagnostics, future references and validity mismatches. Unseen inputs are incomplete coverage. Invalid/liveness/frequency health remains distinct from identity availability; inspect the health counts even when all links resolve.

Exit zero requires complete identities for the scored publications and at least one active controller record. Inactive-only coverage, empty windows and absent instrumentation exit nonzero. JSON includes exact identities, issues, source/runtime/input hashes and limitations. Relocating identical input bytes under the same relative names produces identical reports. Existing output is preserved. Duplicate identities or malformed inputs abort rather than selecting an arbitrary message.

This check does not reconstruct commands or judge lane motion. It cannot certify unrecorded leading/trailing time, controller publications never applied by card, CAN reception, or compatibility with process-replay timestamp rewriting. Its CLI and failure cases are exercised with serialized production-schema fixtures; original pre-instrumentation VW logs correctly fail coverage. Qualification on a new active drive remains required.

```text
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti/test_diagnostics.py tools/mazda_ti/test_audit_diagnostics.py tools/mazda_ti/test_audit_lane_context.py tools/mazda_ti/test_verify_replay.py
```

The diagnostic tests require numpy, pycapnp and pytest. They serialize real schemas, exercise the actual controller and targeted controlsd/card/Mazda steering source blocks, and replace hardware/transport edges. They cover both turn directions, command decomposition, runtime settings, TI gating, prior-apply alignment, skipped/failed applies, and stale/invalid input snapshots. They are not full process or device tests.

Preserve source hashes with corpus comparisons. Include the controller, lane helper, diagnostic helper, PID, plant, carcontroller, card, controlsd and both schemas. Logging-only changes still require fresh equivalence evidence. Stock process-replay tooling rewrites event timestamps; its nested identity handling must be qualified before using these links on generated process-replay logs. Original rlogs retain the actual identities.

Before a drive, verify the remote-backed build, new schema parsing, runtime cost and identity-chain coverage on the device. Do not claim future road logs contain these fields until the running build and actual recorded events confirm it.

For the synthetic controller workload after a build, run:

```text
python -m tools.mazda_ti.benchmark_controller --output NEW-controller-benchmark.json
```

This runs2,400 updates with repeated easing, completion, rearming and inactive phases,
fitting lane geometry every fifth update and serializing the resulting controlsState.
It uses controlsd's actual subscription list and a separate carState event. Serialized
fields are checked against the controller's state outside the timed region. The report
records source/runtime identity, exercised states, event size and timing percentiles;
exit zero requires every state to be exercised and p99 below10ms. Timing results vary
with machine load and are not deterministic artifacts. This controlled workload does
not measure full process transport, concurrent onroad load or actual lane performance.
Keep its result with the build record, then verify real onroad logs separately.

The [instrumented evaluation adapter](INSTRUMENTED_EVALUATION.md) now promotes the local full-controller caller
behind the shared evaluation command. It requires complete exact identities, checks each applied
carControl against its linked controller publication, and qualifies both limiter histories before
evaluating a candidate. The standalone identity auditor remains an identity-coverage check.

## Compare legacy recordings across lateral controllers

Use this procedure when an older rider-confirmed comparison lacks the structured
input identities required by the auditors above. A zero diagnostic version or
pre-instrumentation controller flag is unavailable evidence, not a measured zero.

The current `controller_replay.py` and `instrumented.py` adapters instantiate
`LatControlTorque`. They do not reproduce a historical `LatControlNNFF` baseline.
Loading its original neural model asset is only a feasibility check: qualifying
that baseline also requires the original controller/PID/interface, plan and roll
histories, parameter sampling and command/limiter reproduction. A mismatch from
the torque adapter on an NNFF recording is not a handling verdict.

The separate [legacy NNFF/PID probe](LEGACY_NNFF.md) now reproduces normalized
output on three declared route261 windows. It includes source-derived receipt
sequence checks and shared requests; it does not add NNFF to the full actuator
or release adapters. Use its four timing choices and retain failed holdouts.

For legacy NNFF source `2a098cdb`, distinguish raw `liveDelay.lateralDelay`
passed to `update_live_delay` from the raw delay plus `LAT_SMOOTH_SECONDS`
passed to the controller update. Future neural samples use the former; the
desired-jerk calculation uses the latter. Pitch and demand/roll histories update
only on active full-neural/model-good frames. The old base `reset()` only clears
saturation time, and NNFF has no reset override; never assume inactive intervals
reset its PID or histories. A feedforward-only reconstruction can use an explicit
capture facade, but must label PID, final commands and vehicle response unqualified.

For an original-source NNFF/PID reconstruction, warm the neural pitch/deques
before applying one declared post-update recorded integral anchor. Anchoring I
while neural history is still empty can alter anti-windup decisions and leave a
persistent integral error after feedforward settles. Preserve the failed early
anchor result; do not repeatedly reset I to the recording. In `2a098cdb`,
`publish_logs` computes steering-limited from the current requested steer and
received carOutput after the controller update, for use on the next cycle.
Keep that order and retain separate carOutput receipt hypotheses. Saturation
alert reproduction and applied CAN qualification are separate from PID output.

[`legacy_input_constraints.py`](legacy_input_constraints.py) provides finite
Float32 rounding intervals and `curvature_compatible` for source-verified legacy
angle-model equations. Compare recomputed actual curvature and acceleration
after serialization; check overlap between recorded desired curvature and
desired acceleration rounding intervals at the candidate speed. Do not require
the product of the already-rounded curvature to equal the recorded acceleration.
These necessary conditions reject incompatible candidates without fitting P/I/F
or output. They do not distinguish equal-valued publications, prove receipt,
or remove numerical-runtime uncertainty. Preserve all remaining ambiguity and
score declared timing choices separately. Focused checks:

```text
python -m pytest tools/mazda_ti/tests/test_legacy_input_constraints.py --confcutdir=tools/mazda_ti/tests -o addopts= -p no:cacheprovider --basetemp ABSOLUTE-OWNED-SCRATCH -q
```

1. Bind the rider's symptom and lane annotations to original route/segment/time
   identities. Keep later clarifications as separate records rather than rewriting
   the original annotation. Geographic matching does not establish lane identity
   or transfer an outcome. Within-lane entry position remains a separate variable.
2. Record startup settings, source cleanliness, speed and actuator availability.
   A startup NNFF toggle supports configuration; verify source selection gates
   before declaring effective mode. Do not interpret a controller switch as only
   a change to one feedforward term.
3. Compare original wheel, speed, yaw, model geometry and CAN publications with
   their own timestamps, signs and availability. If exact consumed identities are
   absent, label an as-of publication join explicitly and retain its ages. Never
   fill missing history with future observations or silently score unavailable rows.
4. For cross-drive wheel-angle comparisons, retain both raw angle and each drive's
   learned center. Right-positive adjusted angle is
   `-carState.steeringAngleDeg + liveParameters.angleOffsetDeg`. This is estimator
   normalization, not independent steering-zero calibration. Preserve localizer
   health, mixed ages and reported yaw-only uncertainty; do not divide correlated
   uncertainty by the square root of the sample count.
5. Inspect the recorded source's logging assignments before interpreting internal
   targets or PID terms. For example, plant-controller revision
   `2c50a68456d6f8eb18c5688a72a2c055a2572182` logs `desiredLateralAccel = setpoint`
   while feedback uses the committed `tracked_setpoint`. Logged desired below
   actual can coexist with positive P without proving a sign error. NNFF fields
   require their own units and computation check.
6. Validate a proposed symptom metric against rider-confirmed positive and negative
   examples before using it to select changes. Wheel reversal count alone failed
   this check in the September 2026 VW comparison: a smooth and a symptomatic
   pass both had three reversals under the same existing definition. Preserve
   magnitude, timing and lane context instead of relabeling the rider's outcome.

Completion here means a reproducible descriptive comparison with explicit limits.
It does not provide a candidate-responsive physical test. Keep compact requests,
annotations, hashes, findings and reproduction instructions; lossless compression
can retain full descriptive results while removing verified generated duplicates.
Do not promote a local exploratory helper as a qualified shared runner without
portable inputs and the corresponding command-level acceptance evidence.
