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

`mazdaDiagnostics.version == 1` identifies this format. Version zero means absent, including non-plant controllers and old recordings. If the surrounding torque state is inactive, only request/filter/context/settings fields are populated; zero active-loop fields do not represent measured zero torque or acceleration.

The reference sequence is:

`modelV2.action.desiredCurvature` → curvature limiting in controlsd → `rawRequest` → `filteredRequest` → delayed and committed references → `effectiveFeedforward` / `effectiveSetpoint`.

`rawRequest` is the curvature passed into the controller multiplied by speed squared; it is **not** the network's untouched output. `committedFeedforward` and `committedSetpoint` include the commitment blend and precede lane-supported removal. Separate ratchet states, gate state, blend, removal amounts, dwell counters, lane validity/reason, and motion/position permissions explain the reference transition. Lane geometry is the actual context used, including its camera age. The newest model data are available through the explicit input identity.

Acceleration fields use m/s², measurement rate uses m/s³, lane offset uses metres, headings use radians, and curvature uses 1/metres. Positive means right in this record. Command contributions use TI counts in that same right-positive controller frame. The final `command / tiMax` is negated before publication as `actuators.steer`.

`inverseCommand`, proactive `frictionCompensation`, `frictionRelay`, `breakerBoost`, `commandBeforeSmoothing`, and the output filter/blend describe successive command stages. Each stage retains its existing clipping. Summing every contribution without those clips is incorrect. The historical `frictionTorque` field combines relay and breaker, normalized by TI maximum; it does not include proactive friction compensation.

`integralBefore`, `integralAfter`, the PID input error/feedforward, gains and output permit inspection of accumulated error and anti-windup. `freezeReasons` bits are: 0 limiter feedback, 1 steeringPressed, 2 speed below 5 m/s. The steeringPressed input can be TI-contaminated and is not evidence of hand contact. `plantLimit` is the controller model's current limit, not a measured tire-grip ceiling.

`settings` bits are: 0 commitment, 1 damping, 2 proactive friction compensation, 3 output smoothing, 4 friction relay disabled. These report the toggles actually seen on that update. Float64 values preserve controller precision where Float32 rounding can change an eventual integer command.

`modelContextNow` uses CLOCK_MONOTONIC, matching Python event identities. `cameraContextNow` uses CLOCK_BOOTTIME, matching camerad EOF timestamps. Freshness compares each timestamp within its own clock domain; suspend can separate those clocks. Historical replay without explicit clock samples assumes they were aligned. A numeric lane-fit failure is recorded as `fit_failure` and withdraws geometry permission.

## Actuator record

`carOutput.mazdaDiagnostics.version == 1` identifies a TI-equipped Mazda apply. It records stock and TI requests, limited integer commands, previous commands, the limiter's torque input, TI permission and effective TI limiter settings. A TI request of zero while `tiAllowed` is false is the gated request, not evidence that the controller asked for zero.

Here the counts use the wire convention, **positive left**. `stockLimited` and `tiLimited` are arguments to the CAN packer, not measured EPS/motor delivery. In particular, stock request counts must not be interpreted as the stock EPS's measured ±308-count response. Existing received CAN/FrogPilot state and video remain necessary for the vehicle response.

The records distinguish a late model request, retained reference, persistent integral/friction state, and an actuator-limited request. They do not by themselves prove that a different request would improve the path. Match speed, entry position and rider-confirmed motion when judging handling.

## Local validation

Audit explicit identities in original uncompressed rlogs before interpreting a new drive:

```text
python -m tools.mazda_ti.audit_diagnostics --data-root PATH/TO/rlogs --rlogs ROUTE--3/rlog ROUTE--4/rlog --start-ns 262000000000 --end-ns 285000000000 --output NEW-coverage.json
```

Use integer monotonic nanoseconds and a half-open scoring window. Supply one route and continuous card lifetime, with adjacent segments containing preceding inputs. The auditor joins each scored carOutput publication to its actual carControl, controlsState and eight input identities. It preserves repeated prior-apply publications and reports sequence gaps/resets, changed repeated payloads, missing events, version-zero/unsupported diagnostics, future references and validity mismatches. Unseen inputs are incomplete coverage. Invalid/liveness/frequency health remains distinct from identity availability; inspect the health counts even when all links resolve.

Exit zero requires complete identities for the scored publications and at least one active controller record. Inactive-only coverage, empty windows and absent instrumentation exit nonzero. JSON includes exact identities, issues, source/runtime/input hashes and limitations. Relocating identical input bytes under the same relative names produces identical reports. Existing output is preserved. Duplicate identities or malformed inputs abort rather than selecting an arbitrary message.

This check does not reconstruct commands or judge lane motion. It cannot certify unrecorded leading/trailing time, controller publications never applied by card, CAN reception, or compatibility with process-replay timestamp rewriting. Its CLI and failure cases are exercised with serialized production-schema fixtures; original pre-instrumentation VW logs correctly fail coverage. Qualification on a new active drive remains required.

```text
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti/test_diagnostics.py tools/mazda_ti/test_audit_diagnostics.py tools/mazda_ti/test_verify_replay.py
```

The diagnostic tests require numpy, pycapnp and pytest. They serialize real schemas, exercise the actual controller and targeted controlsd/card/Mazda steering source blocks, and replace hardware/transport edges. They cover both turn directions, command decomposition, runtime settings, TI gating, prior-apply alignment, skipped/failed applies, and stale/invalid input snapshots. They are not full process or device tests.

Preserve source hashes with corpus comparisons. Include the controller, lane helper, diagnostic helper, PID, plant, carcontroller, card, controlsd and both schemas. Logging-only changes still require fresh equivalence evidence. Stock process-replay tooling rewrites event timestamps; its nested identity handling must be qualified before using these links on generated process-replay logs. Original rlogs retain the actual identities.

Before a drive, verify the remote-backed build, new schema parsing, runtime cost and identity-chain coverage on the device. Do not claim future road logs contain these fields until the running build and actual recorded events confirm it.
