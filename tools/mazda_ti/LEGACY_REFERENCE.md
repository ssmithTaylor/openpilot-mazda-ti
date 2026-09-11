# Legacy feedback-reference diagnosis

Use `audit_legacy_reference` to distinguish the current curvature request,
filtered/delayed setpoint, and retained feedback target in the clean route28c
plant controller. This is algebraic interpretation of recorded fields, not
independent controller replay, an actuator prediction or a handling test.

Supported recorded source is `2c50a68456d6f8eb18c5688a72a2c055a2572182`,
MAZDA_CX5 with steering-angle feedback, NNFF/NNFFLite disabled and active
version2 plant logs. Reject unknown or dirty source. Do not use the inversion
on NNFF's neural torque-domain error or silently extend support to another
revision whose field semantics may differ.

Read [storage policy](STORAGE.md), preflight owned scratch, then run:

```text
python -m tools.mazda_ti.audit_legacy_reference --request tools/mazda_ti/cases/legacy_reference_28c.json --data-root ABSOLUTE-RAW-ROOT --output NEW-ABSOLUTE-SCRATCH/result.json
```

The raw root must directly contain the two route/segment directories. The CLI
checks their hashes, recorded revision/settings and gain; extracts constants
from pinned Git blobs; and retains source, schema and runtime hashes. Repeat
in a fresh process and compare full bytes before interpreting a new result.
Exit0 means the audit completed, not that every row is healthy or available.
Inspect missing rows, controller validity, exact selected-model health/age and
as-of parameter-publication health. No parameter receipt identity is inferred.

The original equations are:

```text
actual_acceleration = actual_curvature * speed**2
error = (tracked_target - actual_acceleration) *
        (1 + (low_speed_value/max(speed, MIN_SPEED))**2 / torque_params.kp)
```

Both acceleration and curvature are independently serialized Float32 fields.
Their rounding intervals enclose speed squared without choosing a nearby
carState publication. Monotonic low-speed correction bounds then enclose the
tracked target. The CLI uses recorded `torque_params.kp` (0.800000011920929
on28c), not an assumed1 or an independently overridden PID gain. A curvature
interval spanning zero leaves speed unresolved and is reported unavailable.

`desiredLateralAccel` is the delayed setpoint in this source. It is not the
tracked target. Current request is logged `desiredCurvature * speed**2`, after
upstream curvature limiting. `history_minus_current_mps2` includes both filtering
and delay; it is not a pure transport delay. `commitment_minus_delayed_mps2`
isolates the retained-target displacement. All values are right-positive;
reverse direction when discussing inward retention on the preceding left turn.
These intervals cover serialization only, not sensor, lane or receipt uncertainty.

Native P/I/D/F are acceleration-domain terms. Their amplitudes cannot be compared
directly with NNFF's torque-domain terms. `frictionTorque` contains relay/breaker
contributions but excludes proactive friction compensation. A zero value does
not mean all compensation is absent. Model lanes locate the camera/model origin,
not vehicle-body clearance or independently surveyed lane position.

## Recorded result and its limits

The declared28c windows contain4400 active resolved rows. Around the first VW
unwind, the current request falls below2m/s² at270.476678489s, delayed setpoint
at270.995883072s, and tracked target at271.576210571s. These are retrospective
descriptive crossings, not predeclared performance thresholds. The latter two
add about0.519s and0.580s respectively at this threshold.

All controller and selected-model publications have valid flags, but200 rows
have unavailable lane geometry. Keep those rows for reference accounting;
do not interpret their default lane values as measurements. Model publication
age ranges5.12–64.58ms; this is not the camera age.

At approximately271s, current request is1.7443m/s², delayed target1.9984,
tracked target2.2873 and measured acceleration2.3515. The retained displacement
relative to current request is0.5430m/s². P has turned negative, but I remains
positive; P+I first turns negative at271.346381145s after current request falls
below measured acceleration at269.876259947s. This establishes term timing,
not that all inward torque should have stopped or that earlier release is safe.

The preceding left turn also has inward reference persistence: maximum tracked
minus current in the inward direction is0.33784m/s² at247.985085998s; maximum
commitment-only displacement is0.12907m/s² at247.466994540s. Thus the observed
mechanism is not restricted to the one right turn. Its contribution to the
rider's physical complaint still needs independent comparison and intervention.

Tests cover interval enclosure for both turn directions, unresolved speed,
invalid tables, unknown/dirty source and NNFF-mode rejection:

```text
python -m pytest tools/mazda_ti/tests/test_legacy_reference.py tools/mazda_ti/tests/test_audit_legacy_reference.py --confcutdir=tools/mazda_ti/tests -o addopts= -p no:cacheprovider --basetemp NEW-ABSOLUTE-TEST-SCRATCH -q
```

Keep full compressed evidence when small enough, or mark pruned summaries as
requiring reconstruction. Preserve failed comparisons. A close-speed smooth24d
NNFF compatibility attempt stops at input health:111 invalid liveParameters
publications over1030–1119.6s and dirty source prohibit promoting it to a clean
original-controller baseline. The rider-confirmed smooth outcome is unaffected.
The NNFF probe's generic health exception does not identify the selected bad
service/time; improving that diagnostic remains an infrastructure opportunity.
