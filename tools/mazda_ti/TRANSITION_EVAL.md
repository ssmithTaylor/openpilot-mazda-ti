# Mazda isolated control-transition evaluation

`transition_eval` reads retained serialized `controlsd` observations and writes
the common evidence statuses: `completed_checks`, `failed_check`,
`failed_execution`, or `unsupported`. It never starts `card` or `pandad`,
constructs a CAN publisher, contacts a vehicle, or keeps Params/messaging state
after the command exits.

Every normalized row carries `state_before`/`state_after` plus opaque
`state_before_id`/`state_id` tokens. The evaluator compares both forms: the
last active post-state must equal the next re-engagement pre-state, and the
last TI-available post-state must equal the TI re-entry pre-state. A `RESET`
or token mutation therefore produces `failed_check`; an inactive interval is
not treated as evidence that state was retained.

The portable fixture profile is useful for deterministic failure tests. Its
JSON list has a strictly increasing `mono_time_ns` identity, `active`,
`ti_allowed`, `health` (`valid`, `missing`, `stale`, or `invalid`), an exact
diagnostic reference list, and an opaque `state_id`. A completed fixture must
show active operation, disengage/re-engage, TI bypass/re-entry, and each of the
three unhealthy-message classes while inactive. A missing class is a recorded
availability limit and produces `failed_check`; it is never inferred as a
successful transition.

```text
python -m tools.mazda_ti.transition_eval \
  --observations transition-fixture.json --output NEW-EVIDENCE
```

For a supported integration profile, use built Linux openpilot process replay
and retained Mazda rlogs. The final rlog is the transition segment; earlier
ones may supply its recorded `carParams`.

```text
python -m tools.mazda_ti.transition_eval \
  --rlog PATH/TO/PRIOR/rlog --rlog PATH/TO/TRANSITION/rlog \
  --all-segments --max-carstate-messages 300 --output NEW-EVIDENCE
```

That profile invokes the same real `controlsd` process-replay interface as
`startup_eval`, uses its tool-owned Params directory and unique fake messaging
prefix, observes generated `controlsState`/`carControl`/`carOutput` events, and
rejects observed `can` or `sendcan` output. It derives each diagnostic reference
from serialized fields and requires it to point to an input event with the exact
service and monotonic identity. It then checks active/disengage/re-engage and
TI bypass/re-entry from the actual process output. It does not manufacture a
transition from a fixture. `--all-segments` retains each selected adjacent
segment, its input hash, and its exact whitelisted initial settings snapshot;
those settings are copied only into tool-owned Params and must agree across the
selection. At an active TI-bypass boundary it checks the exact
serialized prior `integralAfter` against the bypass frame's `integralBefore`;
the integral may evolve, but a reset cannot be hidden as retained state.
The top-level result copies the process boundary's exact `runtime_source.git_head`
and normalized `input_sha256` rlog map. Release qualification can therefore bind
this process evidence to both the candidate revision and retained input bytes;
the serialized fixture profile does not claim those full-process identities.
Only the module's actual process boundary emits `full_process_transition`.
An injected boundary emits `process_transition_fixture` even when a caller
forces the capability result to supported, and release qualification rejects
that fixture profile.

`--schema-input-harness` is the bounded live-transition profile. It runs
`controlsd` with process replay simulation disabled and paces the actual
subscriber schemas at their declared service frequencies. Retained valid
payloads are republished for the real `controlsd` subscriptions; generated
inputs are limited to `managerState`, `pandaStates`, `frogpilotCarState`,
`frogpilotPlan`, and `liveDelay`, each carries its retained source-frame
identity. The fixture deliberately withholds `frogpilotCarState` during its
cold start (missing), makes a later publication gap while other subscriptions
stay healthy (stale), and sends one explicitly invalid schema event. These are
reported as expected harness exclusions only when the parsed `controlsd`
diagnostic names exactly those declared services; any extra service, stack
trace, malformed stderr, or non-frequency health failure fails the run.
It requires `--rlog` and `--all-segments`; the command rejects a single-segment
invocation rather than silently rereading an unmodified raw segment after it
has declared ownership of those subscriptions.

This profile demonstrates process response to serialized schema inputs. It
does not claim recorded TI availability beyond the fixed retained cutout, live
vehicle behavior, hardware output, or device timing. Its 100 Hz wall-clock
pacing and host workload are execution metadata. The retained unmodified
cold/inactive replay remains negative evidence rather than a substitute for
the active transition profile.

When the host cannot import the supported runtime, the command records
`unsupported`; when a retained rlog does not contain all required active or
fault sequences, it records `failed_check` with the fixed availability counts.
Neither result satisfies the real integration profile. The existing startup
pair is deliberately cold/inactive evidence and therefore cannot establish
this transition profile by itself.

`execution.json` holds `elapsed_seconds`, the absolute output destination, the
unique messaging prefix, exception details, and captured process output outside
canonical `result.json`. The elapsed value
excludes workload outside this replay and does not establish device scheduling
or latency. A release profile that needs device timing must record that
separately on the target device. The report hashes the process/schema and
transition reader sources, preserves generated diagnostics/state identifiers,
and records no raw rlog bytes in Git.
