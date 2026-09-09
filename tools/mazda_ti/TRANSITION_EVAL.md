# Mazda isolated control-transition evaluation

`transition_eval` reads retained serialized `controlsd` observations and writes
the common evidence statuses: `completed_checks`, `failed_check`,
`failed_execution`, or `unsupported`. It never starts `card` or `pandad`,
constructs a CAN publisher, contacts a vehicle, or keeps Params/messaging state
after the command exits.

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

When the host cannot import the supported runtime, the command records
`unsupported`; when a retained rlog does not contain all required active or
fault sequences, it records `failed_check` with the fixed availability counts.
Neither result satisfies the real integration profile. The existing startup
pair is deliberately cold/inactive evidence and therefore cannot establish
this transition profile by itself.

`elapsed_seconds` is execution metadata outside the canonical `transition`
result. It excludes workload outside this replay and does not establish device
scheduling or latency. A release profile that needs device timing must record
that separately on the target device. The report hashes the process/schema and
transition reader sources, preserves generated diagnostics/state identifiers,
and records no raw rlog bytes in Git.
