# Check whether inferred receipt timing changes a policy comparison

Historical logs can contain many controller requests that produce the same limited
integer command. Exact baseline reproduction therefore may not identify which request
card consumed. A long constant original command can hide substantial request age.
Changing a candidate's command shape can expose that ambiguity through feedback and
integrator state, even though the original command reproduces under both histories.

Run the original case twice under `earliest` and `latest`, each with the proposed
candidate and an otherwise-equivalent current-control revision. Keep original/warmup/
limiter revisions, inputs, settings, anchor, activation and scoring fixed. Qualify each
baseline before executing its candidate. Preserve failed histories; one passing history
cannot stand in for the other. These two histories are sensitivity cases, not bounds
over all possible receipt sequences or evidence of actual receipt latency.

```text
python -m tools.mazda_ti.sampling_sensitivity --earliest-control EVIDENCE/control-earliest --earliest-candidate EVIDENCE/candidate-earliest --latest-control EVIDENCE/control-latest --latest-candidate EVIDENCE/candidate-latest --data-root RAW --evidence-root EVIDENCE --output NEW-REPORT
```

Pass full common evaluation bundles, not their internal baseline/candidate directories.
The command verifies all four bundles against current source and raw identities, rejects
different windows/settings/original sources/inputs, changed per-role controller revisions,
different dependency/runtime identities, and unaligned send timestamps. Verification
checks retained identities; it does not re-execute the recorded runtime.

The deterministic report keeps each history's candidate-minus-current-control effect,
then subtracts those effects at identical send publications. This prevents changes in
the current controller itself from being credited to the new policy. Request ages are
measured from creation to send publication, not to motor delivery or firmware sampling.
All four result hashes and relative bundle paths are retained. A maximum command
difference is not a handling severity score, and a difference between histories is not
evidence that either history happened on the vehicle.

When a history changes the conclusion, inspect `prepared/sampling.json` and the retained
controller traces around the first feedback/integral divergence. Use exact diagnostic
identities from instrumented drives for stronger evidence where available. Preserve
uncertain contact and physical lane outcomes separately from software replay effects.
