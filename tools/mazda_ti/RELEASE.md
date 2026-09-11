# Mazda release qualification

`python -m tools.mazda_ti.release` prepares a portable release record from four
already-retained evidence records: a corpus result, a scenario result, a real
process result, and a recorded comparison. The versioned `mazda-release-v1`
profile requires every kind. Each declared kind is checked against its producer
contract: a passed corpus profile, a synthetic scenario result, a
`full_process_transition` result from the exact
`selfdrive.test.process_replay.replay_process` interface, and a trace comparison whose aggregate and
candidate arm both report `verified_full_bundle`. Each record must be completed,
hash-identified, bound to the requested candidate commit, and contain source and
input identities. Relabelled evidence, `content_bound_unqualified` comparisons,
missing, stale, unsupported, or failed evidence keeps the release unqualified
and remains visible in the result.

The request also records the settings used for review, per-evidence concerns,
and a structured physical question. `physical_evaluation` requires a stable
`route_segment` identity, a `maneuver` (`left_curve`, `right_curve`, `straight`,
or `transition`), a positive `target_speed_mps`, both `lane_position` and
`driver_steering_intervention` measurements, and an integer
`maximum_driver_steering_interventions`. The tool renders those fields into the
question in the portable result; it does not accept subjective question prose.

```json
{
  "physical_evaluation": {
    "route_segment": "route-28f:835-875",
    "maneuver": "left_curve",
    "target_speed_mps": 20,
    "measurements": ["lane_position", "driver_steering_intervention"],
    "maximum_driver_steering_interventions": 0
  }
}
```

The resulting record describes software and recorded-motion qualification only.
It does not predict an acceptable corner path or establish driver contact.

Deployment prerequisites are emitted as `pending` records:

- exact candidate on remote main before device update;
- change-dependent build and parameter-sync checks;
- settings preservation;
- post-reboot running-source verification.

The command never contacts a device, pushes or pulls Git, reboots, publishes
CAN, or actuates a vehicle. A completed record is eligible for separate
authorization and does not claim that any pending prerequisite was performed.

Evidence references are relative to `--evidence-root`; absolute paths and paths
escaping that root are rejected. The result keeps relative artifact references,
SHA-256 identities, status, source identities, input identities, and concerns.
Measured execution time is written separately as execution metadata.
