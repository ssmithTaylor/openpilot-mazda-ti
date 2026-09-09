# Mazda release qualification

`python -m tools.mazda_ti.release` prepares a portable release record from four
already-retained evidence records: a corpus result, a scenario result, a real
process result, and a recorded comparison. The versioned `mazda-release-v1`
profile requires every kind. Each record must be completed, hash-identified,
bound to the requested candidate commit, and contain source and input
identities. Missing, stale, unsupported, failed, or unqualified evidence keeps
the release unqualified and remains visible in the result.

The request also records the settings used for review, per-evidence concerns,
and a specific physical question. The resulting record describes software and
recorded-motion qualification only. It does not predict an acceptable corner
path or establish driver contact.

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
