# Mazda fresh-session acceptance

Run the public acceptance from a new checkout or a new agent session when you
need to verify the complete local evaluation infrastructure without private
drive data or session notes. The command creates its fixtures from source; the
output directory must not already exist.

## Setup and run

Use Python 3.11 or newer with the repository's numpy and pycapnp dependencies.
The candidate must be a committed revision available in the local Git object
database. Run from the repository root:

```text
python -m tools.mazda_ti.fresh_session --candidate HEAD --output PATH/TO/NEW-OUTPUT
```

The runner performs this local chain twice:

1. Generate independent, byte-identical historical and exact-identity
   instrumented fixtures at different paths.
2. Discover and explicitly collect both routes through the ingestion API into
   a new raw root.
3. Evaluate both adapters through the resumable batch API, then evaluate the
   mixed corpus with its retained excluded case.
4. Run the controller scenario producer and the deterministic process-contract
   producer, compare the verified instrumented trace arms, and bind the four
   producer results through release qualification. The fixture process profile
   is intentionally rejected by that release boundary.
5. Repeat from the relocated fixture and raw roots with new evidence outputs
   and the shared validated case cache.
6. Reject changed source, raw, manifest, runtime, and evidence identities, then
   prove a missing full-process runtime produces an unqualified release.

`result.json` and `report.md` are the portable acceptance boundary. They contain
relative artifact names, statuses, SHA-256 identities, exclusions, and evidence
limits. `execution.json` contains host details, absolute output location, cold
and repeated elapsed time, cache hits, estimated saved time, and remaining
bottlenecks. An all-miss run is `cold`, an all-hit run is `repeated`, and a
partial hit is `mixed`.

## Read the result

A `completed_checks` infrastructure acceptance requires identical fixture bytes, canonical
artifact hashes, and qualification decisions across both locations. Both replay
adapters must reproduce their baselines, the second batch must reuse every
valid cached case, all five invalidation probes must reject changed evidence,
and both the process fixture and intentionally unsupported process result must
keep their releases unqualified.

The completed process fixture demonstrates the transition producer contract,
but it carries `process_transition_fixture`, which the release qualifier rejects.
It cannot become `full_process_transition` through an injected capability or
boundary. `execution.json` reports the host's actual process capability. A
qualified release still requires separately retained evidence produced by the
real process boundary in the declared Linux openpilot runtime with suitable
Mazda rlogs.

## Diagnose a failure

- `Fresh-session acceptance could not start` usually means the output already
  exists, the candidate revision is unavailable locally, or Python dependencies
  are missing. Preserve the existing directory and choose a new output path.
- `canonical_outputs_portable: false` means a canonical JSON value or report
  contains an absolute Windows, UNC, or POSIX path. Full exception details,
  tracebacks, unique messaging prefixes, and machine paths belong in execution
  metadata.
- `canonical_outputs_equal: false` means a canonical producer captured
  location, timing, task order, or another unstable value. Compare the named
  component hashes in `run-a` and `run-b`; timing differences belong only in
  execution files.
- A `mixed` repeat means at least one cache entry failed verification and was
  recomputed. Inspect the batch execution record's invalidation reasons and the
  retained `attempt-*` directories. Do not relabel a partial hit as a repeated
  timing measurement.
- An unsupported host process capability is expected outside the declared
  runtime. It cannot satisfy the full process profile or qualify the incomplete
  release.
- A changed raw file, source dependency, manifest, runtime, or release evidence
  file requires new preparation and qualification. Failed and prior attempts
  remain available for diagnosis.

The generated raw logs, cache, traces, and release bundles belong outside Git.
The workflow reads local Git objects but performs no network access, Git remote
operation, device communication, deployment, CAN publication, or vehicle
actuation.

## Coverage boundary

The fixtures establish deterministic software commands, exact baseline
reproduction, TI loss and stock fallback, TI re-entry, inactive-to-active
history, scenario invariants, producer contracts, comparison identity, and
release binding. The public corpus deliberately retains a difficult-corner
exclusion and declares recorded successful/failed physical outcomes and matched
entry conditions unavailable.

Infrastructure completion and a reproduced `unqualified` decision do not establish an acceptable lane path, driver
contact, tire grip, device readiness, or safe deployment. Physical handling is
still answered by separately authorized, matched-condition drive evidence.
Predictive simulation remains a separate research track with its own validation
domain.
