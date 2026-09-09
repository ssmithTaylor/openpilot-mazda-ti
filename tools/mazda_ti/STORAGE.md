# Scratch space and compact retention

Keep manifests, controller revisions, input hashes, findings and reproduction
instructions in durable storage. Put full traces, prepared mappings, redundant
cache copies and pytest temporary files in a dedicated disposable scratch
directory. Raw drive recordings are separate source data.

For this Windows campaign, use a named directory under the system temporary
directory on C: for bulky work. F: holds the repository and compact study records.
Read current free space before a run; a cache often duplicates the full output.
Use an explicit scratch output and a separate scratch cache even when the batch
CLI's default would place the cache inside its output.

## Before execution

The read-only preflight checks explicit absolute output/cache paths, a caller's
estimate of the bytes needed by **each** workspace, and a free-space headroom
amount. The example values are planning inputs, not measured run-size guarantees:

```text
python -m tools.mazda_ti.storage_preflight --output ABSOLUTE-SCRATCH/output --cache ABSOLUTE-SCRATCH/cache --required-workspace-bytes 2000000000 --headroom-bytes 5000000000 --report ABSOLUTE-DURABLE/preflight.json
```

Exit 0 means the declared demand fits at that instant; 1 means insufficient
space; 2 means invalid inputs or a reporting failure. Gate execution on exit 0.
The command creates only the requested report, outside both target directories.
It groups paths by filesystem identity, including aliases, and counts output
and cache separately. Headroom is counted once per filesystem. Changing free
space readings use the conservative minimum. It neither reserves space nor
limits actual output size; revise estimates from completed runs and inspect
headroom between batches. Stop new work if headroom is exhausted.

## During analysis and after a storage failure

Full bundles remain available in scratch while comparisons and reviews consume
them. Record their scratch location and hashes in the durable study. Keep failed
attempts until their error, request and identity information has been captured.
If a run stops, inspect its exact process handle and terminal state before
resuming. The batch runner can reuse verified completed attempts and creates new
attempts for failed work. Relocated attempts must retain identical bytes; source,
runtime and request checks still apply. Never relabel an ENOSPC failure as a
controller-baseline failure.

Large logical cache sizes do not guarantee the same amount of physical space
will be freed; filesystems may share blocks. Verify actual free space after
cleanup. Do not delete raw inputs or another project's data to make room.

## Long-term evidence

A compact summary is a record of findings and how to repeat the work. It is not
a substitute for the full bundle's trace, mapping and artifact verification.
Once bulky scratch is pruned, label the retained record **summary only; rebuild
the full bundle before verification or release qualification**. Preserve failure
findings as well as successful summaries. Reproduction still needs the declared
raw bytes, pinned controller source, compatible dependency snapshot and runtime.

Clean only explicitly owned generated scratch, after verifying its retained
record and current file inventory. A cleanup plan must identify exact resolved
roots, reject raw-source and reparse-point traversal, and preserve the compact
record outside the deletion target. Reclaim scratch after its comparisons and
review are finished; retaining every historical full run indefinitely defeats
the storage policy.

### Pack and verify a cleanup plan

Use `retention_pack` after analysis. Supply absolute paths; the durable pack and
cleanup manifest must be outside the generated run. Repeat `--protected-root`
for raw inputs and any active evidence that must remain available.

```text
python -m tools.mazda_ti.retention_pack pack --study-root ABSOLUTE-SCRATCH --run-root ABSOLUTE-SCRATCH/completed-run --pack ABSOLUTE-DURABLE/run.json.gz --protected-root ABSOLUTE-RAW --max-retained-file-bytes 262144 --max-retained-total-bytes 2097152
python -m tools.mazda_ti.retention_pack verify-cleanup --study-root ABSOLUTE-SCRATCH --run-root ABSOLUTE-SCRATCH/completed-run --pack ABSOLUTE-DURABLE/run.json.gz --protected-root ABSOLUTE-RAW --manifest ABSOLUTE-DURABLE/cleanup.json
```

Generated run roots require the batch `cases/*/attempt-*/result.json` layout or
an explicitly created `.mazda-generated-run.json` ownership marker. A marker
records caller ownership; it is not authorization to discard unrelated data.
The tool rejects traversal, reparse points, known raw-recording names and
overlapping protected roots. Pass real scratch paths, not junction aliases.

The deterministic gzip pack inventories every file's name, size and SHA-256.
Embedded metadata prioritizes reproduction requests, then findings, then
redundant nested result records. Per-file and total byte budgets bound embedded
source bytes; the complete inventory and compression overhead are additional.
Check actual pack size. Omitted metadata have explicit budget reasons. Store
the study's human findings and reproduction manifest alongside the pack.

`verify-cleanup` compares the current tree and regenerated pack byte for byte,
then emits an exact resolved deletion root and file inventory. It never deletes.
After confirming all writers have stopped, recheck the plan's pack hash and
target inventory immediately before a native deletion of that one root. On
Windows, use one PowerShell operation with `-LiteralPath` after checking the
resolved root remains inside the intended scratch directory. Remove obsolete
junction aliases separately without recursing through them. Keep the pack and
a deletion receipt; later consumers must rebuild the pruned full bundle.
