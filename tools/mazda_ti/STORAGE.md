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
