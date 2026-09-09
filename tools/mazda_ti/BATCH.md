# Resumable corpus execution

Run a versioned corpus manifest through the existing historical or instrumented
evaluation adapter:

```text
python -m tools.mazda_ti.batch --manifest corpus.json --data-root PATH/TO/raw \
  --output evidence/corpus --cache-root evidence/cache --workers 2
```

The manifest declares positive `max_workers`, `host_memory_mb`, and
`worker_memory_mb`, plus cases with a relative request path and `isolation` set
to `demonstrated` or `unproven`. The effective worker count is bounded by each
declared resource limit and the host CPU count. Any unproven process group runs
the batch serially. Historical and instrumented adapters remain responsible for
their own Params, messaging, and process constraints; this scheduler does not
start a vehicle process, send CAN, or change vehicle state.

Each case writes a new `attempt-NNN` directory. A completed attempt is reused
only after `verify_bundle` validates its raw input bytes, source lock, runtime,
resolved configuration, and every required artifact. Failed, interrupted, and
invalid cache attempts remain in place; a retry writes a later attempt rather
than overwriting evidence. Raw data may move to another data root when bytes
remain unchanged.

Before scheduling, branch references are resolved to full commits. A clean
`worktree` candidate is frozen to its current `HEAD`; a dirty worktree is
rejected. Source snapshots before and after each case reject source drift.
Canonical case results contain only validated adapter evidence and frozen
requests. `execution-NNN.json` separately records host identity, bounded worker
count, cold/repeated classification, elapsed time, and measured cases/second.
Those measurements describe that host and run only; they promise no speedup.

The public fixture tests cover equivalent serial/parallel canonical results,
relocation, interruption and failed-attempt retention, selective cache
invalidation, branch freezing, and serial process groups. Synthetic fixture
results remain software-command evidence, not vehicle handling evidence.
