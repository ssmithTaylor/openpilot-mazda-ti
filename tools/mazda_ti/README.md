# Mazda TI replay evidence tools

For structured rlog fields, the identity-coverage audit command, sign conventions and logging tests, read [DIAGNOSTICS.md](DIAGNOSTICS.md). Verify the recorded diagnostic versions and vehicle source separately from the analyzer's current checkout.

For instrumented-drive candidate replay, that guide also documents `RecordedTiFeedback`,
the shared library for coupling candidate requests to software TI feedback through exact
consumed-output and applied-command identities. It is separate from the historical runner
below and requires a caller that reconstructs controller inputs and state.

`verify_replay.py` checks that an integrated controller reproduces an already-qualified reference replay, and that the source files still match the integration run's recorded hashes. It runs with Python3.11+ and the standard library on Windows or Linux. It reads local files and performs no network, vehicle or deployment operations.

The shared runner now prepares input identities and runs baseline/candidate replay directly from raw rlogs. It has been checked against the qualified VW window under both compatible request histories and a second window containing an actuator-state transition. Preparation and candidate artifacts reproduced byte-for-byte after relocating the raw inputs. This does not yet qualify every older corpus case or predict improved lane tracking.

Version2 runner records cover relevant repository dependencies (including PID, plant, limiter, schemas, controlsd and card), every shared Python tool, loaded repository modules, Python version/executable, and numpy/pycapnp versions plus Python/native runtime files. Candidate execution requires an exact recorded-command baseline from the same preparation, history and environment, with unchanged source and artifact hashes. These are reproducibility checks, not signed attestations or a hermetic machine image. Legacy integration locks still cover only their listed three runtime files; they cannot qualify changes elsewhere.

## Prepare and replay

Run these commands from the repository root. The runner needs numpy and pycapnp compatible with the chosen Python runtime; the verifier itself remains standard-library-only. It loads real controller/PID/filter/plant/limiter code with constant-only facades for hardware imports. It does not open sockets, invoke vehicle hardware or modify the checkout.

Create an experiment JSON with these fields:

| Field | Meaning |
| --- | --- |
| `format_version` | `1` for the experiment specification |
| `name` | Descriptive experiment name |
| `rlogs` | Distinct paths relative to the data root, such as `route--3/rlog`; uncompressed Cap'n Proto events |
| `window` | `anchor_i_at`, `activation`, `start`, `end`, all monotonic seconds; anchor precedes activation, activation is at/before scoring start |
| `baseline_controller`, `warmup_controller`, `limiter` | Explicit Git revisions; preparation resolves them to full commits |
| `candidate_controller` | Explicit revision or `worktree` |
| `fpcs_sample_at` | `carState` or `controlsState`; a declared publication-based sampling assumption |
| `force_offset` | Explicit boolean for the historical live-torque offset policy |
| `settings` | Numeric values for the required recorded parameters below |

Required settings are `TiSteerMax`, `TiSteerDeltaUp`, `TiSteerDeltaDown`, `TiSteerDriverAllowance`, `TiSteerDriverMultiplier`, `TiSteerDeltaUpKnee`, `TiSteerDeltaUpHigh`, `LatDamping`, `LatCommitSetpoint`, `LatFrictionComp`, `LatOutputFilter`, `LatNoFrictionRelay`, `TorqueInterceptorEnabled`, and `SteerKP`. Read these from raw `initData.params.entries`, not an extraction summary that may omit toggles. Only these whitelisted settings enter the report; other recorded Params can contain private information.

The current runner requires TI600, interceptor enabled, output smoothing off, and the friction relay disabled. It checks the expected initial settings in every supplied segment. That does not prove those values persisted at runtime; baseline reproduction and recorded flags remain necessary, and newly instrumented drives can provide direct runtime settings. Configure enough preceding segments for reference, friction, integral and actuator-state history. An integral anchor is a declared state initialization, not a fitted correction on every frame.

```text
python -m tools.mazda_ti.run prepare --spec experiment.json --data-root PATH/TO/rlogs --output evidence/prepared
python -m tools.mazda_ti.run replay --prepared evidence/prepared --data-root PATH/TO/rlogs --variant baseline --history earliest --output evidence/baseline-earliest
python -m tools.mazda_ti.run replay --prepared evidence/prepared --data-root PATH/TO/rlogs --variant candidate --history earliest --baseline-result evidence/baseline-earliest/result.json --output evidence/candidate-earliest
```

Repeat baseline and candidate with `latest` and distinct output directories. These are two compatible sampling histories, not bounds on all possible histories. Candidate execution uses its own commands and software TI feedback after activation; recorded vehicle motion remains fixed. The schema's newly logged input identities are not yet consumed by this historical-log runner.

Preparation writes the resolved spec, input audit, identity mapping, request sampling maps and `preparation.json`. Replay writes `trace.json`, `trace.jsonl`, `trace-sends.json` and `result.json`. The baseline requires non-empty coverage, identical integer sends, matching plant flags, and less than one count of controller-output residual. The residual allowance is separate from the exact integer-command requirement. Failed evidence is retained, and candidates are refused when their baseline is unqualified. The isolated historical Float32 rounding exception is not implemented in this shared runner; it must fail rather than silently accept one-count mismatches.

Use a new output directory for every run. Changed source, runtime, prepared artifacts or raw-log bytes require fresh preparation and qualification. Paths are relative in reports, which allows identical bytes at a different data root. Raw logs, generated traces and large result manifests should remain outside Git.

## Compare an integration

```text
python tools/mazda_ti/verify_replay.py --baseline PATH/TO/reference --candidate PATH/TO/integration --source-lock PATH/TO/integration-result.json --start 230 --end 327 --additional-log-flag 4096 --output verification.json
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti/test_verify_replay.py
```

Each prefix identifies a `.jsonl` control trace and a `-sends.json` integer-send trace. Times are route monotonic seconds; send timestamps are integer nanoseconds. `--source-lock` is the result captured by the integration run with a `production_sha256` mapping for the controller, lane helper and controlsd. Preserve that record with the traces. It is not a request to hash today's source and assign it to old results.

The optional test command requires pytest. Its explicit configuration boundary excludes hardware-dependent repository fixtures and plugins; the verifier itself needs no openpilot runtime or third-party packages. The full shared checks are:

```text
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti/test_workflow.py tools/mazda_ti/test_verify_replay.py tools/mazda_ti/test_diagnostics.py tools/mazda_ti/test_audit_diagnostics.py tools/mazda_ti/test_audit_lane_context.py tools/mazda_ti/test_recorded_feedback.py tools/mazda_ti/test_measurement_history.py
```

The verifier requires identical active frame identities, controller/reference/geometry fields, integer sends and their coverage. The optional4096 allowance verifies the newly added lane-release log flag on precisely the reference-removal frames. It cannot waive a torque or reference difference. The TI600-count and15-count adjacent-send bounds are checked for this campaign's configuration.

Reports contain byte hashes and no wall-clock timestamps or absolute machine paths, so the same input bytes produce the same report. Existing output files are preserved. Version2 verification also checks the broader repository source map and the candidate's saved artifact hashes; it does not execute or compare the current runtime environment. Changed source, missing frames, changed commands or non-finite JSON produce a nonzero exit. Source hashes deliberately cover actual file bytes; line-ending conversion may require a fresh integration run rather than reusing an old source lock.

Keep raw drives and large generated traces outside Git. Share the tool, tests and skill; provide evidence artifacts separately when another reviewer needs to reproduce a particular comparison.
