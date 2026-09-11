# One recorded case from revision to report

`python -m tools.mazda_ti.evaluate` implements [issue #2](https://github.com/ssmithTaylor/openpilot-mazda-ti/issues/2):
prepare one historical or instrumented case, qualify its original baseline, evaluate a pinned candidate,
and write a structured bundle and readable comparison. It reuses the real-controller
runner, TI limiter/feedback, state migration, source locks and exact-send verifier.
It performs no network, vehicle, Params, CAN, deployment or source-write operations.
The [instrumented adapter](INSTRUMENTED_EVALUATION.md) adds exact diagnostic identities and both actuator histories.
Use [CORPUS.md](CORPUS.md) for serial mixed-corpus evaluation. Worktree snapshots and
process checks remain unsupported by this single-case command.

## Reproducible references

Use Python 3.11+ with NumPy and pycapnp. Local qualification used CPython 3.14.7,
NumPy 2.5.2, pycapnp 2.2.4 and pytest 9.1.1 on Windows AMD64. Bundles record the
Python executable and package/native-file hashes: version strings alone do not pin a runtime.
Run from the repository root, always choosing new output directories.

The public fixture is checked in as readable serialization code in `example_fixture.py`.
It supplies 30 synthetic straight-line input frames: zero steering ask, zero measured
lateral acceleration, and independently specified zero integer commands. It does not
run a controller to manufacture expected outputs. Preparation, the historical controller,
limiter, baseline qualification, candidate feedback and verifier must reproduce all
19 scored sends. Repeated fixture generation produces identical input bytes and hashes.

```text
python -m tools.mazda_ti.example_fixture --output PATH/TO/public-fixture
python -m tools.mazda_ti.evaluate --request PATH/TO/public-fixture/request.json --data-root PATH/TO/public-fixture --output PATH/TO/evidence/public-01
python -m tools.mazda_ti.evaluate --verify-bundle PATH/TO/evidence/public-01 --data-root PATH/TO/public-fixture
```

Its `origin: synthetic` result explicitly denies recorded physical handling evidence.
It is a functional pipeline check, separate from the private recorded VW qualification:

```text
python -m tools.mazda_ti.evaluate --request tools/mazda_ti/examples/vw-evaluation.json --data-root PATH/TO/rlogs --output PATH/TO/evidence/vw-01
python -m tools.mazda_ti.evaluate --verify-bundle PATH/TO/evidence/vw-01 --data-root PATH/TO/rlogs
```

The supplied VW request pins `0000028c--0ea8954ea3--3/rlog` and `--4/rlog`, their
SHA-256 values, all 14 whitelisted settings and source commits. Obtain the private
uncompressed rlogs through the authorized data-sharing channel; they are excluded
from Git. Their sizes are 45,213,968 and 47,075,352 bytes respectively.
Baseline, warmup and candidate use `2c50a68456d6f8eb18c5688a72a2c055a2572182`;
the limiter is `e409edb8d7ce3766d02379f2f2dafcb5887007ce`. Warmup starts with the
supplied segments, integral anchoring is at 225 seconds, activation at 230, and scoring
at 230–290 monotonic seconds. History is `earliest`, and FPCS sampling is `carState`.
Expected: 6,000 exact baseline sends and zero changed same-source candidate sends.
Different source/runtime dependencies require fresh qualification; these counts waive no checks.

## Version 1 contract

A request contains `format_version: 1`, `candidate_revision`, and `case`:

| Case field | Meaning |
| --- | --- |
| `id` | Stable case identity |
| `method` | `historical` or `instrumented` |
| `history` | Historical: one explicit `earliest` or `latest`; instrumented: `exact` |
| `input_sha256` | SHA-256 for exactly every relative rlog path |
| `experiment` | [Runner spec](README.md), excluding the candidate supplied by the request |
| `origin` | `recorded` by default, or `synthetic`; persisted explicitly |

Every object has a strict field allowlist, including window and settings. Unknown fields,
extra Params, nonnumeric settings and invalid paths/hashes fail before copying requests.
Rejection messages do not echo rejected field names or values. Stored and resolved requests
are rebuilt from validated fields. A candidate revision selects **controller-file source**;
other dependencies are captured from the current checkout. It does not execute an entire
arbitrary historical repository. Aliases are resolved once to full commits before preparation.
Worktree candidates are unsupported; commit the intended controller first.

The output must be new. Completed bundles contain `request.json`, `resolved-request.json`,
`experiment.json`, runner artifacts in `prepared/`, `baseline/`, `candidate/`, `result.json`,
`report.md` and `execution.json`. Result version 1 has four statuses:

| Status | Meaning |
| --- | --- |
| `completed_checks` | Declared baseline and candidate command checks completed |
| `failed_check` | Input/provenance, schema, baseline or command invariant failed |
| `unsupported` | Version, method/history, origin or source mode is unsupported |
| `failed_execution` | Required file/runtime unavailable or execution could not finish |

Only completed checks return CLI exit zero. Candidate comparison is absent after failure;
baseline failure blocks candidate execution. Partial evidence remains. An unwritable output
cannot retain a bundle and produces stderr instead. No status establishes successful cornering.
Exact integer commands, matching plant flags, nonempty coverage, output residual below one
count and TI600 remain unchanged. Historical replay retains its 15-count adjacent-send
bound; instrumented replay checks each actual TI/stock limiter, including bypass and reset jumps. A controller residual allowance
never permits a one-count send mismatch. Comparisons retain minimum/maximum and absolute
command changes without a weighted handling score.

The verifier checks all artifact/source/raw identities and matching preparation/history/runtime
records, then rebuilds every trusted top-level field: case, window, input/runtime/source
provenance, limitations, scope, status, findings, qualification and comparison. Tampering fails.
It does not execute the recorded runtime or provide cryptographic attestation. Canonical artifact
hashes exclude the result/report and `execution.json`; machine paths and measured elapsed time
stay in execution metadata. Desktop elapsed time is not device scheduling evidence. Raw-data
relocation preserves canonical comparisons. This slice always prepares anew and claims no cache speedup.

## Tests and extensions

```text
python -m pytest -c tools/mazda_ti/pytest.ini --confcutdir=tools/mazda_ti tools/mazda_ti
```

The default suite runs the public end-to-end fixture plus structural rejection and tamper tests.
GitHub Actions runs it on Windows and Linux. Set `MAZDA_EVAL_DATA_ROOT` to include the private
recorded VW and TI-transition tests; otherwise each reports an explicit skip. For an inaccessible Windows pytest temp
root, use `--basetemp` with a fresh directory under an existing writable parent.

The public boundaries are `evaluate(request_path, data_root, output) -> result` and
`verify_bundle(output, data_root)`. Request validation/result derivation live in
`evaluation_contract.py`. Future adapters must version shared fields, qualify their own baseline,
retain source/runtime/input identities, preserve candidate-owned feedback and state migration,
and keep unsupported coverage and deployment separate. See [README.md](README.md) for historical
sampling/anchor limitations. Fixed physical observations, planner output and availability remain
recorded; synthetic fixtures remain labeled. Commands cannot establish a new vehicle path,
driver contact, a firmware threshold or an acceptable corner path.
