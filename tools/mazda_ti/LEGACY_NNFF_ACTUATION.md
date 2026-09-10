# Historical NNFF steering-command audit

`python -m tools.mazda_ti.legacy_nnff_actuation` checks original request scaling,
TI and stock limiter histories, decoded steering counts, and previous-apply TI
feedback for clean route `00000261--7120d4c827`. Read
[the NNFF reconstruction guide](LEGACY_NNFF.md) first. This is a component audit
on recorded observations, not full process replay or physical-path prediction.

Supported original revision: `2a098cdbb2ae1a1c231220f37faca45b9d8b8c97`.
Its recorded TI scale is650, while stock request normalization is600. That
historical setting does not authorize raising the present vehicle's TI600 limit.
Stock wire requests are not measured EPS delivery or the EPS's308-count ceiling.
The ordinary600-only candidate adapter is unchanged.

## Reproduce

Read [storage policy](STORAGE.md), preflight capacity and use a new owned scratch
output. Generate all four full NNFF traces using the matching shared request,
`--constraints serialized --receipt-sequence`, earliest/latest input choices,
and carState/controlsState output bounds, as described in LEGACY_NNFF.md.

```text
python -m tools.mazda_ti.legacy_nnff_actuation --request tools/mazda_ti/cases/legacy_nnff_261_holdout2.json --data-root ABSOLUTE-RAW-ROOT --receipt-mode echo-latest --controller-trace ABSOLUTE-SCRATCH/earliest-carState.json --controller-trace ABSOLUTE-SCRATCH/earliest-controlsState.json --controller-trace ABSOLUTE-SCRATCH/latest-carState.json --controller-trace ABSOLUTE-SCRATCH/latest-controlsState.json --output NEW-ABSOLUTE-SCRATCH/actuation.json
```

Omit controller traces to audit recorded requests alone. Run `latest-bound` and
`echo-earliest` separately and preserve their failures. Exit0 means the report
was written. `summary.exact` concerns **recorded requests only**;
`all_supplied_controller_arms_exact` additionally requires nonempty supplied arms
with exact counts and unchanged checked feedback flags. Neither is release
qualification. A failed recorded-request baseline prevents controller-arm runs.

## Request receipt and output timing

Original card publishes carOutput from the previous apply, then carState, then
applies the sampled request and publishes sendcan. The tool maintains separate
TI/stock limiter histories after one initialization; it never reanchors each
frame. Treating carOutput as the current apply changes feedback timing.

Original Mazda code copies the request's actuator struct, replacing only steer
and steerOutputCan. The next carOutput therefore echoes gas, brake,
steeringAngleDeg, accel, longControlState, speed and curvature from the request
just applied. Those fields constrain retrospective request identities; steer
and steerOutputCan never select the history. They do not imply longitudinal
authority. The echo is retrospective evidence, not a future controller input.

`echo-latest` selects the latest monotone compatible request published before
the preceding output. On holdout2, copied fields exclude two requests selected
too early by the original latest-publication rule. This repairs19 TI,3 stock,
and19 output mismatches. The other two windows' latest maps are unchanged.

Copied fields are not unique: up to5 compatible requests remain, with ambiguous
identities on3849/3653/3637 of6060 card cycles for VW/holdout1/holdout2.
`echo-earliest` fails4560/4841/4727 TI counts respectively; preserve these
failures. Successful latest histories do not prove every historical receipt
identity or qualify all possible sampling histories.

## Recorded results, 2026-09-09

Each actuation window covers6060 sends, including one second of controller
settling before the scoring windows in LEGACY_NNFF.md. Recorded requests
reproduce all TI/stock counts and all previous-apply output values in all three
`echo-latest` cases:18180 sends per actuator and18181 output publications total.

Substituting reconstructed NNFF output gives the same result under each of the
four controller-input timing choices:

| Window | TI count mismatches | Stock count mismatches | Largest difference | Changed next-cycle limiter flags |
|---|---:|---:|---:|---:|
| VW and preceding curve | 7 | 4 | 1 count | 0 |
| First unused section | 1 | 0 | 1 count | 0 |
| Second unused section | 0 | 0 | 0 | 0 |

Main VW subwindow284.221355775–309.521644568 alone has2530 sends: all TI
counts match, with4 stock one-count differences. The7 TI differences above are
on the preceding curve. Holdout1 reaches629 recorded TI counts; do not describe
that historical case as within the present600-count envelope.

At262.911070940s, normalized native/reconstructed output multiplied by650 is
361.500611901/361.499798298. Rounding produces362/361 and the rate limit carries
the one-count difference across7 frames. Stock rounding boundaries similarly
explain the four VW stock differences. These are retained exact-match failures;
no tolerance is widened. They are consistent with the small normalized-output
residuals, whose complete numerical origin remains unqualified.

The feedback check asks whether reconstructed applied output would change the
next controller cycle's `abs(request-output)>0.01` limiter decision. It is not
a general candidate-feedback adapter. Fixed observations, continuous active TI
RUN, historical source, and initial states remain conditions of this result.
Full CAN payload/checksum reproduction, transitions, changed physical motion,
and a cure for scalloping remain outside its scope.

Nine focused tests cover scaling, independent limiter histories, previous-apply
ordering, invalid replacements, copied-field exclusions and monotone timing:

```text
python -m pytest tools/mazda_ti/tests/test_legacy_nnff_actuation.py --confcutdir=tools/mazda_ti/tests -o addopts= -p no:cacheprovider --basetemp NEW-ABSOLUTE-TEST-SCRATCH -q
```

Retain compact summaries, failing cases, requests, hashes and reproduction
instructions. Full traces belong in disposable scratch while used for timing
analysis. After pruning, summaries require rebuilding full evidence before
verification; they cannot substitute for a release bundle.
