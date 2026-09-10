# Isolate NNFF rate feedback

Use `legacy_nnff_feedback` when investigating whether the original neural
controller's rate inputs explain earlier correction. It reruns the fixed clean
261 VW case using a full [NNFF baseline](LEGACY_NNFF.md), captures neural call
inputs, and evaluates feedback with desired rate, measured rate, or both zeroed.
Every other feature stays fixed. Side evaluations never enter the original PID,
feedforward, history, actuators or physical observations.

The rate feature is scaled lateral jerk, not raw steering rate. Original full
NNFF evaluates setpoint and measurement separately and can blend a stronger
error response at high demand. The helper preserves that nonlinear expression
and Float32 stores; the individual rate effects therefore need not add.

## Run and verify

Read [storage policy](STORAGE.md), preflight owned scratch and retain a full
unchanged baseline from the fixed `legacy_nnff_261_vw.json` request. The helper
rejects another request or a changed complete baseline. Run all four baseline
input-choice/output-bound combinations and a fresh repeat:

```text
python -m tools.mazda_ti.legacy_nnff_feedback --baseline ABSOLUTE-SCRATCH/vw-nnff-earliest-carState.json --data-root ABSOLUTE-RAW-ROOT --output NEW-ABSOLUTE-SCRATCH/feedback.json
```

The report requires exact equality between the instrumented and complete
original replay result, plus exact original feedback-expression agreement on
every update. It retains input vectors, neural evaluations, controller terms,
timestamps, source/raw provenance through the baseline, and rate interventions.
This equality preserves the baseline's existing numerical residuals against
the drive; it does not remove them. No saturation/process/physical qualification
is inherited. The tool supports only the pinned 18-input neural model, no
friction override, and active full-neural path.

## Fixed first-unwind result, 2026-09-09

Use the existing `lane_motion_balance_vw.json` windows, not a window optimized
for this hypothesis. NNFF261 contributes 550 rows over
291.721355775–297.221355775 s. All four timing cases agree on these findings:

- Both rate inputs are zero throughout the 62 rows from the window start
  through first negative proportional feedback at 292.335206241 s. Zeroing
  either or both therefore leaves that onset unchanged. Rate feedback does
  **not** explain the initial correction in this recorded unwind.
- Rate features are nonzero on 185 of 550 rows later in the window. Compared
  with zeroing both, they change a nonnegative error to negative on 16 rows,
  and a negative error to nonnegative on 38. They do not act as uniform damping.
- At 293.495236917 s, original feedback error is -0.0111663 versus -0.0855995
  with both rate features zero. At 294.235265459 s, original is -0.0951135
  versus +0.0109681 without those features. These are instantaneous expression
  differences, not candidate trajectories or evidence of worse/better handling.

Compared with the existing [28c reference audit](LEGACY_REFERENCE.md), time from
current request first falling below measured lateral acceleration is:

| Feedback event | Smooth261 NNFF | Symptomatic28c plant |
|---|---:|---:|
| First negative P | 0.000 s | 0.950 s |
| First P negative for at least 100 ms | 0.000 s | 1.000 s |
| First negative P+I | 0.350 s | 1.470 s |
| First P+I negative for at least 100 ms | 0.950 s | 1.470 s |

The negative-P+I onset in261 is initially intermittent; reporting only its
first sign change overstates the sustained timing difference. Including logged
D on28c does not change the listed combined-feedback onset. NNFF's explicit D
is zero. The reference crossing times are292.335206241 s on261 and
269.876259947 s on28c. A continuous-negative run fails across gaps over20 ms.
The100 ms duration is a descriptive sensitivity check, not a release criterion.

Only sign timing is compared: NNFF native P/I/F are torque-domain values while
28c native terms are acceleration-domain values. Feedforward and compensation
can still produce inward torque during negative feedback. This is evidence
that the two controllers respond to different references, not proof that
inward torque is then physically wrong.261 is slower; matched lane and road
do not establish matched within-lane entry or independent lane truth.

## Tests and retention

Four focused tests cover rate isolation without mutating inputs, nonlinear
blend sign checks and invalid vectors. Fresh capture repeats are byte-identical.

```text
python -m pytest tools/mazda_ti/tests/test_legacy_nnff_feedback.py --confcutdir=tools/mazda_ti/tests -o addopts= -p no:cacheprovider --basetemp NEW-ABSOLUTE-TEST-SCRATCH -q
```

Keep compact results and source/input hashes; full captures belong in disposable
scratch. The local study's `summarize_nnff_feedback.py` binds the shared fixed
windows and the28c reference report, preserves instantaneous and sustained
crossings, and writes `nnff-feedback-summary-v1.json`. Pruned full captures must
be rebuilt before verification. This module exposes a mechanism for diagnosis;
it neither proposes a steering policy nor predicts a cure for scalloping.
