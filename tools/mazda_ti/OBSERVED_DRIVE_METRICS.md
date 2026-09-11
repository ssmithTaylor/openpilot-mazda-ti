# Observed-drive measurement qualification

This measures recorded vehicle motion on already-supplied drives so the
evaluator's physical metrics (settling, lane-motion cycles, inward-hold/
late-wide, path quality, a lateral comfort proxy) can be qualified against
retained rider labels. It does not assess, rank, revive or retire a
controller candidate, does not set a physical acceptance threshold, and
does not authorize deployment. See
[`docs/superpowers/specs/2026-09-10-observed-drive-metrics-design.md`](../../docs/superpowers/specs/2026-09-10-observed-drive-metrics-design.md)
for the full design; this guide summarizes it and records the one real-data
calibration run attempted against it.

## Three modules

`tools/mazda_ti/observed_drive_rows.py` turns explicit rlogs into one
hash-bound, health-annotated row stream at model publication times. It reads
events through `tools.mazda_ti.audit_diagnostics.read_events` and loads the
production lane fit `selfdrive/car/mazda/lateral_reference.py::fit_lane_context`
by `importlib` from its path, recording its hash; production code is never
modified. CLI:

```text
python -m tools.mazda_ti.observed_drive_rows --data-root PATH --rlog route--seg/rlog --sha256 HEX --start-ns N --end-ns N [--margin-ns N] --output NEW_FILE.json
```

`tools/mazda_ti/maneuver_metrics.py` is a pure function library (no file I/O,
no hashing, no randomness): `orient`, `phase_anchors`, `settling`, `cycles`,
`hold_late_wide`, `path_quality`, `comfort_proxy`, `annotations`, and
`to_physical_records`. It has no CLI of its own; `measurement_qualification.py`
calls it.

`tools/mazda_ti/measurement_qualification.py` runs extraction and metrics
over a declared case set and reports measurement qualification, never a
handling verdict. CLI (as shipped, `python -m tools.mazda_ti.measurement_qualification <stage> ...`):

```text
calibrate --request REQUEST.json --params-out PARAMS_OUT.json --output OUTPUT.json [--params BASE_PARAMS.json]
evaluate  --request REQUEST.json --params PARAMS.json --output OUTPUT_DIR
```

`evaluate` keeps extraction in memory and writes only the compact
`result.json`/`report.md`/`holdout-sealed.json` outputs under `OUTPUT_DIR`;
it has no `--scratch` flag, and running `storage_preflight` over the
extraction workspace stays the operator's own step beforehand.

The exact commands run for the one real-data attempt (Task 9, 2026-09-10,
worktree-absolute paths; see "Real-data calibration attempt" below for why
`evaluate` was never reached):

```text
python -m tools.mazda_ti.storage_preflight --output .../rows --cache .../cache --required-workspace-bytes 2000000000 --headroom-bytes 5000000000 --report F:/repos/openpilot-mazda-ti/.claude/worktrees/comma-wifi-disconnect-logs-fd2949/.ti-local/observed-drive-metrics-20260910/preflight.json

python -m tools.mazda_ti.measurement_qualification calibrate --request F:/repos/openpilot-mazda-ti/.claude/worktrees/comma-wifi-disconnect-logs-fd2949/.ti-local/observed-drive-metrics-20260910/request-v3.json --params-out F:/repos/openpilot-mazda-ti/.claude/worktrees/comma-wifi-disconnect-logs-fd2949/.ti-local/observed-drive-metrics-20260910/params-v1.json --output F:/repos/openpilot-mazda-ti/.claude/worktrees/comma-wifi-disconnect-logs-fd2949/.ti-local/observed-drive-metrics-20260910/calibration.json
```

`storage_preflight`'s `--report` must be an absolute path (`_target` rejects
a relative one with `Paths must be absolute`, exit 2); `calibrate` and
`evaluate` accept relative paths.

## Row contract and per-group health

Each row is one `modelV2` publication joined to the latest preceding
`carState`, `liveLocationKalman`, `liveParameters`, torque-state
`controlsState`, and 0x249 `sendcan` frame, with each source's age recorded.
No row is dropped and no missing value becomes zero; a missing source
leaves its fields `null` with that row's group unhealthy. Health is
per field group, never one shared gate:

| Group | Healthy when |
| --- | --- |
| `lane` | model event valid; production fit returns `valid` (lane-change state off, both adjacent lane probabilities at least 0.8, width 2.5-5.0 m, fit support and agreement); camera age `t - timestampEof` within `[0, 0.2]` s |
| `speed` | carState exists, valid, CAN valid, age at most 0.2 s |
| `steering` | same carState conditions (angle and rate come from the same event) |
| `yaw` | localizer exists, valid, calibrated yaw valid, sensors/inputs/posenet OK, age at most 0.2 s |
| `accel` | localizer conditions plus calibrated acceleration valid |
| `roll` | liveParameters exists, valid, age at most 0.2 s |
| `controls` | torque-state controlsState exists, valid, age at most 0.2 s; `controls_active` recorded separately |
| `command` | 0x249 frame exists, age at most 0.2 s |

Row fields (right-positive unless stated, no sign normalization in the
extractor): `mono_ns`, `camera_eof_ns`, `camera_age_s`, `frame_id`, `health`,
`lane_change_state`, `lane_fit_reason`, `lane_probability_min`,
`lane_offset_m`, `lane_heading10_rad`, `lane_heading20_rad`,
`road_curvature10_per_m`, `road_curvature20_per_m`, `lane_width_m`,
`speed_mps`, `steering_angle_deg`, `steering_rate_dps`, `yaw_rate_rps`,
`yaw_std_rps`, `yaw_lateral_accel_mps2`, `device_lateral_accel_mps2`,
`roll_rad`, `controls_active`, `request_mps2`, `actual_mps2`, `diag_version`,
`ti_command_counts`, `ages_ns` per source. The output JSON carries
`format_version`, `scope`, the request echo, `source_sha256` (this module,
`lateral_reference.py`, `audit_diagnostics.py`, `provenance.py`,
`cereal/*.capnp`), `runtime`, `raw_sha256`, `initdata` settings, `window_ns`,
`rows`, `unhealthy_count_by_group_and_reason`, `camera_age_s` distribution,
`lane_fit_policy`, and `limits`.

Each metric function declares only the health groups it needs (`orient` and
`phase_anchors` need `lane` and `speed`; `settling`, `cycles`,
`hold_late_wide` and `path_quality` need `lane` only; `comfort_proxy` needs
`yaw` and `speed`; `annotations` needs none). A row unhealthy for one group
remains usable for a metric that does not declare that group. Missing
`command`, `controls`, `roll` or `accel` data never changes an anchor or a
lane-motion metric.

## Phase anchoring and the entry-speed freeze

`phase_anchors(rows, window, params)` is a geometry anchor derived from the
model's fitted road curvature; no command, request, steering or controller
signal touches it. Entry is the first instant where oriented, smoothed
`a_entry = curvature10 * speed^2` stays at or above `entry_accel_mps2` (1.0)
for `anchor_persistence_s` (0.5 s). The speed at that instant, `v_entry`, is
then frozen, and every later anchor uses `a_road = curvature10 * v_entry^2` —
so a vehicle slowing on unchanged road curvature cannot fake an unwind.
Peak is the maximum smoothed `a_road` after entry. Unwind is the first
instant after the peak where `a_road` stays at or below `exit_accel_mps2`
(0.5) for `anchor_persistence_s`. Recovery is
`[unwind, min(unwind + recovery_horizon_s, window_end))`
(`recovery_horizon_s` is 8.0 in `DEFAULT_PARAMETERS`; see the discrepancy
noted below). The anchor record names `anchor_source: model_road_geometry`,
`v_entry`, each anchor's `mono_ns`, inter-anchor coverage, whether a gap over
`max_gap_s` in `lane`-healthy rows touches an anchor or its persistence
interval (`critical_gap`), absent phases, and a diagnostic for where the
*instantaneous* (unfrozen-speed) crossing would have occurred, so
speed-driven divergence stays visible. Rider or geographic windows only
bound the search; they are never themselves anchors.

## Metric records and statuses

Every metric record carries: `metric`, `version`, `value`, `unit`,
`valid_duration_s`, `required_duration_s`, `maximum_gap_s`, `critical_gap`,
`phase_coverage`, `status`, `lower_bound_s` (when censored or unresolved),
`reason`, `uncertainty_method`, `supporting_event_ids`, and `diagnostics`.
`status` is one of:

- `measured_estimate` — a value with confirmed support.
- `right_censored` — observation ended before the outcome was confirmed
  (a lower bound is reported).
- `unresolved` — a gap or missing evidence prevents a conclusion (a lower
  bound is reported where applicable).
- `unscorable` — the window itself cannot be scored (for example
  `no_sustained_bend`, `critical_gap`, `assisted_recovery`).

`settling` distinguishes `measured_estimate` (continuous observation from
unwind through a confirmed dwell), `right_censored` (recovery interval ends
before a dwell is confirmed), `unresolved` (a gap precedes any confirmed
dwell; any later observed dwell is retained only as `later_dwell_observation`
and does not establish the first settling time), `unscorable: critical_gap`
(a gap touches the unwind anchor), and `unscorable: assisted_recovery` (an
intervention overlaps recovery). `cycles` and `hold_late_wide` report zero
observed events as `measured_estimate` only when the whole window was
observed apart from filter-edge loss; otherwise they are `unresolved` with
`critical_gap` set, because missing data could contain the entire symptom.
`annotations` reports a zero count as a measurement only when the case's
labels explicitly declare that event absent over a stated scope; an empty
annotation list alone is `unresolved`.

## Frozen signal processing

- No resampling; actual row timestamps.
- Smoothing: centred boxcar over `[t - h, t + h]`, `h = smooth_span_s / 2`,
  as the time-weighted integral of the left-constant signal divided by `2h`.
  A smoothed value exists only with bracketing healthy samples at or beyond
  `t - h` and `t + h` and no consecutive gap over `max_gap_s` between them.
- Derivative: difference of smoothed values nearest `t - d` and `t + d`
  (`d = velocity_span_s / 2`) over their actual separation; `null` if either
  endpoint is missing, unsupported, too far from its target, or gapped.
- Extrema: sign changes of the derivative; a flat run with
  `|de/dt| <= plateau_deadband_mps` is one extremum at its midpoint only
  when the surrounding derivative signs are opposite (otherwise a shoulder,
  not an extremum). Prominence is the offset difference to the previous
  opposite extremum.
- Cycle: three consecutive alternating extrema, each with prominence at
  least `a_min_m` and adjacent spacing at least `t_min_s`; a quiet interval
  longer than `episode_quiet_s` terminates a chain. `cycle_count` sums
  `floor((n - 1) / 2)` per maximal qualifying chain (episode); a
  two-extrema chain is a `truncated_cycle`, reported separately and never
  counted.
- Gaps: a gap over `max_gap_s` in a metric's healthy rows splits the window
  into observation segments; smoothing, derivatives, extrema, cycle chains
  and dwell checks never cross a segment boundary.
- Coverage: a value holds only until the next sample within `max_gap_s`; the
  final sample holds nothing. Every record reports `valid_duration_s`,
  `required_duration_s`, `maximum_gap_s`, and leading/trailing unobserved
  spans.

## Calibration eligibility rules

`calibrate` runs extraction and `phase_anchors` on development cases whose
labels declare smooth unwinding and no scalloping (the three smooth
controls: `24d_smooth_vw`, `261_smooth_vw`, `280_seg12_positive`).
Eligibility is decided per parameter and every smooth case appears in the
output with its eligibility and reason:

| Parameter | Eligible case requires | Derivation over eligible cases |
| --- | --- | --- |
| `e_settle_m`, `v_settle_mps` | unwind anchor present without `critical_gap`; at least `min_recovery_support_s` of continuous `lane`-healthy recovery observation | max of the elapsed-time p95 `\|e\|` (and `\|de/dt\|`) over that continuous recovery observation, rounded up to 0.01 |
| `t_settle_max_s` | same as above, and the case reaches a qualifying dwell within its continuous observation | max observed `settling_time_s` times 1.5, rounded up to 0.1 s |
| `a_min_m` | at least `min_recovery_support_s` of continuous `lane`-healthy rows anywhere in the window | three times the max over eligible cases of the median absolute smoothed-offset residual about a 2 s moving mean, rounded up to 0.01 |

A parameter is derived only when at least `min_eligible_cases` (2) cases are
eligible for it; otherwise `calibrate` fails for that parameter and names
the shortfall in one `ValueError` covering every parameter, carrying the
full derivation table as `err.derivation`. `min_eligible_cases` (2) and
`min_recovery_support_s` (4.0 s) are declared constants and are never
lowered to reach an outcome.

## The report's three outcomes and the sealed holdout

`evaluate` extracts and measures every case. Development results go to
`O/result.json` and `O/report.md`, with per case: role, labels and contact
status, extraction health summary, orientation, anchors, every metric
record, and physical-evidence verdicts. Holdout results — including their
extraction summaries, anchors and every metric record — go only to
`O/holdout-sealed.json`; `result.json` and `report.md` carry solely the
holdout case ids, the sealed file's SHA-256, and the case count. Holdout
cases carry no labels, annotations, or non-`unknown` `contact_status`; the
runner refuses a request that attaches any outcome-bearing field to a
holdout case.

`false_alarms`, `symptom_detection`, and `disagreements` (rider label vs.
metric family) are each scored as one of `detected`, `not_detected`,
`unresolved` per family/label — never a two-valued pass/fail. `unscorable`
families are always `unresolved`. `validation_status` is `unresolved` with
the sealed holdout hash and reason (no independent labels), because the 22
holdout traversals are archive logs with no independent video or rider
review to compare against.

## Real-data calibration attempt (Task 9, 2026-09-10) — failed on data health

A real request was built and calibration was attempted three times against
retained artifacts. **No `params-v1.json`, no `calibration.json`, no
`evaluation/` directory, no `params_id`, no result hash, and no false-alarm,
detection or 28c-disagreement tables exist** — `evaluate` requires
`--params` and none was ever produced. Everything below is transcribed
verbatim from
`.ti-local/observed-drive-metrics-20260910/{request-v3.json,calibration-failure-3.json,calibration-failure-3.correction.md,preflight.json}`.

**Request.** `request-v3.json` (sha256
`2e301667d1bc4228e944881ba2c00a3635817c08f98186cbc557f92bac2773c4`): 33
cases (11 development, 22 holdout); 38 distinct rlog segments referenced,
all 38 retained-hash verified against current bytes (zero placeholder
hashes, zero unavailable segments, zero mismatches). `preflight.json`
reports `status: "ready"`, one volume, `free_bytes: 809650810880`,
`required_bytes: 9000000000`, `sufficient: true`.

**Three attempts, three window rulings, same shortfall pattern:**

1. **Attempt 1** — the predeclared `lateral-sensor-windows-v1.json` bounds
   for the two smooth controls are 5.0 s (`24d_smooth_vw`) and 5.5 s
   (`261_smooth_vw`); both end while the vehicle is still in the bend, so
   there is no unwind anchor and no recovery phase inside either window
   (100% lane-healthy rows in both — health was never the problem).
2. **Attempt 2** — the controller ruled in the legacy
   `geographic_lookup_window_ns` bounds (23.30 s / 25.30 s). Both windows
   still end inside the bend: the road-geometry unwind occurs 1.65 s
   (`24d_smooth_vw`) and 1.50 s (`261_smooth_vw`) past the window end.
3. **Attempt 3** — the controller ruled a structural, outcome-free
   extension of the two windows' ends by 12.0 s
   (`RECOVERY_HORIZON_EXTENSION_NS`). This found both unwinds (at
   1118.714 s and 311.020 s respectively) and produced a real recovery
   observation, but calibration still failed — on lane-observation health
   inside the recovery, not window length.

**Attempt-3 derivation table** (per parameter, per case; `min_eligible_cases`
2, `min_recovery_support_s` 4.0, neither lowered):

| Parameter | Value | Eligible | Rule |
| --- | --- | --- | --- |
| `a_min_m` | **0.06** | 3 of 3 | three times the max over eligible cases of the median \|smoothed offset - 2 s moving mean\|, rounded up to 0.01 |
| `e_settle_m` | none (shortfall) | 0, need 2 | max over eligible cases of elapsed-time p95 \|e\| in continuous recovery observation, rounded up to 0.01 |
| `v_settle_mps` | none (shortfall) | 0, need 2 | max over eligible cases of elapsed-time p95 \|de/dt\| in continuous recovery observation, rounded up to 0.01 |
| `t_settle_max_s` | none (shortfall) | 0, need 2 | max over eligible cases of confirmed settling_time_s times 1.5, rounded up to 0.1 |

| case | eligible (recovery params) | reason | `e_settle_m` value | `v_settle_mps` value |
| --- | --- | --- | --- | --- |
| `24d_smooth_vw` | false | `recovery_support 2.10s < 4.0s` — a 2.70 s lane-confidence gap at 1120.813-1123.514 s splits 7.25 s of observed recovery into runs of 2.10 s, 0.40 s and 1.80 s | 0.46192026138305664 | 0.28049765874451627 |
| `261_smooth_vw` | false | `critical_gap_at_anchor` — a 0.200569 s observation gap at 310.669-310.870 s lies within `anchor_persistence_s` (0.5 s) of the unwind anchor at 311.020 s, against `max_gap_s` 0.2 s (569 microseconds over) | 0.43428295850753784 | 0.14197835821225757 |
| `280_seg12_positive` | false | `critical_gap_at_anchor` — an 11.301 s lane gap (2398.122-2409.422 s) ends 0.151 s before the unwind anchor; only 1.148 s of recovery (`280_seg12` recovery 1.15 s) exists before the window ends | 0.6283485293388367 | 0.5325926857201784 |

All three smooth controls are `eligible: true` for `a_min_m` (values
0.05955928310189552, 0.04527609050755922, 0.04179519782095335).

**Observations surfaced for the controller, not applied:**

- The settling family (`e_settle_m`, `v_settle_mps`, `t_settle_max_s`)
  cannot be calibrated from the three predeclared smooth controls as
  recorded.
- `max_gap_s` (0.2 s) equals exactly four modelV2 frame periods at 20 Hz, so
  a four-frame dropout passes or fails on sub-millisecond jitter (the
  `261_smooth_vw` shortfall is 569 microseconds over threshold). This was
  observed, not applied — no gap or persistence threshold was changed.
- The third ruling's plan text describes the extension as sized to the
  spec's `recovery_horizon_s`, given there as 12.0 s; the shipped
  `mm.DEFAULT_PARAMETERS.recovery_horizon_s` is 8.0. The 12.0 s window
  extension comfortably contains the 8.0 s horizon, so this did not change
  the outcome, but the plan text and the code disagree on one value.

**Status:** `validation_status: unresolved`; `calibration_status: failed
(data health)`. No parameter file, no `params_id`, and no evaluation output
exists for this attempt.

## Limits

Model-derived geometry (one camera-model lane fit, not surveyed truth);
latest-publication join (not exact consumed-input identity); TI counts are
command evidence only, not a physical clearance measurement;
`hold_late_wide`'s boundary-distance proxies are reference-point distances
without vehicle footprint; thresholds in `DEFAULT_PARAMETERS` are drafts
pending calibration; only the development cases already inspected have any
independent label at all. All cases are legacy logs (diagnostics version 0,
no exact consumed-input or applied-command identity, no motor-delivery
measurement) — this limits exact command attribution, not the validity of
recorded motion or a rider label.

## Validation status and what would resolve it

`validation_status` is `unresolved` and stays that way until either
labelled future drives collected under the matched-observation protocol, or
archive video for the sealed holdout traversals, supply an independent
label to compare against. The 22 holdout traversals in this evidence base
have no video and inherit no rider label; they can be measured but not
validated. A new `params_id` and a written reason are required for any
future threshold change — sealed holdout results are never relabelled, and
this real-data attempt did not produce a threshold to change.
