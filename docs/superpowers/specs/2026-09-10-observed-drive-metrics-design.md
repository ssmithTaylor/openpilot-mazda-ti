# Observed-drive symptom measurement: design

Date: 2026-09-10. Scope: assessment infrastructure. This work measures recorded
vehicle motion on already supplied drives to qualify the evaluator's physical
metrics. It does not assess, rank, revive or retire controller candidates, does
not produce a physical pass/fail threshold, and does not authorize deployment.

Specifications this design implements: metric definitions and calibration
procedure in `docs/mazda-road-performance-action-plan.md`; the physical
evidence boundary and "Use the good drives already provided" in
`docs/mazda-physical-evidence.md`; work packages 5 and 6 in
`docs/mazda-assessment-remediation.md`; and the P1 geometry/labels/uncertainty
defect in `docs/mazda-idea-reassessment.md`. Those documents remain the
specifications; this design does not replace them.

## Decisions taken with the user

| Decision | Choice |
| --- | --- |
| Metric families in this iteration | All lane-motion families: settling after unwind, lane-motion cycles (scallop severity), inward-hold / late-wide episodes, routine path quality, lateral comfort proxy. Containment and catches are carried only from explicit rider annotations. Footprint clearance stays unqualified. |
| Phase anchors | Road-curvature anchors derived from the model's fitted road geometry inside the existing predeclared rider/geographic/camera windows. No command, request, steering or controller signal touches an anchor. |
| Validation | Calibrate on the inspected development cases. Run the 22 unlabelled archive traversals outcome-blind and seal their results. Validation status stays `unresolved` until those traversals or future labelled drives supply independent labels. Archive traversals have no video, so future drives collected under the matched-observation protocol are the realistic validation set. |
| Structure | Three shared modules under `tools/mazda_ti/`: extractor, pure metric library, two-stage qualification runner. |

## Evidence base

Development cases and their retained label sources:

| Case id | Route / segments | Window source | Retained labels |
| --- | --- | --- | --- |
| `24d_smooth_vw` | `0000024d--8deb0716b5` 14-16 | `lateral-sensor-windows-v1.json` bounds `[1100765371121, 1105765371121]` ns | no scalloping, smooth timely unwind, left lane; contact unknown |
| `261_smooth_vw` | `00000261--7120d4c827` 1-3 | same file, `[291721355775, 297221355775]` ns | same as 24d; contact unknown |
| `280_seg12_positive` | `00000280--c49cd43d5b` 11-12 | `route280-positive-control-v1.json` `[2387.933750338, 2411.034288298]` s | smooth, went a bit wide in a very wide lane; retained rows have an 11.3 s gap |
| `280_seg9_settling` | `00000280--c49cd43d5b` 9 | clip 32-44 s from segment start `2206.752201033` s | held inward, over-release, no evident short scallop (low confidence), contact not remembered |
| `28c_preceding_hold`, `28c_cycle1`, `28c_cycle2`, `28c_main` | `0000028c--0ea8954ea3` 3-4 | clip windows 16-18, 46.5-49.5, 53.5-57.5, 44-61 s; camera-EOF bounds retained in `phase_normalized_multisensor.py` | scallop, held inward, over-release (medium); contact uncertain |
| `28a_seg47`, `28a_seg48` | `0000028a--601b728d66` 47, 48 | `curve-review-20260907/rider-event-annotations.json`; interventions near mono 2879 s and 2934 s | oversteer complaint with intervention; contact end unknown |
| `28f_seg13_crossing` | `0000028f--1fd872c052` 13 | `post-drive-28f/rider-annotations-v2.json`; clip 850.0145-875 s | confirmed boundary crossing 850-852 s with no intervention; interventions near 859 s and 868 s |

Raw roots: `C:/Users/Taylor/Downloads/TI-analysis/raw` for 24d, 261, 280;
`.ti-local/curve-review-20260907/rlogs` for 28a, 28c, 28f. Every request entry
carries expected SHA-256 for each rlog; a mismatch rejects the case.

Holdout traversals: the 22 accepted same-direction traversals in
`vw-factory-pass-inventory-20260910-v2.json` whose route is not 24d, 261 or
280 (`historic-05` through `historic-22`, `historic-25` through `historic-28`).
Their windows are the inventory's accepted windows; raw hashes come from
`vw-factory-pass-raw-sha256-20260910.json`. They inherit no rider label.

All cases are legacy logs: diagnostics version 0, no exact consumed-input or
applied-command identity, no motor-delivery measurement. This limits exact
command attribution, not the validity of the recorded motion or rider label.

## Module 1: `tools/mazda_ti/observed_drive_rows.py`

Purpose: turn explicit rlogs into one hash-bound, health-annotated row stream
at model publication times.

Inputs: `--data-root`, repeated `--rlog` relative paths with `--sha256`
expectations, `--start-ns`, `--end-ns` (integer monotonic nanoseconds,
`start < end`), `--margin-ns` (rows read outside the window to support
centred derivatives; default 1 s), `--output` (new file; existing output is
refused).

Reader: `tools.mazda_ti.audit_diagnostics.read_events`. The production lane
fit `selfdrive/car/mazda/lateral_reference.py::fit_lane_context` is loaded by
`importlib` from its path and its hash recorded; production code is not
modified.

Join rule: for each `modelV2` publication with time `t`, the latest preceding
`carState`, `liveLocationKalman`, `liveParameters`, torque-state
`controlsState` and 0x249 `sendcan` frame. Each source's age is recorded.

Health is recorded per field group, never as one shared gate. Each row
carries a `health` object with a boolean and a reason for each group:

| Group | Healthy when |
| --- | --- |
| `lane` | model event valid; production fit returns `valid` (its own policy: lane-change state off, both adjacent lane probabilities at least 0.8, width 2.5-5.0 m, fit support and agreement); camera age `t - timestampEof` within `[0, 0.2]` s |
| `speed` | carState exists, valid, CAN valid, age at most 0.2 s |
| `steering` | same carState conditions (angle and rate come from the same event) |
| `yaw` | localizer exists, valid, calibrated yaw valid, sensors/inputs/posenet OK, age at most 0.2 s |
| `accel` | localizer conditions plus calibrated acceleration valid |
| `roll` | liveParameters exists, valid, age at most 0.2 s |
| `controls` | torque-state controlsState exists, valid, age at most 0.2 s; `controls_active` recorded separately |
| `command` | 0x249 frame exists, age at most 0.2 s |

No row is dropped and no missing value becomes zero; a missing source leaves
its fields `null` with the group unhealthy. The extractor does not apply a
separate lane-probability threshold: the production fit's 0.8 is the
effective policy and is recorded in the output as
`lane_fit_policy`. Camera freshness is enforced explicitly by the extractor,
not by the fit. Clock alignment: `timestampEof` is camerad's boot-time clock
and `logMonoTime` is the monotonic clock; the extractor assumes no suspend
between them (the same assumption the production `LaneObserver` documents),
records the observed camera-age distribution per case, and rejects negative
or over-limit ages as unhealthy `lane` rows rather than correcting them.

Row fields (right-positive unless stated; no sign normalization here):
`mono_ns`, `camera_eof_ns`, `camera_age_s`, `frame_id`, `health`,
`lane_change_state`, `lane_fit_reason`, `lane_probability_min`,
`lane_offset_m`, `lane_heading10_rad`, `lane_heading20_rad`,
`road_curvature10_per_m`, `road_curvature20_per_m`, `lane_width_m`
(interpolated left-right separation at x = 0, computed in the extractor),
`speed_mps`, `steering_angle_deg` (sign flipped from `steeringAngleDeg` to
right-positive, as prior studies did), `steering_rate_dps`, `yaw_rate_rps`,
`yaw_std_rps`, `yaw_lateral_accel_mps2` (yaw rate times speed),
`device_lateral_accel_mps2`, `roll_rad`, `controls_active`,
`request_mps2`, `actual_mps2`, `diag_version`, `ti_command_counts`,
`ages_ns` per source.

Output JSON: `format_version`, `scope`, request echo, `source_sha256`
(this module, `lateral_reference.py`, `audit_diagnostics.py`, `provenance.py`,
`cereal/*.capnp`), `runtime` from `provenance.environment()`, `raw_sha256`,
`initdata` settings snapshot per segment using the existing whitelist,
`window_ns`, `rows`, `unhealthy_count_by_group_and_reason`,
`camera_age_s` distribution, `lane_fit_policy`, and `limits`.

## Module 2: `tools/mazda_ti/maneuver_metrics.py`

Purpose: pure functions from rows plus frozen parameters to structured metric
records. No file I/O, no hashing, no randomness.

### Parameters

`Parameters` is a frozen dataclass with `params_id` (string version, e.g.
`observed-drive-v1-draft`) and, in declared units:

| Group | Fields | v1 draft policy |
| --- | --- | --- |
| Sampling | `max_gap_s` 0.2, `smooth_span_s` 0.25, `velocity_span_s` 0.5, `plateau_deadband_mps` 0.02 | declared constants |
| Anchors | `entry_accel_mps2` 1.0, `exit_accel_mps2` 0.5, `anchor_persistence_s` 0.5, `recovery_horizon_s` 8.0 | declared constants; road lateral acceleration uses curvature10 times the entry speed squared (speed frozen at entry) |
| Settling | `e_settle_m`, `v_settle_mps`, `t_dwell_s` 1.0, `t_settle_max_s` | `e_settle_m`, `v_settle_mps`, `t_settle_max_s` derived by `calibrate`; `t_dwell_s` declared |
| Cycles | `a_min_m`, `t_min_s` 0.4, `episode_quiet_s` 2.0 | `a_min_m` derived by `calibrate`; `t_min_s` and `episode_quiet_s` declared |
| Absence | none (derived `absence_edge_s = smooth_span_s + velocity_span_s`) | a metric may assert absence of a symptom only when its relevant interval has no internal gap over `max_gap_s` and its leading/trailing unobserved spans do not exceed the filter edge |
| Calibration | `min_eligible_cases` 2, `min_recovery_support_s` 4.0 | declared constants |
| Corridor | `corridor_half_width_m` | equals `e_settle_m` in v1; recorded as a draft driver corridor, not an accepted buffer |
| Comfort | `jerk_smooth_span_s` 0.25, `j_limit_mps3` null | proxy only; no limit in v1 |

Every parameter value is written into every output. Changing any value
changes `params_id`.

### Per-metric input requirements

Each function declares the health groups it needs and uses only rows where
those groups are healthy. Availability is therefore specific to each metric:

| Function | Required groups |
| --- | --- |
| `orient`, `phase_anchors` | `lane`, `speed` |
| `settling`, `cycles`, `hold_late_wide`, `path_quality` | `lane` only; offset rates come from distance and time |
| `comfort_proxy` | `yaw`, `speed` |
| `annotations` | none (request-supplied) |
| command diagnostics (reversal counts) | `command` |

Missing `command`, `controls`, `roll` or `accel` data never changes an
anchor or a lane-motion metric. A row unhealthy for one group remains usable
for metrics that do not need it. Gap handling is evaluated per metric on
the rows healthy for that metric's groups.

### Signal processing (frozen before calibration)

- Time base: actual row timestamps; no resampling to a fixed grid.
- Smoothing: centred boxcar over `[t - h, t + h]` with
  `h = smooth_span_s / 2`. The value is the time-weighted integral of the
  left-constant signal over that interval, using sample intervals clipped to
  the interval, divided by `2h`. Support is checked with bracketing samples:
  a smoothed value exists only when a healthy sample exists at or before
  `t - h` and another at or after `t + h`, and no consecutive pair of
  healthy samples between those brackets is separated by more than
  `max_gap_s`; otherwise the value is `null`. At 20 Hz with no gaps this
  yields a value at every interior row. Window edges lose `h` of support
  unless margin rows exist.
- Derivative: the difference of the smoothed values at the samples nearest
  `t - d` and `t + d` (`d = velocity_span_s / 2`), divided by the actual
  separation of those two samples; `null` if either endpoint is missing,
  unsupported, further than `max_gap_s` from its target, or separated from
  the other by a gap. Using the actual separation keeps the estimate stable
  under timestamp jitter.
- Extrema: sign changes of the derivative. A flat run where
  `|de/dt| <= plateau_deadband_mps` counts as one extremum at the run
  midpoint only when the derivative signs immediately before and after the
  run are opposite; a run between same-sign derivatives is a shoulder and
  is not an extremum. Prominence of an extremum is
  `|e_extremum - e_previous_opposite_extremum|`; the first extremum's
  prominence uses the first valid smoothed value.
- Cycle: three consecutive alternating extrema (peak-trough-peak or
  trough-peak-trough) each with prominence at least `a_min_m` and adjacent
  spacing at least `t_min_s`. A quiet interval is an actual plateau
  (derivative within the deadband) lasting longer than `episode_quiet_s`;
  its extremum is a terminator that joins no chain, so a long flat stretch
  cannot connect two separate episodes while a slow continuous oscillation
  still counts. Every maximal alternating chain of qualifying
  extrema with at least three members is an `episode` with
  `floor((n - 1) / 2)` cycles. `cycle_count` is the sum over episodes;
  `episode_count`, per-episode cycles, duration and peak-to-peak, and
  `longest_episode_cycles` are reported separately. A chain with exactly
  two qualifying extrema is a `truncated_cycle`, reported separately and
  never counted.
- Gaps: a gap over `max_gap_s` in a metric's healthy rows splits the
  window into observation segments; smoothing, derivatives, extrema, cycle
  chains and dwell checks never span a segment boundary. Extrema carry their
  segment identity so that no consumer can chain across a gap.
- Coverage: a value is held only until the next sample within `max_gap_s`;
  the final sample holds nothing, so observation is never fabricated beyond
  the last supported interval. Every record reports `valid_duration_s`,
  `required_duration_s`, `maximum_gap_s`, and the leading and trailing
  unobserved spans of its window.

### Orientation

`orient(rows, window)` returns the direction multiplier from the sign of the
elapsed-time integral of `road_curvature10_per_m` over `lane`-healthy rows in
the window, so a bend need not fill the window. Positive oriented values point
toward the inside of the sustained bend. It multiplies `lane_offset_m`,
headings, curvatures, steering, yaw and TI fields. It rejects windows whose
mean curvature magnitude is below `1e-4` per metre (no sustained bend) with
`unscorable: no_sustained_bend`.

### Phase anchors

`phase_anchors(rows, window, params)` is a geometry anchor. Entry is found
on `a_entry = curvature10 * speed^2` (oriented, smoothed): the first instant
where it stays at or above `entry_accel_mps2` for `anchor_persistence_s`.
The speed at entry, `v_entry`, is then frozen and every later anchor uses
`a_road = curvature10 * v_entry^2`, so a slowing vehicle on unchanged road
curvature cannot create an unwind anchor. Peak is the maximum of smoothed
`a_road` after entry. Unwind is the first instant after the peak where
`a_road` stays at or below `exit_accel_mps2` for `anchor_persistence_s`.
Recovery is `[unwind, min(unwind + recovery_horizon_s, window_end))`.
The record names `anchor_source: model_road_geometry`, `v_entry`, each
anchor's `mono_ns`, coverage between anchors, any gap over `max_gap_s` in
`lane`-healthy rows that touches an anchor or its persistence interval
(`critical_gap`), and lists absent phases. A diagnostic field reports where
instantaneous `curvature10 * speed^2` would have crossed `exit_accel_mps2`
earlier, so speed-driven divergence is visible. Rider or geographic windows
only bound the search.

### Metric functions

All functions accept oriented rows, the window, the anchors record and
parameters. They use only rows healthy for their declared groups, weight by
elapsed time, hold values constant between samples, and never bridge a gap
longer than `max_gap_s`.
Every record has: `metric`, `version`, `value`, `unit`, `valid_duration_s`,
`required_duration_s`, `maximum_gap_s`, `critical_gap`, `phase_coverage`,
`status` (`measured_estimate`, `right_censored`, `unresolved`, `unscorable`),
`lower_bound_s` where censored or unresolved, `reason`,
`uncertainty_method` (`not_established_v1` plus a parameter-sensitivity note
where computed), `supporting_event_ids`, and `diagnostics`.

- `settling(...)`: settling time is the elapsed time from the unwind
  anchor to the *start* of the first dwell, where a dwell is an interval of
  at least `t_dwell_s` throughout which `|e| <= e_settle_m` and
  `|de/dt| <= v_settle_mps` are confirmed on valid values. Establishing a
  first settling time requires continuous observation (`lane`-healthy rows,
  no gap over `max_gap_s`, valid smoothed and derivative values) from the
  unwind anchor through the end of that dwell. Smoothing and derivatives
  are computed on all lane-healthy rows including margin rows beyond the
  recovery interval, so support is never discarded at the recovery
  boundary; only dwell scoring is restricted to the recovery interval.
  Bounds distinguish dwell start from dwell confirmation: when no dwell is confirmed,
  `lower_bound_s` is the elapsed time from the unwind anchor to the last
  instant at which a settling condition was *confirmed violated* on valid
  values. An unfinished candidate dwell in progress at the end of
  observation, or a tail where derivative support is `null`, therefore does
  not raise the bound. Outcomes:
  `measured_estimate` with `settling_time_s`, `last_overshoot_m`,
  `residual_offset_m` when continuity holds and a dwell is confirmed;
  `right_censored` with `lower_bound_s` and, if present,
  `candidate_dwell_start_s`, when the recovery interval ends before a dwell
  is confirmed;
  `unresolved` with `lower_bound_s` (last confirmed violation before the
  first gap) when a gap precedes any confirmed dwell, retaining any later
  observed dwell as `later_dwell_observation` (start time and residual)
  that does not establish the first settling time;
  `unscorable: critical_gap` when a gap touches the unwind anchor;
  `unscorable: assisted_recovery` when an intervention interval overlaps
  the recovery interval, where an intervention with unknown end is treated
  as continuing (so one that starts before the unwind anchor still counts),
  with the pre-intervention portion reported as diagnostics only.
- `cycles(...)`: extrema and cycles as defined under signal processing,
  per observation segment. Observed cycles are evidence at any coverage, but
  zero cycles is `measured_estimate` only when the whole window was observed
  apart from filter-edge loss (no internal gap over `max_gap_s`); otherwise
  the record is `unresolved` with `critical_gap` set and the observed count
  retained as a diagnostic, because missing data could contain the entire
  symptom. The same rule applies to the inward-hold/late-wide record over
  its entry-to-recovery interval.
  Reports `cycle_count`, `truncated_cycles`,
  `peak_to_peak_m` (max and per cycle), `episode_duration_s`,
  `amplitude_ratio` of successive peak-to-peaks, and the extrema list.
  Command sign reversals are a separate diagnostic (requires `command`)
  and never enter `cycle_count`.
- `hold_late_wide(...)`: within sustained-plus-recovery, `inward_peak_m`
  (max positive `e`), `inward_dwell_s` beyond `corridor_half_width_m`,
  `outward_peak_m` (min `e` after the inward peak), `time_to_recovery_s`
  (from unwind anchor to sustained corridor re-entry, right-censored if
  absent), and boundary-distance proxies computed before orientation from
  the right-positive offset `e_r` as `left_m = lane_width_m / 2 + e_r` and
  `right_m = lane_width_m / 2 - e_r`, then mapped to `inside_m` and
  `outside_m` by the bend direction. These are reference-point distances
  without vehicle footprint and are labelled as such.
- `path_quality(...)`: elapsed-time RMS and 95th percentile of `|e|`, and
  time outside the corridor, over the window and separately over recovery.
  No re-centering.
- `comfort_proxy(...)`: jerk proxy as the centred derivative of
  `yaw_lateral_accel_mps2` after `jerk_smooth_span_s` smoothing; margin rows
  supply filter support only, and peak and percentile are taken over
  supported in-window values; reports
  peak and elapsed-time 95th percentile of `|j|`, duration above
  `j_limit_mps3` when a limit is declared, and the declared basis (fused
  localizer yaw times speed, 20 Hz model-rate join, smoothing, derivative
  span). Labelled `kinematic_proxy`, not occupant acceleration.
- `annotations(...)`: carries rider-supplied `crossing` and `intervention`
  intervals from the request into records `boundary_crossings` and
  `driver_catches` with `provenance: rider_annotation` and confidence. A
  count of zero is a measurement only when the case's labels explicitly
  declare that event `absent` over a stated scope; an empty annotation list
  alone yields `unresolved`. Never inferred from TI, steering or geometry.

### Physical-evidence bridge

`to_physical_records(records, identity)` maps to the names in
`physical_evidence.METRIC_UNITS` where one exists (`settling_time`,
`scallop_peak_to_peak`, `late_wide_excursion`, `lateral_jerk_p95`,
`driver_catches`, `corridor_violation_duration`), passing contact status and
completeness so that `classify_metrics` can reject them for the documented
reasons. Clearance names are supplied as absent. The runner records both the
raw measurement record and the boundary's verdict.

## Module 3: `tools/mazda_ti/measurement_qualification.py`

Purpose: run extraction and metrics over a declared case set and report
measurement qualification, never a handling verdict.

Request JSON (`format_version` 1): `data_roots` (named absolute roots),
`cases` list. Each case: `id`, `role` (`development` or `holdout`),
`route`, `data_root`, `rlogs` with `sha256`, `window_ns` (integers),
`window_source` (text and source file hash), optional `labels`
(`scalloping`, `settling`, `held_inward`, `over_release`, `crossing`,
`intervention`; each with value, confidence, provenance, scope), optional
`annotations` (crossing/intervention intervals in ns with provenance), and
`contact_status` (`confirmed_no_intervention`, `confirmed_intervention`,
`unknown`). Holdout cases must carry no `labels`, no `annotations`, and no
`contact_status` other than `unknown`; the runner refuses a request that
attaches any outcome-bearing field to a holdout case.

Stages:

1. `calibrate --request R --params-out P --output O`: runs extraction and
   `phase_anchors` on development cases whose labels declare smooth
   unwinding and no scalloping. Eligibility is decided per parameter, and
   every smooth case appears in the output with its eligibility and reason:

   | Parameter | Eligible case requires | Derivation over eligible cases |
   | --- | --- | --- |
   | `e_settle_m`, `v_settle_mps` | unwind anchor present without `critical_gap`; at least `min_recovery_support_s` of continuous `lane`-healthy recovery observation | maximum of the elapsed-time 95th percentile of `|e|` (and `|de/dt|`) over that continuous recovery observation, rounded up to 0.01 |
   | `t_settle_max_s` | same as above, and the case reaches a qualifying dwell within its continuous observation | maximum observed `settling_time_s` times 1.5, rounded up to 0.1 s |
   | `a_min_m` | at least `min_recovery_support_s` of continuous `lane`-healthy rows anywhere in the window | three times the maximum of the median absolute smoothed-offset residual about a 2 s moving mean, rounded up to 0.01 |

   A parameter is derived only when at least `min_eligible_cases` cases are
   eligible for it; otherwise `calibrate` fails for that parameter and names
   the shortfall. Excluded cases stay in the denominator with their reason.
   The derivation rule, input values, and the parameter file with a new
   `params_id` are written.
2. `evaluate --request R --params P --output O`: extraction kept in memory,
   metrics on every case. Development results go to
   `O/result.json` and `O/report.md`. Holdout results, including their
   extraction summaries, anchors and every metric record, go only to
   `O/holdout-sealed.json`; `result.json` and `report.md` carry solely the
   holdout case ids, the sealed file's SHA-256 and the case count.

Decisions from the 2026-09-10 real-data attempt review, frozen for the next
run:

- The window amendments made in attempts 2 and 3 for `24d_smooth_vw` and
  `261_smooth_vw` are exploratory development amendments made after
  failures, not the original predeclared protocol; they keep their
  diagnostic value. Window protocol v2, frozen for the next attempt: window
  start is the predeclared `geographic_lookup_window_ns` start; window end
  is the predeclared geographic end plus a fixed 12.0 s extension. The
  request's `window_source` label "recovery_horizon_s (12.0 s)" is a
  mislabel; `recovery_horizon_s` is 8.0 s. All three calibration failure
  records are preserved; the sealed holdout is untouched and must stay
  untouched.
- `max_gap_s` stays 0.2 s for this protocol; any jitter allowance requires a
  separately justified, versioned rule with a new `params_id`.
  Timestamp-gap semantics: a gap is the difference between the monotonic
  `mono_ns` timestamps of two consecutive rows in the group-healthy row
  list under test (lane rows for lane metrics; yaw+speed rows for the
  comfort proxy); it exceeds the limit when that difference is greater than
  `max_gap_s` in nanoseconds. Four frame periods are not the same as four
  missing frames: a row is also removed by an unhealthy group, and the
  comparison is on the actual timestamp difference, not a frame count.
- Precondition for supporting a parameter file with changed gap or filter
  settings: `orient()` must take its gap limit from the parameters in use
  rather than `DEFAULT_MAX_GAP_S`, and `case_rows()` must size the
  extraction margin from the parameters in use rather than
  `DEFAULT_PARAMETERS`. Until then only the default `max_gap_s`,
  `smooth_span_s` and `velocity_span_s` are supported.
- `settling`: `unresolved` with reason `gap_before_first_dwell` now carries
  `critical_gap` true; a gap after an already-confirmed dwell does not
  invalidate a measured settling time.

Report content (development cases only):

- Per case: role, labels and contact status, extraction health summary by
  group, orientation, anchors, every metric record, physical-evidence
  verdicts.
- `false_alarms`: per labelled smooth maneuver, for the families `settling`,
  `cycles` and `hold_late_wide_sequence`, one of `detected`, `not_detected`,
  `unresolved`. Detection rules: cycles, `cycle_count >= 1`; `held_inward`,
  `inward_dwell_s > t_dwell_s`; `over_release`, outward excursion beyond
  `corridor_half_width_m` after the inward peak; `hold_late_wide_sequence`,
  both (a modest inside offset alone is not the complaint sequence, so only
  the sequence counts as a smooth-pass alarm while each symptom label is
  scored by its own detector); settling,
  `settling_time_s > t_settle_max_s`, or a `right_censored`/`unresolved`
  observation whose `lower_bound_s` already exceeds `t_settle_max_s`. A
  censored or unresolved observation whose lower bound does not exceed the
  threshold is `unresolved`, never a false alarm. `unscorable` families are
  `unresolved`.
- `symptom_detection`: per complaint case and per label in its scope, the
  same three outcomes, with the metric values.
- `disagreements`: cases where a rider label and a metric family disagree,
  listed as calibration gaps; 28c cycle windows are expected here.
- `unscorable`: every case-family pair with its reason.
- `validation_status`: `unresolved` with the sealed holdout hash and the
  reason (no independent labels).
- `limits`: model-derived geometry, latest-publication join, TI as command
  evidence, proxy boundary distance without footprint, draft thresholds,
  development cases already inspected.

Every output carries the request hash, parameter file hash, runtime
identity, raw hashes rechecked after execution, and the hashes of every
measurement dependency (`observed_drive_rows.py`, `maneuver_metrics.py`,
`measurement_qualification.py`, `physical_evidence.py`, the production lane
fit, the reader, provenance helpers and the capnp schemas) taken before
execution and verified unchanged afterwards. Calibration retains each case's
extractor provenance beside its statistics.

The request builder takes expected raw hashes only from retained study
artifacts and verifies current bytes against them, per case and per exact
required segment. A case with a missing or changed segment is not dropped:
it carries `input_status.ok = false` with per-segment problems, is never
extracted, and appears in every stage and table as `unscorable:
input_unavailable`, counted in the report's coverage totals. The builder
never adopts current bytes as a new expectation.

## Tests

`tools/mazda_ti/test_maneuver_metrics.py` (synthetic rows, no rlogs):
resampling a trace at 10, 20 and 50 Hz gives equal durations, cycle counts
and RMS within declared tolerance; irregular timestamps give the same result
as regular ones within tolerance; a gap over `max_gap_s` splits episodes and
marks the touching metric `critical_gap`; a gap inside smoothing or
derivative support nulls those values rather than bridging them; a gap
before the first dwell gives `unresolved` with a lower bound and retains the
later dwell; a recovery that never dwells is `right_censored` with a lower
bound, not settled; an intervention in recovery gives `assisted_recovery`;
missing lane geometry gives `unscorable`, never zero; deleting the
`command`, `controls`, `roll` and `accel` streams leaves anchors and every
lane-motion record byte-identical; a flat extremum counts once at its
midpoint; two qualifying extrema are a truncated cycle with
`cycle_count == 0`; peak-trough-peak is one cycle; two separate one-cycle
episodes give `cycle_count == 2`, `episode_count == 2` and
`longest_episode_cycles == 1`; a plateau between opposite derivative signs
is one extremum and a plateau between same-sign derivatives is none; a
regular 20 Hz gap-free trace yields a smoothed value at every interior row;
a censored trace with the settling conditions satisfied but unconfirmed at
the end reports a lower bound at the last confirmed violation, not at the
observation end; two smooth traces with
different command reversal counts both report zero cycles; a low-RMS trace
with a declared crossing still reports the crossing; a trace whose speed
falls while curvature stays constant produces no unwind anchor and reports
the instantaneous-crossing diagnostic; orientation flips all signed fields
consistently for a left bend and a right bend, and the inside/outside
boundary proxies map correctly for both (4 m lane, right-positive
offset +0.5 m gives left 2.5 m, right 1.5 m); `to_physical_records` with
unknown contact yields `not_measured` from `classify_metrics`.

`tools/mazda_ti/test_observed_drive_rows.py`: a generated capnp rlog with
known modelV2, carState, localizer, params, controlsState and 0x249 frames
produces the expected joined rows, per-group health reasons and lane width;
a row with lane probability 0.75 is `lane`-unhealthy under the production
fit's 0.8 policy; a stale camera EOF is `lane`-unhealthy; a missing 0x249
frame leaves only `command` unhealthy; a changed raw hash is refused; an
existing output is refused.

`tools/mazda_ti/test_measurement_qualification.py`: request validation
(holdout with labels, annotations or non-unknown contact refused; missing
hash refused; bad window refused); `calibrate` derivation from synthetic
smooth cases including one excluded for short recovery support and a
failure when fewer than `min_eligible_cases` remain; `evaluate` on synthetic
rows via an injected extractor writes the sealed holdout file, keeps every
holdout metric out of `result.json` and `report.md`, and produces the
three-outcome false-alarm and detection tables; a censored smooth case
below `t_settle_max_s` is `unresolved`, not `detected`.

Existing suites are not modified. Run with `-p no:cacheprovider` and a C
scratch `--basetemp` per `STORAGE.md`.

## Storage and documentation

- Run `storage_preflight` before extraction. Full row files go under
  `C:/Users/Taylor/AppData/Local/Temp/mazda-observed-drive-metrics-20260910/`.
- Compact durable outputs go under
  `.ti-local/observed-drive-metrics-20260910/`: request, parameter file,
  `result.json`, `report.md`, `holdout-sealed.json`, preflight report.
- New guide `tools/mazda_ti/OBSERVED_DRIVE_METRICS.md`; a row in
  `tools/mazda_ti/README.md`; status rows updated in
  `docs/mazda-physical-evidence.md` (measurement work) and
  `docs/mazda-assessment-remediation.md` (implementation status); one
  paragraph in `.agents/skills/mazda-ti-analysis/SKILL.md`.
- No Stash write in this iteration.

## Out of scope

Footprint clearance and calibrated buffers; contact inference; predicted
candidate motion; any candidate comparison; changes to production controller
code; Linux runtime work; the exact-experiment producer contract.
