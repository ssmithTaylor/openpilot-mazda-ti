# Check lane-motion consistency before using road curvature as a target

`audit_lane_motion_balance` compares measured lane-heading/offset changes with
integrated yaw, speed and fitted road geometry. It uses original model
publications, with latest preceding speed/localizer observations. This is
retrospective accounting on recorded states, not a candidate vehicle simulator.

Read [storage policy](STORAGE.md), preflight owned scratch, then run:

```text
python -m tools.mazda_ti.audit_lane_motion_balance --request tools/mazda_ti/cases/lane_motion_balance_vw.json --old-data-root ABSOLUTE-OLD-RAW-ROOT --reference-data-root ABSOLUTE-REFERENCE-RAW-ROOT --output NEW-ABSOLUTE-SCRATCH/result.json
```

The request declares one route per case, exact raw hashes and integer time
bounds. Its selection provenance records the existing geographic anchors and
nearest half-second endpoints; no motion residual selected the windows. Both
root arguments point directly to directories containing route-segment folders.
Raw data are not included in Git. Windows on additional routes need independent
geographic and rider qualification before assigning road names or outcomes.

The audit checks source/raw/runtime hashes before and after extraction. Model
endpoints must be within40ms of the requested bounds. It preserves all selected
rows, separate camera EOF and publication times, input ages and health. Missing
or invalid geometry prevents whole-window integration rather than producing a
zero measurement. Publication gaps above100ms also prevent integration. No lag
is fitted and no future speed/yaw publication is selected. Exit0 means the
audit completed: inspect unavailable rows and each fit result separately.

## Equations and assumptions

The pure helper `lane_motion_balance.py` uses planar motion with no sideslip:

```text
offset_rate = speed * sin(relative_heading)
road_tangent_rate = road_curvature * speed * cos(relative_heading)
                    / (1 - road_curvature * offset)
relative_heading_rate = yaw_rate - road_tangent_rate
```

All signs are right-positive. A positive offset is toward the inside of a
right turn. Trapezoidal integration compares these rates with the observed
endpoint changes. Full interval residuals remain available because a small net
error can hide intermediate departures. The model's fitted coordinates only
approximate Frenet coordinates; camera/model calibration, sideslip, asynchronous
observations, road-fit horizon and estimator filtering remain unqualified.

Yaw-only sensitivity integrates a coherent perturbation of one reported yaw
standard deviation over time. It is not a confidence interval, excludes other
uncertainties and must not be reduced by sqrt(sample_count). The10m/20m fits
share model samples and are sensitivity cases, not independent measurements or
calibrated uncertainty bounds. Neither a small closure residual nor a valid
model confidence flag establishes surveyed road truth.

## First-unwind comparison, September9

The original half-second diagnostic raised a curvature consistency concern.
Repeating at original model rate gives101/111/101 healthy observations for
24d/261/28c. The full-rate check does not remove the fit dependence:

This check does not use liveParameters. Healthy yaw/speed/model observations
therefore do not resolve24d's separate NNFF replay health failure or dirty source.

| Rider case | Heading closure error,10m fit | Heading closure error,20m fit | Mean20m-minus10m curvature × speed² |
|---|---:|---:|---:|
| Smooth24d, similar speed | -1.362° | +0.581° | +0.163m/s² |
| Smooth261, slower | -0.061° | +1.142° | +0.079m/s² |
| Symptomatic28c | -0.021° | +2.184° | +0.180m/s² |

The error is observed heading change minus integrated yaw/road-relative change.
Selecting the10m fit because it agrees on28c would hide its poorer agreement on
24d. These residuals remain compatible with unresolved yaw and geometry
uncertainty; they do not identify a failed sensor or uniquely blame the fit.

The lane-offset balance gives a complementary observation. On28c, model offset
changes0.962m; integrated heading/speed implies0.929m (10m) or1.069m (20m).
Smooth24d changes0.812m versus0.780/0.916m. Both contain inward movement; entry
offset and timing still matter, so these are not standalone symptom scores.
261 offset closure differs by0.156–0.227m and is retained as a limitation.

The0.180m/s² mean fit difference on28c is larger than the current lane-release
helper's0.10m/s² excess-acceleration threshold. This exposes a precision question
for road-curvature-based eligibility; it does not show that the deployed gate
misfired, that one fit is correct, or that the28c controller used that later
helper. The algebraically measured reference retention remains established,
but these measurements do not independently certify an optimal replacement
target. Avoid tuning to a single convenient fit or claiming a physical pass.

Sixteen tests cover circular equilibrium, both directional signs, exact straight
motion, invalid/gapped observations, one-route manifests and integer bounds:

```text
python -m pytest tools/mazda_ti/tests/test_lane_motion_balance.py tools/mazda_ti/tests/test_audit_lane_motion_balance.py --confcutdir=tools/mazda_ti/tests -o addopts= -p no:cacheprovider --basetemp NEW-ABSOLUTE-TEST-SCRATCH -q
```

Shared execution exactly matches all private full-rate rows and fit metrics.
Two fresh public processes produce identical bytes. Retain the request, failures,
source hashes and compact lossless results; rebuild any pruned full evidence
before verification. No steering policy or physical release qualification is
changed by this audit.
