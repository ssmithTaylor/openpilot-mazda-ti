"""Build a hash-bound outcome/controller-feature intersection audit.

This is an evidence index, not a detector and not a controller evaluator.  The
case table is deliberately explicit: a missing setting remains ``null`` and
the record carries an availability reason.  It consumes compact, retained
study records and does not read raw rlogs or generate bulky traces.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


SETTINGS = (
    "LatCommitSetpoint",
    "LatDamping",
    "LatFrictionComp",
    "LatNoFrictionRelay",
    "LatOutputFilter",
    "SteerKP",
    "TiSteerDeltaDown",
    "TiSteerDeltaUp",
    "TiSteerDeltaUpHigh",
    "TiSteerDeltaUpKnee",
    "TiSteerMax",
    "TorqueInterceptorEnabled",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(root: Path, relative: str) -> tuple[dict[str, Any], str]:
    path = root / relative
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected object: {relative}")
    return value, sha256(path)


def settings_from(init: dict[str, Any]) -> dict[str, Any]:
    raw = init.get("settings", {})
    return {key: raw.get(key) for key in SETTINGS}


def compact_settings(init: dict[str, Any]) -> dict[str, Any]:
    return {
        "source_commit": init.get("git_commit"),
        "source_dirty": init.get("dirty"),
        "settings": settings_from(init),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--md", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()

    conditions, conditions_sha = read_json(
        root, ".ti-local/older-vw-video-candidates-20260909/comparison-conditions-v1.json"
    )
    phases, phases_sha = read_json(
        root, ".ti-local/compensation-study-20260909/settling-vs-scallop-phase-v1.json"
    )
    prelabel, prelabel_sha = read_json(
        root, ".ti-local/compensation-study-20260909/route280-prelabel-physical-metrics-v1.json"
    )
    smooth_labels, smooth_labels_sha = read_json(
        root, ".ti-local/older-vw-video-candidates-20260909/rider-unwind-confirmation-20260909.json"
    )
    problem_labels, problem_labels_sha = read_json(
        root, ".ti-local/compensation-study-20260909/route28c-rider-annotations-v1.json"
    )
    settling_label, settling_label_sha = read_json(
        root, ".ti-local/compensation-study-20260909/route280-rider-annotation-v1.json"
    )
    positive_control, positive_control_sha = read_json(
        root, ".ti-local/compensation-study-20260909/route280-positive-control-v1.json"
    )
    rider_28f, rider_28f_sha = read_json(
        root, ".ti-local/curve-review-20260907/post-drive-28f/rider-annotations-v2.json"
    )
    init_28f, init_28f_sha = read_json(
        root, ".ti-local/curve-review-20260907/post-drive-28f/28f-initdata-v1.json"
    )
    rider_28a, rider_28a_sha = read_json(
        root, ".ti-local/curve-review-20260907/rider-event-annotations.json"
    )
    source_audit_path = root / ".ti-local/older-vw-video-candidates-20260909/SOURCE-CONTROLLER-AUDIT.md"
    source_audit_sha = sha256(source_audit_path)

    modern_settings = settings_from(phases["cases"]["28c_cycle1"]["controller_init"][0])
    e409_init = {
        "git_commit": "e409edb8d7ce3766d02379f2f2dafcb5887007ce",
        "dirty": False,
        "settings": {
            "LatCommitSetpoint": "0",
            "LatDamping": "1",
            "LatFrictionComp": "1",
            "LatNoFrictionRelay": "1",
            "LatOutputFilter": "0",
            "SteerKP": "0.800000",
            "TiSteerDeltaDown": "15.000000",
            "TiSteerDeltaUp": "10.000000",
            "TiSteerDeltaUpHigh": "9.000000",
            "TiSteerDeltaUpKnee": "630.000000",
            "TiSteerMax": "600.000000",
            "TorqueInterceptorEnabled": "1",
        },
    }
    e409_commit = e409_init["git_commit"]
    modern_settings_1 = dict(modern_settings)
    modern_settings_1["LatCommitSetpoint"] = "1"

    unknown_old = {key: None for key in SETTINGS}
    cases: list[dict[str, Any]] = []

    def add(
        case_id: str,
        outcome: str,
        labels: list[str],
        source: dict[str, Any],
        *,
        road: str | None,
        lane: str | None,
        speed: dict[str, Any] | None,
        load: dict[str, Any] | None,
        angle: dict[str, Any] | None,
        evidence_level: str,
        rider_confidence: str,
        breaker: str,
        nnff: dict[str, Any],
        lane_release: dict[str, Any],
        notes: list[str],
        provenance: list[str],
    ) -> None:
        cases.append(
            {
                "case_id": case_id,
                "outcome_class": outcome,
                "rider_labels": labels,
                "rider_confidence": rider_confidence,
                "evidence_level": evidence_level,
                "controller": {
                    "source_commit": source.get("git_commit"),
                    "source_dirty": source.get("dirty"),
                    "settings": source.get("settings"),
                    "breaker": breaker,
                    "nnff": nnff,
                    "lane_release": lane_release,
                },
                "operating_context": {
                    "road": road,
                    "lane": lane,
                    "speed_mps": speed,
                    "load_proxy": load,
                    "steering_angle_deg": angle,
                },
                "notes": notes,
                "provenance": provenance,
            }
        )

    c24 = conditions["cases"]["24d"]
    c261 = conditions["cases"]["261"]
    add(
        "24d_segments15-16",
        "smooth",
        ["no_scalloping", "smooth_and_timely_unwinding"],
        {"git_commit": c24["initial_snapshots"][0]["git_commit"], "dirty": True, "settings": unknown_old},
        road="geographically matched VW bend",
        lane="left (rider-confirmed)",
        speed={"window_median": c24["speed_mps"]["median"], "first_second_median": c24["first_second_speed_mps"]["median"]},
        load={"initial_ti_counts": 496, "source": "matched-load inventory; exact event join unavailable"},
        angle={"initial_deg": 15.7, "source": "matched-load inventory; exact event join unavailable"},
        evidence_level="rider outcome plus native command/telemetry window",
        rider_confidence="confirmed",
        breaker="unavailable (legacy NNFF log has no breaker-version field)",
        nnff={"startup_requested": True, "model": None, "runtime_status": "source mapping expects MAZDA_CX9_2021; runtime inner path unavailable"},
        lane_release={"value": None, "reason": "legacy recording has no explicit lane-release field"},
        notes=["Dirty recorded tree; exact uncommitted controller bytes are not reconstructable.", "Lat* settings absent from retained initial snapshot and deliberately left null."],
        provenance=[".ti-local/older-vw-video-candidates-20260909/comparison-conditions-v1.json", ".ti-local/older-vw-video-candidates-20260909/SOURCE-CONTROLLER-AUDIT.md"],
    )
    add(
        "261_segments2-3",
        "smooth",
        ["no_scalloping", "smooth_and_timely_unwinding"],
        {"git_commit": c261["initial_snapshots"][0]["git_commit"], "dirty": False, "settings": unknown_old},
        road="geographically matched VW bend",
        lane="left (rider-confirmed)",
        speed={"window_median": c261["speed_mps"]["median"], "first_second_median": c261["first_second_speed_mps"]["median"]},
        load={"initial_ti_counts": 410, "source": "matched-load inventory; exact event join unavailable"},
        angle={"initial_deg": 14.8, "source": "matched-load inventory; exact event join unavailable"},
        evidence_level="rider outcome plus native command/telemetry window",
        rider_confidence="confirmed",
        breaker="unavailable (legacy NNFF log has no breaker-version field)",
        nnff={"startup_requested": True, "model": None, "runtime_status": "source mapping expects MAZDA_CX9_2021; runtime inner path unavailable"},
        lane_release={"value": None, "reason": "legacy recording has no explicit lane-release field"},
        notes=["Clean recorded source commit.", "Lat* settings absent from retained initial snapshot and deliberately left null."],
        provenance=[".ti-local/older-vw-video-candidates-20260909/comparison-conditions-v1.json", ".ti-local/older-vw-video-candidates-20260909/SOURCE-CONTROLLER-AUDIT.md"],
    )

    route280_context = prelabel["window_metrics"]
    add(
        "280_segment9",
        "settling_without_evident_short_scallop",
        ["held_inward", "over_release_or_failure_to_settle", "path_relatively_smooth"],
        e409_init,
        road="route280; exact road feature name unavailable",
        lane=None,
        speed={"window_median": route280_context["speed_mps"]["median"], "window_min": route280_context["speed_mps"]["min"], "window_max": route280_context["speed_mps"]["max"]},
        load={"ti_median": route280_context["command"]["ti_counts"]["median"], "ti_max": route280_context["command"]["ti_counts"]["max"]},
        angle={"window_median_deg": route280_context["measured_steering"]["angle_deg"]["median"], "window_max_deg": route280_context["measured_steering"]["angle_deg"]["max"]},
        evidence_level="rider outcome plus outcome-blind physical metrics",
        rider_confidence="medium held/over-release; low scallop",
        breaker="not separately versioned; source-only legacy breaker implementation",
        nnff={"startup_requested": False, "model": None, "runtime_status": "NNFF disabled in retained controller init"},
        lane_release={"value": None, "reason": "no explicit rider/event lane-release field in retained record"},
        notes=["Rider could not identify exact hold/release times from Konik presentation.", "Video absence of visible short scallops is not proof of no oscillation."],
        provenance=[".ti-local/compensation-study-20260909/route280-rider-annotation-v1.json", ".ti-local/compensation-study-20260909/route280-prelabel-physical-metrics-v1.json"],
    )
    add(
        "280_segment12_positive_control",
        "smooth_positive_control",
        ["no_scalloping", "smooth_unwind", "no_held_too_long", "no_excessive_release"],
        {"git_commit": positive_control["init_data"][0]["git_commit"], "dirty": positive_control["init_data"][0]["dirty"], "settings": positive_control["init_data"][0]["settings"]},
        road="route280 A-pass1; road feature name unavailable",
        lane="wide lane (rider wording; intended lane not independently calibrated)",
        speed={"approx_peak_mps": 20.35},
        load={"peak_ti_counts": 600},
        angle={"peak_deg": 22.6},
        evidence_level="rider outcome plus native command/telemetry window",
        rider_confidence="confirmed with lane-width caveat",
        breaker="not separately versioned; source-only legacy breaker implementation",
        nnff={"startup_requested": False, "model": None, "runtime_status": "NNFF disabled in retained controller init"},
        lane_release={"value": None, "reason": "no explicit lane-release setting in retained record"},
        notes=["Positive control is high-demand but does not match 28c speed/angle/load envelope."],
        provenance=[".ti-local/compensation-study-20260909/route280-positive-control-v1.json"],
    )

    for cycle_id, clip in (("28c_preceding_hold", "~17 s"), ("28c_cycle1", "47-49 s"), ("28c_cycle2", "54-57 s")):
        phase = phases["cases"]["28c_main" if cycle_id == "28c_preceding_hold" else cycle_id]
        # The retained phase artifact is not event-resolved to the preceding
        # rider bracket. Do not project full-cycle speed/load/angle onto it.
        phase_context = cycle_id == "28c_preceding_hold"
        add(
            cycle_id,
            "scalloping_problem" if cycle_id != "28c_preceding_hold" else "problem_held_inward",
            ["scallop", "held_inward", "over_release"] if cycle_id != "28c_preceding_hold" else ["held_inward"],
            {"git_commit": phase["controller_init"][0]["git_commit"], "dirty": phase["controller_init"][0]["dirty"], "settings": phase["controller_init"][0]["settings"]},
            road="VW factory curve / preceding bend",
            lane=None,
            speed=None if phase_context else {"window_median_mps": 24.718},
            load=None if phase_context else {"ti_max": phase["phase"]["ranges"]["ti"]["max"], "ti_median": phase["phase"]["ranges"]["ti"]["median"]},
            angle=None if phase_context else {"window_min_deg": phase["phase"]["ranges"]["wheel_angle"]["min"], "window_max_deg": phase["phase"]["ranges"]["wheel_angle"]["max"]},
            evidence_level="rider-labeled inspection plus native/reconstructed telemetry; exact contact unavailable",
            rider_confidence="medium; visual scallop/over-release timing uncertain",
            breaker="not separately versioned; diagnostic breaker rows zero in retained phase windows",
            nnff={"startup_requested": False, "model": None, "runtime_status": "NNFF disabled in retained controller init"},
            lane_release={"value": None, "reason": "lane permission is not a rider lane-release label and exact apply identity is unavailable"},
            notes=[f"Rider inspection bracket: {clip}.", "Preceding-hold speed/load/angle are unavailable because the retained phase record is not event-resolved." if phase_context else "Both cycles show command decline before wheel/inertial response; this is not a physical causality claim."],
            provenance=[".ti-local/compensation-study-20260909/route28c-rider-annotations-v1.json", ".ti-local/compensation-study-20260909/settling-vs-scallop-phase-v1.json"],
        )

    add(
        "28a_segment47",
        "problem_oversteer_with_intervention",
        ["oversteer", "intervention"],
        {"git_commit": e409_commit, "dirty": False, "settings": modern_settings_1},
        road="route28a curve; exact road feature name unavailable",
        lane=None,
        speed=None,
        load=None,
        angle=None,
        evidence_level="rider-confirmed intervention; command replay only for controller context",
        rider_confidence="approximate intervention timing",
        breaker="not separately versioned; e409 source implementation",
        nnff={"startup_requested": False, "model": None, "runtime_status": "NNFF disabled in supplied replay settings"},
        lane_release={"value": None, "reason": "no rider lane-release label"},
        notes=["Rider says oversteering; intervention near ~7 s, contact end unknown."],
        provenance=[".ti-local/curve-review-20260907/rider-event-annotations.json", ".ti-local/curve-review-20260907/post-drive-28f/friction-boundary-02/28a47/run.json"],
    )
    add(
        "28a_segment48",
        "problem_oversteer_with_intervention",
        ["oversteer", "intervention"],
        {"git_commit": e409_commit, "dirty": False, "settings": modern_settings_1},
        road="route28a curve; exact road feature name unavailable",
        lane=None,
        speed=None,
        load=None,
        angle=None,
        evidence_level="rider-confirmed intervention; command replay only for controller context",
        rider_confidence="approximate intervention timing",
        breaker="not separately versioned; e409 source implementation",
        nnff={"startup_requested": False, "model": None, "runtime_status": "NNFF disabled in supplied replay settings"},
        lane_release={"value": None, "reason": "no rider lane-release label"},
        notes=["Rider intervention about 4 s into clip; later wheel motion is driver-influenced."],
        provenance=[".ti-local/curve-review-20260907/rider-event-annotations.json", ".ti-local/curve-review-20260907/post-drive-28f/friction-boundary-02/28a45/run.json"],
    )
    add(
        "28f_segment13_return_850-852",
        "problem_lane_boundary_crossing",
        ["unassisted_lane_boundary_crossing", "autonomous_recovery_does_not_erase_failure"],
        {"git_commit": init_28f["git_commit"], "dirty": init_28f["dirty"], "settings": settings_from(init_28f)},
        road="return right bends / left bend approach",
        lane="left lane to right lane across dashed divider (rider-confirmed)",
        speed={"window_min_mps": 19.909, "window_max_mps": 20.750},
        load=None,
        angle=None,
        evidence_level="rider-confirmed physical outcome; source/deployment record supplies controller context",
        rider_confidence="confirmed outcome; approximate timing",
        breaker="not separately versioned; source rlog has no breaker-version field",
        nnff={"startup_requested": False, "model": init_28f["settings"].get("Model"), "runtime_status": "NNFF=0 and NNFFLite=0 in recorded initData"},
        lane_release={"value": None, "reason": "no rider lane-release setting; lane validity is separate"},
        notes=["Contact absent for this 0-2 s interval; later catches are not independently hands-off."],
        provenance=[".ti-local/curve-review-20260907/post-drive-28f/rider-annotations-v2.json", ".ti-local/curve-review-20260907/post-drive-28f/FINDINGS.md", ".ti-local/curve-review-20260907/post-drive-28f/28f-initdata-v1.json"],
    )

    feature_values = {
        key: sorted({case["controller"]["settings"].get(key) for case in cases if case["controller"]["settings"].get(key) is not None})
        for key in SETTINGS
    }
    conclusions = [
        {"feature": "LatCommitSetpoint", "finding": "Not sufficient or necessary by itself in this corpus: the symptomatic 28c sequence has 1, while route280 segment9 settling without evident short scallop and route280 positive control have 0; era, load and source differ.", "status": "confounded"},
        {"feature": "LatDamping/LatFrictionComp/LatOutputFilter", "finding": "The retained modern smooth, settling and problem records all use damping=1, friction compensation=1 and output filter=0; these settings have no outcome separation here.", "status": "common_across_modern_cases"},
        {"feature": "NNFF", "finding": "Old smooth controls requested NNFF while modern symptomatic/settling records have NNFF disabled; controller era and all associated source/settings changes are confounded, so no NNFF benefit or harm is isolated.", "status": "era_confounded"},
        {"feature": "breaker", "finding": "No retained rider record carries an explicit breaker version. Diagnostic breaker rows are zero in the 28c phase windows, which does not prove the feature was absent globally or physically irrelevant.", "status": "unavailable"},
        {"feature": "lane_release", "finding": "No rider-labeled case supplies a controller lane-release setting that can be intersected with outcome. Model lane permission and rider lane containment remain separate evidence.", "status": "unavailable"},
    ]
    source_paths = {
        ".ti-local/older-vw-video-candidates-20260909/comparison-conditions-v1.json": conditions_sha,
        ".ti-local/compensation-study-20260909/settling-vs-scallop-phase-v1.json": phases_sha,
        ".ti-local/compensation-study-20260909/route280-prelabel-physical-metrics-v1.json": prelabel_sha,
        ".ti-local/older-vw-video-candidates-20260909/rider-unwind-confirmation-20260909.json": smooth_labels_sha,
        ".ti-local/compensation-study-20260909/route28c-rider-annotations-v1.json": problem_labels_sha,
        ".ti-local/compensation-study-20260909/route280-rider-annotation-v1.json": settling_label_sha,
        ".ti-local/compensation-study-20260909/route280-positive-control-v1.json": positive_control_sha,
        ".ti-local/curve-review-20260907/post-drive-28f/rider-annotations-v2.json": rider_28f_sha,
        ".ti-local/curve-review-20260907/post-drive-28f/28f-initdata-v1.json": init_28f_sha,
        ".ti-local/curve-review-20260907/rider-event-annotations.json": rider_28a_sha,
        ".ti-local/older-vw-video-candidates-20260909/SOURCE-CONTROLLER-AUDIT.md": source_audit_sha,
    }
    source_paths["tools/mazda_ti/feature_intersection_audit.py"] = sha256(Path(__file__).resolve())
    result = {
        "format_version": 1,
        "generated_by": "tools/mazda_ti/feature_intersection_audit.py",
        "scope": "Explicit rider-labeled smooth, settling-without-evident-short-scallop, positive-control, and problem passes found in retained local Mazda evidence.",
        "source_sha256": source_paths,
        "cases": cases,
        "feature_value_intersection": feature_values,
        "findings": conclusions,
        "limits": [
            "Route-level labels are not event labels; each case preserves its rider bracket and confidence.",
            "Unknown settings are null; no value is inferred from controller era, source commit, or default behavior.",
            "Road/lane geometry is model-derived or rider-described unless explicitly marked rider-confirmed.",
            "Controller commands, predicted response, and demonstrated road behavior remain separate evidence levels.",
            "Legacy 24d/261 logs lack exact Lat* settings, breaker identity, lane-release fields and NNFF inner-path identity.",
            "This audit does not select a production change and does not establish physical causality.",
        ],
    }
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.md.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = [
        "# Mazda rider outcome/controller-feature intersection audit",
        "",
        f"Generated by `{result['generated_by']}`. JSON artifact: `feature-intersection-audit-v1.json`.",
        "",
        "The table is an evidence index. Null means the retained source did not record the field; it is not a default.",
        "",
        "| Case | Outcome | Source | Commit | Damping | Friction | Commit-setpoint | NNFF | Speed/load/angle | Road/lane |",
        "|---|---|---|---:|---:|---:|---:|---|---|---|",
    ]
    for case in cases:
        settings = case["controller"]["settings"]
        context = case["operating_context"]
        source = case["controller"]["source_commit"] or "unknown"
        nnff_value = case["controller"]["nnff"].get("runtime_status", "unknown")
        lines.append(
            "| {case} | {outcome} | {source} | {commit} | {damping} | {friction} | {setpoint} | {nnff} | {context} | {road} / {lane} |".format(
                case=case["case_id"], outcome=case["outcome_class"], source="recorded", commit=source[:12],
                damping=settings.get("LatDamping"), friction=settings.get("LatFrictionComp"), setpoint=settings.get("LatCommitSetpoint"),
                nnff=nnff_value, context=json.dumps(context, separators=(",", ":")), road=context.get("road"), lane=context.get("lane") or "unknown"
            )
        )
    lines.extend(["", "## Findings", ""])
    for finding in conclusions:
        lines.append(f"- **{finding['feature']}** ({finding['status']}): {finding['finding']}")
    lines.extend(["", "## Limitations", ""])
    lines.extend(f"- {limit}" for limit in result["limits"])
    args.md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
